#!/usr/bin/env python3
"""CSI radar: raw ESP32 CSI over UDP -> detection (detect.py) -> browsers via Server-Sent Events."""
import json, os, queue, re, socket, threading, time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import detect

UDP_PORT, CMD_PORT = 5005, 5006
HTTP_PORT = int(os.environ.get("PORT", 8765))
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.environ.get("DATA", os.path.join(HERE, "data"))
STATE_FILE = os.path.join(DATA, "state.json")
INDEX = open(os.path.join(HERE, "index.html"), "rb").read()

REC_DT = np.dtype([("ms", "<u4"), ("rssi", "i1"), ("iq", "i1", (104,))])  # matches firmware sample_t
RING = 3200                 # ~32 s of packets
MOTION_WIN = 100            # ~1 s for the motion statistic
HOLD_S = 1.0                # motion stays on this long after the statistic drops
CAL_S, REC_S = 10, 30
LABELS = ("empty", "walking", "still")
# the page is public: only these commands pass
CMD_RE = re.compile(r"^(cal|apply|thr 0?\.\d{1,4}|rec (empty|walking|still))$")
DEFAULT_CONFIG = {"room": [4.0, 4.0, 2.7], "pi": [0.3, 0.3, 0.8], "esp": [3.7, 3.7, 0.8]}

os.makedirs(os.path.join(DATA, "rec"), exist_ok=True)
try:
    state = json.load(open(STATE_FILE))
except (OSError, ValueError):
    state = {}
state.setdefault("thr", 0.2)
state.setdefault("config", DEFAULT_CONFIG)
state.setdefault("rec", {})          # label -> list of motion_stat samples
state.setdefault("tuned", None)

lock = threading.Lock()
clients = set()
history = deque(maxlen=600)          # ~60 s of frames for late joiners
events = deque(maxlen=200)
esp_addr, last_seen = None, 0.0
job = None                           # {"kind": "cal"|"rec", "label", "until", "ms": [], "raw": []}


def save_state():
    tmp = STATE_FILE + ".tmp"
    json.dump(state, open(tmp, "w"))
    os.replace(tmp, STATE_FILE)


def meta():
    rec = {k: {"n": len(v), "p50": round(float(np.percentile(v, 50)), 3), "p99": round(float(np.percentile(v, 99)), 3)}
           for k, v in state["rec"].items() if v}
    return {"thr": state["thr"], "config": state["config"], "tuned": state["tuned"], "rec": rec}


def broadcast(kind, payload):
    msg = f"event: {kind}\ndata: {json.dumps(payload)}\n\n".encode()
    with lock:
        for q in list(clients):
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass


def led(on):
    if esp_addr:
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(b"led 1" if on else b"led 0", (esp_addr, CMD_PORT))


def finish_job(j):
    if j["kind"] == "cal":
        # ponytail: assumes the room is empty during calibration; the record/tune flow is the precise way
        state["thr"] = round(float(np.clip(np.percentile(j["ms"], 99) * 1.5, 0.12, 0.6)), 4)
    else:
        state["rec"][j["label"]] = [round(v, 4) for v in j["ms"]]
        raw = np.concatenate(j["raw"])
        files = sorted(os.listdir(os.path.join(DATA, "rec")))
        for old in files[:-29]:  # keep the newest 30 recordings
            os.remove(os.path.join(DATA, "rec", old))
        np.save(os.path.join(DATA, "rec", f"{time.strftime('%Y%m%d-%H%M%S')}-{j['label']}.npy"), raw)
        if state["rec"].get("empty") and state["rec"].get("walking"):
            thr, ok = detect.tune(state["rec"]["empty"], state["rec"]["walking"])
            state["tuned"] = {"thr": round(thr, 4), "separable": ok}
    save_state()
    broadcast("meta", meta())


def udp_loop(sock):
    global esp_addr, last_seen, job
    t_ring = np.zeros(RING)
    na_ring = np.zeros((RING, 52), np.float32)
    n_seen = 0
    motion, hold_until, last_spec = False, 0.0, 0.0
    bpm = prev_b = None
    streak = 0
    tones = []
    pps_t, pps_n, pps = time.time(), 0, 0
    last_motion_t = 0.0
    while True:
        data, addr = sock.recvfrom(4096)
        if data[:3] != b"CS\x02" or len(data) != 4 + data[3] * REC_DT.itemsize:
            continue
        recs = np.frombuffer(data, REC_DT, count=data[3], offset=4)
        now = time.time()
        esp_addr, last_seen = addr[0], now
        na = detect.amplitudes(recs["iq"].reshape(-1, 52, 2))
        k = len(recs)
        idx = (n_seen + np.arange(k)) % RING
        t_ring[idx] = recs["ms"] / 1000.0
        na_ring[idx] = na
        n_seen += k
        pps_n += k
        if now - pps_t >= 1:
            pps, pps_n, pps_t = round(pps_n / (now - pps_t)), 0, now

        def last(n):
            i = (n_seen - min(n, n_seen) + np.arange(min(n, n_seen))) % RING
            return t_ring[i], na_ring[i]

        ms = detect.motion_stat(last(MOTION_WIN)[1])
        thr = state["thr"]
        if ms > thr:
            hold_until = now + HOLD_S
            last_motion_t = now
        new_motion = now < hold_until
        if new_motion != motion:
            motion = new_motion
            led(motion)
            if motion:
                events.append({"start": now, "end": None})
            elif events and events[-1]["end"] is None:
                events[-1]["end"] = now
            broadcast("events", list(events))

        if now - last_spec >= 1 and n_seen >= 2000:
            last_spec = now
            t, x = last(RING)
            back = np.flatnonzero(np.diff(t) < 0)    # ESP rebooted (or clock wrapped): only use the latest run
            if len(back):
                t, x = t[back[-1] + 1:], x[back[-1] + 1:]
            spec = detect.spectrum(t, x)
            b, _ = detect.breathing(spec)
            if now - last_motion_t < 30:              # gross motion swamps breathing for the whole window
                b = None
            # a single prominent peak can be noise; only report a rate that holds (+-2/min) for 5 checks in a row
            streak = streak + 1 if b and prev_b and abs(b - prev_b) <= 2 else (1 if b else 0)
            prev_b = b
            bpm = b if streak >= 5 else None
            tones = detect.tones(spec)

        j = job
        progress = None
        if j:
            j["ms"].append(ms)
            if j["kind"] == "rec":
                j["raw"].append(recs.copy())
            total = CAL_S if j["kind"] == "cal" else REC_S
            progress = {"kind": j["kind"], "label": j.get("label"), "pct": min(100, round(100 * (1 - (j["until"] - now) / total)))}
            if now >= j["until"]:
                job = None
                finish_job(j)

        amp = np.hypot(recs["iq"][:, 0::2].astype(np.float32), recs["iq"][:, 1::2].astype(np.float32)).mean(0)
        f = {"t": now, "ms": round(ms, 4), "thr": thr, "motion": int(motion), "rssi": int(recs["rssi"][-1]),
             "pps": pps, "amp": [int(a) for a in amp], "bpm": bpm, "tones": tones, "job": progress}
        history.append(f)
        broadcast("frame", f)


def heartbeat():
    while True:
        time.sleep(2)
        broadcast("status", {"online": time.time() - last_seen < 3})


def valid_config(c):
    try:
        room, pi, esp = ([float(v) for v in c[k]] for k in ("room", "pi", "esp"))
    except (KeyError, TypeError, ValueError):
        return None
    if len(room) != 3 or len(pi) != 3 or len(esp) != 3 or not all(0.5 <= v <= 30 for v in room):
        return None
    if not all(0 <= p[i] <= room[i] for p in (pi, esp) for i in range(3)):
        return None
    return {"room": room, "pi": pi, "esp": esp}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def body(self, limit):
        n = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(min(n, limit)).decode(errors="replace").strip()

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(INDEX)))
            self.end_headers()
            self.wfile.write(INDEX)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            q = queue.Queue(maxsize=200)
            init = {"history": list(history), "events": list(events), "online": time.time() - last_seen < 3, "meta": meta()}
            q.put(f"event: init\ndata: {json.dumps(init)}\n\n".encode())
            with lock:
                clients.add(q)
            try:
                while True:
                    try:
                        self.wfile.write(q.get(timeout=15))
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except OSError:
                pass
            finally:
                with lock:
                    clients.discard(q)
        else:
            self.send_error(404)

    def do_POST(self):
        global job
        if self.path == "/cmd":
            cmd = self.body(64)
            if not CMD_RE.match(cmd):
                return self.send_error(400)
            if cmd == "cal" or cmd.startswith("rec "):
                if job:
                    return self.send_error(409)
                kind = "cal" if cmd == "cal" else "rec"
                job = {"kind": kind, "label": cmd[4:] if kind == "rec" else None,
                       "until": time.time() + (CAL_S if kind == "cal" else REC_S), "ms": [], "raw": []}
            elif cmd == "apply":
                if not state["tuned"]:
                    return self.send_error(409)
                state["thr"] = state["tuned"]["thr"]
            else:
                v = float(cmd[4:])
                if not 0.02 <= v <= 0.95:
                    return self.send_error(400)
                state["thr"] = v
            save_state()
            broadcast("meta", meta())
        elif self.path == "/config":
            try:
                c = valid_config(json.loads(self.body(512)))
            except ValueError:
                c = None
            if not c:
                return self.send_error(400)
            state["config"] = c
            save_state()
            broadcast("meta", meta())
        else:
            return self.send_error(404)
        self.send_response(204)
        self.end_headers()


if __name__ == "__main__":
    assert CMD_RE.match("thr 0.25") and CMD_RE.match("rec walking") and not CMD_RE.match("thr 5") and not CMD_RE.match("rec x")
    assert valid_config(DEFAULT_CONFIG) and not valid_config({"room": [4, 4, 2.7], "pi": [9, 0, 0], "esp": [1, 1, 1]})
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("0.0.0.0", UDP_PORT))
    threading.Thread(target=udp_loop, args=(udp,), daemon=True).start()
    threading.Thread(target=heartbeat, daemon=True).start()
    print(f"udp :{UDP_PORT}  http :{HTTP_PORT}  data {DATA}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()
