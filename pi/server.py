#!/usr/bin/env python3
"""CSI radar relay: ESP32 UDP frames -> browsers via Server-Sent Events. Stdlib only."""
import json, os, queue, re, socket, threading, time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UDP_PORT, CMD_PORT = 5005, 5006
HTTP_PORT = int(os.environ.get("PORT", 8765))
INDEX = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html"), "rb").read()
CMD_RE = re.compile(r"^(cal|thr \d+(\.\d+)?([eE]-?\d+)?)$")  # the page is public: only these two commands pass

history = deque(maxlen=600)   # ~60 s of frames for clients that join late
events = deque(maxlen=200)    # {"start": epoch, "end": epoch|None}
clients, lock = set(), threading.Lock()
esp_addr = None
last_seen = 0.0


def broadcast(kind, payload):
    msg = f"event: {kind}\ndata: {json.dumps(payload)}\n\n".encode()
    with lock:
        for q in list(clients):
            try:
                q.put_nowait(msg)
            except queue.Full:  # slow client; it'll catch up on the next frames
                pass


def udp_loop(sock):
    global esp_addr, last_seen
    while True:
        data, addr = sock.recvfrom(2048)
        try:
            f = json.loads(data)
        except ValueError:
            continue
        esp_addr, last_seen = addr[0], time.time()
        f["t"] = last_seen
        prev = history[-1]["motion"] if history else 0
        history.append(f)
        if f["motion"] and not prev:
            events.append({"start": last_seen, "end": None})
            broadcast("events", list(events))
        elif prev and not f["motion"] and events and events[-1]["end"] is None:
            events[-1]["end"] = last_seen
            broadcast("events", list(events))
        broadcast("frame", f)


def heartbeat():
    while True:
        time.sleep(2)
        broadcast("status", {"online": time.time() - last_seen < 3, "esp": esp_addr})


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

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
            self.send_header("X-Accel-Buffering", "no")  # stop reverse proxies from buffering the stream
            self.end_headers()
            q = queue.Queue(maxsize=200)
            init = {"history": list(history), "events": list(events),
                    "online": time.time() - last_seen < 3}
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
        if self.path != "/cmd":
            return self.send_error(404)
        body = self.rfile.read(min(int(self.headers.get("Content-Length", 0)), 64)).decode(errors="replace").strip()
        if not CMD_RE.match(body) or not esp_addr:
            return self.send_error(400 if esp_addr else 503)
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(body.encode(), (esp_addr, CMD_PORT))
        self.send_response(204)
        self.end_headers()


if __name__ == "__main__":
    assert CMD_RE.match("cal") and CMD_RE.match("thr 2.5e-05") and not CMD_RE.match("thr 1; rm")
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("0.0.0.0", UDP_PORT))
    threading.Thread(target=udp_loop, args=(udp,), daemon=True).start()
    threading.Thread(target=heartbeat, daemon=True).start()
    print(f"udp :{UDP_PORT}  http :{HTTP_PORT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()
