FROM python:3.13-slim
RUN pip install --no-cache-dir numpy==2.2.6
WORKDIR /app
COPY server.py detect.py index.html ./
RUN python3 detect.py
ENV DATA=/data
CMD ["python3", "-u", "server.py"]
