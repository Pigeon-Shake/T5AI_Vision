#!/usr/bin/env python3
"""Serial console for the T5AI camera gimbal project.

monitor: stream board log to stdout/file and serve a local TCP control port
         so commands can be injected while monitoring.
send:    inject one command (via the monitor daemon, or directly if absent)
         and print the board reply.
shell:   interactive terminal for a human operator.
"""
import argparse
import os
import queue
import socket
import sys
import threading
import time

import serial

DEFAULT_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
DEFAULT_BAUD = int(os.environ.get("SERIAL_BAUD", "460800"))
DEFAULT_CTL = 7777
IDLE_CLOSE_S = 1.5
MAX_REPLY_S = 15.0


def open_serial(port, baud):
    return serial.Serial(port, baud, timeout=0.1, write_timeout=2)


def reader_loop(ser, sinks, clients, lock):
    while True:
        try:
            data = ser.read(4096)
        except serial.SerialException as exc:
            sys.stderr.write(f"serial read error: {exc}\n")
            return
        if not data:
            continue
        for sink in sinks:
            sink.write(data)
            sink.flush()
        with lock:
            for q in clients:
                q.put(data)


def ctl_loop(ser, ctl_port, clients, lock):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", ctl_port))
    srv.listen(4)
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=handle_client, args=(conn, ser, clients, lock), daemon=True).start()


def handle_client(conn, ser, clients, lock):
    q = queue.Queue()
    with lock:
        clients.append(q)
    try:
        conn.settimeout(MAX_REPLY_S)
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = conn.recv(1024)
            if not chunk:
                return
            buf += chunk
        cmd = buf.decode(errors="ignore").strip()
        if cmd:
            ser.write((cmd + "\r\n").encode())
        deadline = time.time() + MAX_REPLY_S
        while time.time() < deadline:
            try:
                data = q.get(timeout=IDLE_CLOSE_S)
            except queue.Empty:
                break
            conn.sendall(data)
    except (OSError, serial.SerialException):
        pass
    finally:
        with lock:
            clients.remove(q)
        conn.close()


def cmd_monitor(args):
    ser = open_serial(args.port, args.baud)
    sinks = [sys.stdout.buffer]
    if args.out:
        sinks.append(open(args.out, "ab", buffering=0))
    clients, lock = [], threading.Lock()
    threading.Thread(target=reader_loop, args=(ser, sinks, clients, lock), daemon=True).start()
    threading.Thread(target=ctl_loop, args=(ser, args.ctl, clients, lock), daemon=True).start()
    sys.stderr.write(f"monitoring {args.port}@{args.baud}, ctl 127.0.0.1:{args.ctl}, out={args.out}\n")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


def collect_reply(ser, seconds_idle=IDLE_CLOSE_S, max_s=MAX_REPLY_S):
    deadline = time.time() + max_s
    last = time.time()
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
            last = time.time()
        elif time.time() - last > seconds_idle:
            break
    sys.stdout.buffer.write(b"\n")
    sys.stdout.buffer.flush()


def cmd_send(args):
    line = " ".join(args.cmd)
    try:
        conn = socket.create_connection(("127.0.0.1", args.ctl), timeout=2)
    except OSError:
        conn = None
    if conn is not None:
        with conn:
            conn.sendall((line + "\n").encode())
            conn.settimeout(MAX_REPLY_S + 2)
            while True:
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    break
                if not data:
                    break
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
        return
    ser = open_serial(args.port, args.baud)
    try:
        ser.reset_input_buffer()
        ser.write((line + "\r\n").encode())
        collect_reply(ser)
    finally:
        ser.close()


def cmd_shell(args):
    ser = open_serial(args.port, args.baud)
    clients, lock = [], threading.Lock()
    threading.Thread(target=reader_loop, args=(ser, [sys.stdout.buffer], clients, lock), daemon=True).start()
    sys.stderr.write(f"shell {args.port}@{args.baud}, ctrl-c to quit\n")
    try:
        for line in sys.stdin:
            ser.write((line.rstrip("\n") + "\r\n").encode())
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", default=DEFAULT_PORT)
    common.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    sub = ap.add_subparsers(dest="mode", required=True)
    m = sub.add_parser("monitor", help="stream log + TCP command injection", parents=[common])
    m.add_argument("--out", default="monitor.log")
    m.add_argument("--ctl", type=int, default=DEFAULT_CTL)
    m.set_defaults(fn=cmd_monitor)
    s = sub.add_parser("send", help="send one command and print reply", parents=[common])
    s.add_argument("cmd", nargs="+")
    s.add_argument("--ctl", type=int, default=DEFAULT_CTL)
    s.set_defaults(fn=cmd_send)
    h = sub.add_parser("shell", help="interactive terminal", parents=[common])
    h.set_defaults(fn=cmd_shell)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
