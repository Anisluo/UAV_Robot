#!/usr/bin/env python3
import socket
import json
import sys


def call(method, params=None):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(15.0)
    s.connect("/tmp/uav_proc_arm.sock")
    payload = {"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}
    s.sendall((json.dumps(payload) + "\n").encode())
    data = b""
    while b"\n" not in data:
        c = s.recv(4096)
        if not c:
            break
        data += c
    s.close()
    return json.loads(data.decode().splitlines()[0]) if data else None


if __name__ == "__main__":
    method = sys.argv[1]
    params = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    print(json.dumps(call(method, params), indent=2))
