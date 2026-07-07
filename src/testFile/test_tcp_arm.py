#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import time

ip = "192.168.4.1"
port = 10000

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(2.0)

print(f"正在连接 {ip}:{port} ...")
client.connect((ip, port))
print("✅ TCP 已连接")

time.sleep(0.01)


def _commmon(hex_command):
    """
    发送十六进制字符串到 TCP socket，然后读取返回
    例如: _commmon("FD FD 04 04 01 02 FE")
    """

    cmd_bytes = bytes.fromhex(hex_command)

    client.sendall(cmd_bytes)
    print(f"✅ 发送: {hex_command}")

    try:
        read_bytes = client.recv(1024)
    except socket.timeout:
        print("❌ 接收超时，无返回\n")
        return ""

    read_hex = read_bytes.hex(" ")

    print(f"📥 返回: {read_hex}\n")

    return read_hex


commands = [
    "FD FD 04 04 01 02 FE",
    "FD FD 04 04 01 03 FE",
    "FD FD 04 04 01 04 FE",
    "FD FD 04 04 01 05 FE",
    "FD FD 04 04 01 06 FE",
    "FD FD 04 04 01 07 FE",
    "FD FD 04 04 01 08 FE",
    "FD FD 04 04 01 01 FE",

    "FD FD 04 04 02 02 FE",
    "FD FD 04 04 02 03 FE",
    "FD FD 04 04 02 04 FE",
    "FD FD 04 04 02 05 FE",
    "FD FD 04 04 02 06 FE",
    "FD FD 04 04 02 07 FE",
    "FD FD 04 04 02 08 FE",
    "FD FD 04 04 02 01 FE",
]


for cmd in commands:
    _commmon(cmd)
    time.sleep(0.02)


client.close()
print("已关闭连接")
