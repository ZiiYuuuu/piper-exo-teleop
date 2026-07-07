import socket
import time
# ===================== TCP参数（与openarm_mini_m5_tcp保持一致） =====================
TCP_IP = "192.168.4.1"
TCP_PORT = 10000
SOCKET_TIMEOUT = 2
# 初始化TCP客户端
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(SOCKET_TIMEOUT)
client.connect((TCP_IP, TCP_PORT))
time.sleep(0.01)  # 等待连接稳定

def send_and_receive_robust(cmd_bytes):
    """复刻openarm_mini_m5_tcp可靠收发逻辑，读取完整37字节关节帧"""
    client.sendall(cmd_bytes)
    start_wait = time.time()
    # 等待37字节完整数据，10ms超时
    recv_buf = b""
    while len(recv_buf) < 37:
        if time.time() - start_wait > 0.01:
            print("\033[31m[超时] 未收到完整37字节关节数据\033[0m")
            return None
        chunk = client.recv(37 - len(recv_buf))
        if not chunk:
            print("\033[31m[连接断开] 无数据返回\033[0m")
            return None
        recv_buf += chunk
    read_bytes = recv_buf
    read_hex = read_bytes.hex()
    # 校验帧头 FD FD
    if read_hex[0:2] != "fd" or read_hex[2:4] != "fd":
        print(f"\033[31m[帧错误] 非法帧头，原始数据：{read_hex}\033[0m")
        return None
    payload_hex = read_hex[8:-2]
    return payload_hex

def get_all_joint_angles():
    """读取双臂16路关节角度：右臂8个 + 左臂8个"""
    read_cmd = bytes([0xFD, 0xFD, 0x02, 0x01, 0xFE])
    payload = send_and_receive_robust(read_cmd)
    if payload is None:
        return None
    
    joint_angles = []
    for i in range(16):
        offset = i * 4
        seg = payload[offset:offset+4]
        # 高低字节反转
        raw_code = int(seg[2:4] + seg[0:2], 16)
        # 转换为±180°角度
        if raw_code == 2048:
            angle = 0.0
        elif raw_code > 2048:
            angle = 180 * (raw_code - 2048) / 2048
        else:
            angle = -180 * (2048 - raw_code) / 2048
        joint_angles.append(round(angle, 2))
    return joint_angles

def print_joint_info(angles):
    """格式化打印左右臂所有关节角度"""
    if angles is None:
        print("读取关节数据失败！")
        return
    right = angles[:8]
    left = angles[8:]
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"]
    print("========== 右臂关节角度 ==========")
    for name, angle in zip(joint_names, right):
        print(f"{name:8s}: {angle:6.2f} °")
    print("========== 左臂关节角度 ==========")
    for name, angle in zip(joint_names, left):
        print(f"{name:8s}: {angle:6.2f} °")
    print("-" * 45 + "\n")

# ===================== 主循环：仅持续读取打印关节角度 =====================
if __name__ == "__main__":
    print("=== TCP持续读取双臂关节角度，Ctrl+C 退出程序 ===")
    try:
        while True:
            angles = get_all_joint_angles()
            print_joint_info(angles)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n程序退出，关闭TCP连接")
    finally:
        client.close()
