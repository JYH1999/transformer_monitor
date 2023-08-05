import socket

# 创建一个 UDP 套接字
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    # 从键盘获取数据
    send_data = input("请输入要发送的数据：")
    # 如果输入的数据是exit, 就退出程序
    if send_data == "exit":
        break
    # 可以使用套接字收发数据
    udp_socket.sendto(b"Hello World!", ("8.136.87.151", 4950))
    udp_socket.sendto(send_data.encode("utf-8"), ("8.136.87.151", 4950))
# 关闭套接字
udp_socket.close()