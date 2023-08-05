import socket

# 创建一个 UDP 套接字
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 绑定一个本地信息
local_address = ("0.0.0.0", 4950)
# 必须绑定自己电脑的 ip 以及 port，其他的不行
udp_socket.bind(local_address)
while True:
    # 接收数据
    receive_data = udp_socket.recvfrom(1024)
    # receive_data 这个变量存储的是一个元组，(接收到的数据，(发送方的ip, port))
    print(receive_data)
    print("接收到的信息：%s， 发送方地址信息： %s" % (receive_data[0].decode("utf-8"), str(receive_data[1])))
# 关闭套接字
udp_socket.close()