import socket
import cv2

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
message ="hello"
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.sendto(message, (UDP_IP,UDP_PORT))
