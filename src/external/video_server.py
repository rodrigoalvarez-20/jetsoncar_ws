import pickle
import socket
import struct

import cv2

HOST = "192.168.1.14"
PORT = 8089

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')

s.bind((HOST, PORT))
print('Socket bind complete')
s.listen(10)
print('Socket now listening')

conn, addr = s.accept()
payload_size = struct.calcsize(">L") 
data = b""
while True:
    # Receive the packed size
    while len(data) < payload_size:
        data += conn.recv(4096) # Adjust buffer size as needed
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]

    while len(data) < msg_size:
        data += conn.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]
    
    # Deserialize the numpy array of the encoded image
    encoded_frame_array = pickle.loads(frame_data, fix_imports=True, encoding="bytes")

    # Decode the image data into an OpenCV frame
    frame = cv2.imdecode(encoded_frame_array, cv2.IMREAD_COLOR) 

    # Display the frame
    cv2.imshow('Received Frame', frame)
    cv2.waitKey(1)