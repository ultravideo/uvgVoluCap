# ZMQ Receiver (Windows)
import zmq

context = zmq.Context()
socket_c = context.socket(zmq.PULL)
socket_c.bind("tcp://*:5558")  # Listen on all interfaces on port 5555

socket_p = context.socket(zmq.PULL)
socket_p.bind("tcp://*:5559")

print("Waiting for messages...")
while True:
    msg_c = socket_c.recv_string()
    print("Received:", msg_c)
    msg_p = socket_p.recv_string()
    print("Received:", msg_p)