import socket
import struct


class UdpReceiver:
    def __init__(self, process_function):
        UDP_IP = "127.0.0.1"  # IP address to listen on
        UDP_PORT = 12345  # Port to listen on
        BUFFER_SIZE = 1024 * 2  # Buffer size for incoming messages

        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))

        print("UDP receiver started")
        data={}
        while True:
            # Receive message
            data_bytes, addr = sock.recvfrom(BUFFER_SIZE)
            if not data_bytes:
                break


            l = len(data_bytes)
            # print(l)
            if l == 4*4:
                float_array = struct.unpack(f'{4}f', data_bytes)
                data={'n':int(float_array[0]),'body':[]}
                data_len_f=int(float_array[1])
                if data['n']==0:
                    process_function(data)

            if l == 4 * (1 + 32 * 8):
                data_len_f = 1 + 32 * 8

                # Unpack the received data into a float array
                float_array = struct.unpack(f'{data_len_f}f', data_bytes)

                data['body'].append(float_array)
                # print("Received float array:", float_array[:10])
                if float_array[0]==data['n']-1:
                    process_function(data)
            if l == 8:
                ss = struct.unpack('!8c', data_bytes)
                print("Received ss:", ss)

        # Close the socket
        sock.close()
