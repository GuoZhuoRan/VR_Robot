import grpc
from avp_stream.grpc_msg import * 
from threading import Thread
from avp_stream.utils.grpc_utils import *
from avp_stream.grpc_msg import *
import time 
import numpy as np 
import json
import gc
from datetime import datetime

# Get the current date and time



YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype = np.float64)

running=True

vision_pro_ip='192.168.113.27'
# vision_pro_port=

udp_ip = "127.0.0.1"  # Target IP address
udp_port = 5015  # Target port number

class VisionProStreamer:

    def __init__(self, ip,root,json_name,record = True): 
        #guozi:
        self.root = root
        self.json_name = json_name
        # Vision Pro IP 
        self.ip = ip
        self.record = record 
        self.recording = [] 
        self.latest = None 
        self.axis_transform = YUP2ZUP
        # self.start_streaming()

    def start_streaming(self): 

        stream_thread = Thread(target = self.stream)
        stream_thread.start() 
        while self.latest is None: 
            pass 
        print(' == DATA IS FLOWING IN! ==')
        print('Ready to start streaming.') 


    def stream(self):
        

        print(f"start udp send VP data to {udp_ip}:{udp_port}")

        while running:


            request = handtracking_pb2.HandUpdate()
            try:
                with grpc.insecure_channel(f"{self.ip}:12345") as channel:
                    stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                    responses = stub.StreamHandUpdates(request)
                    for response in responses:
                        transformations = {
                            "left_wrist": self.axis_transform @  process_matrix(response.left_hand.wristMatrix),
                            "right_wrist": self.axis_transform @  process_matrix(response.right_hand.wristMatrix),
                            "left_fingers":   process_matrices(response.left_hand.skeleton.jointMatrices),
                            "right_fingers":  process_matrices(response.right_hand.skeleton.jointMatrices),
                            "head": rotate_head(self.axis_transform @  process_matrix(response.Head)) ,
                            "left_pinch_distance": get_pinch_distance(response.left_hand.skeleton.jointMatrices),
                            "right_pinch_distance": get_pinch_distance(response.right_hand.skeleton.jointMatrices),
                            # "rgb": response.rgb, # TODO: should figure out how to get the rgb image from vision pro
                        }
                        transformations["right_wrist_roll"] = get_wrist_roll(transformations["right_wrist"])
                        transformations["left_wrist_roll"] = get_wrist_roll(transformations["left_wrist"])
                        
                        if self.record:
                            self.recording.append(transformations)
                        self.latest = transformations

                        latest = self.get_latest()

                        head_data = np.array(latest['head'])
                        head_data = head_data.reshape((-1))
                        right_data = np.array(latest['right_wrist'])
                        right_data = right_data.reshape((-1))
                        left_fin_data = np.array(latest['left_fingers'])
                        left_fin_data=left_fin_data.reshape((-1))
                        right_fin_data = np.array(latest['right_fingers'])
                        right_fin_data = right_fin_data.reshape((-1))

                        left_data = np.array(latest['left_wrist'])
                        left_data = left_data.reshape((-1))
                        float_array = []
                        for i in range(16):
                            float_array.append(head_data[i])
                        for i in range(16):
                            float_array.append(left_data[i])
                        for i in range(16):
                            float_array.append(right_data[i])

                        for i in range(400):
                            float_array.append(left_fin_data[i])
                        for i in range(400):
                            float_array.append(right_fin_data[i])

                        # print(len(float_array))

                        send_float_array(udp_ip, udp_port, float_array)




            except Exception as e:
                print(f"An error occurred: {e}")
                pass
        
        print("VP end")

    def get_latest(self): 
        return self.latest
        
    def get_recording(self): 
        return self.recording
    

    def conv_json(self):
        import json
        import numpy as np


        self.latest['left_wrist'] = self.latest['left_wrist'].tolist()
        self.latest['right_wrist'] =self.latest['right_wrist'].tolist() 
        self.latest['left_fingers'] = self.latest['left_fingers'].tolist()
        self.latest['right_fingers'] = self.latest['right_fingers'].tolist()
        self.latest['head'] = self.latest['head'].tolist()

        tmp = self.latest

        json_string = json.dumps(tmp)
        # print(json_string)
        # print('---------------------------succeed in convertion-----------------------')
        with open(self.root+'/'+self.json_name, "w") as outfile:
            outfile.write(json_string)
            print('---------------------------succeed in saving files!-----------------------')


import socket
import struct
import numpy as np


# Function to send a float array over UDP
def send_float_array(udp_ip, udp_port, float_array):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Convert the float array to binary format using struct
    # 'f' is the format character for a float (4 bytes)
    # Add a count before 'f' to specify the number of floats in the array
    data = struct.pack(f'{len(float_array)}f', *float_array)

    # Send the data over UDP
    sock.sendto(data, (udp_ip, udp_port))

def start_vision_pro_server():
    streamer = VisionProStreamer(ip = vision_pro_ip,root='output',json_name="123")##guozi:10.29.230.57
    streamer.stream()
    


if __name__ == "__main__": 
    
    time_step = 1

    js_name = f"file_{time_step}.json"




    streamer = VisionProStreamer(ip = '192.168.112.109',root='output',json_name=js_name)##guozi:10.29.230.57
    while True: 
        js_name = f"file_{time_step}.json"
        time.sleep(1)



        # print(latest)
        print('-----start recording positions-------------')
        streamer.conv_json()

        # if time_step==10:
        #     break
        
        # del streamer
        # time_step+=1
        now = datetime.now()
        print(now)

    
    