import grpc
from avp_stream.grpc_msg import * 
from threading import Thread
from avp_stream.utils.grpc_utils import * 
import time 
import numpy as np 
import json
import gc


YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype = np.float64)


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
        self.start_streaming()

    def start_streaming(self): 

        stream_thread = Thread(target = self.stream)
        stream_thread.start() 
        while self.latest is None: 
            pass 
        print(' == DATA IS FLOWING IN! ==')
        print('Ready to start streaming.') 


    def stream(self): 

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

        except Exception as e:
            print(f"An error occurred: {e}")
            pass 

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
        print(json_string)
        print('---------------------------succeed in convertion-----------------------')
        with open(self.root+'/'+self.json_name, "w") as outfile:
            outfile.write(json_string)
            print('---------------------------succeed in saving files!-----------------------')

    



    

if __name__ == "__main__": 
    
    time_step = 1

    js_name = f"file_{time_step}.json"

    # streamer = VisionProStreamer(ip = '192.168.113.40',root='visionpro_data/output',json_name=js_name)##guozi:10.29.230.57
    while True: 
        js_name = f"file_{time_step}.json"


        streamer = VisionProStreamer(ip = '192.168.113.40',root='visionpro_data/output',json_name=js_name)##guozi:10.29.230.57


        latest = streamer.get_latest()
        print(latest)

        streamer.conv_json()

        if time_step==10:
            break
        
        del streamer
        time_step+=1

    
    