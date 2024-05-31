#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import struct

import receiver
import datetime
import numpy as np

def create_tf_xyz_rpy(x,y,z,roll,pitch,yaw,parent,child):
    tf_msg = TransformStamped()

    # Set the frame IDs
    tf_msg.header.frame_id = parent # Parent frame
    tf_msg.child_frame_id = child # Child frame

    # Set the initial position (x, y, z)
    tf_msg.transform.translation.x = x
    tf_msg.transform.translation.y = y
    tf_msg.transform.translation.z = z

    # Set the initial orientation (roll, pitch, yaw)
   
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    tf_msg.transform.rotation.x = quaternion[0]
    tf_msg.transform.rotation.y = quaternion[1]
    tf_msg.transform.rotation.z = quaternion[2]
    tf_msg.transform.rotation.w = quaternion[3]
    return tf_msg

def create_tf_xyz_quat(x,y,z,qw,qx,qy,qz,parent,child):
    tf_msg = TransformStamped()

    # Set the frame IDs
    tf_msg.header.frame_id = parent # Parent frame
    tf_msg.child_frame_id = child # Child frame

    # Set the initial position (x, y, z)
    tf_msg.transform.translation.x = x
    tf_msg.transform.translation.y = y
    tf_msg.transform.translation.z = z

    # Set the initial orientation (roll, pitch, yaw)
   

    tf_msg.transform.rotation.x = qx
    tf_msg.transform.rotation.y = qy
    tf_msg.transform.rotation.z = qz
    tf_msg.transform.rotation.w = qw
    return tf_msg

class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher')

        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Create a TransformStamped message
        
        # read tf


        self.tf_msg = create_tf_xyz_rpy(1,2,3,0,0,0,'map','camera_link')

        # Publish the TF message at a rate of 10 Hz
        self.tf_publish_rate = rospy.Rate(10)

        self.publish_tf()

    def publish_tf(self):
        import socket
        UDP_IP = "127.0.0.1"  # IP address to listen on
        UDP_PORT = 12345  # Port to listen on
        BUFFER_SIZE = 1024  # Buffer size for incoming messages

        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))

        print("UDP receiver started")
        links=[]

        while not rospy.is_shutdown():
            while True:
                # Receive message
                data, addr = sock.recvfrom(BUFFER_SIZE)
                if not data:
                    break
                
                if len(data)==40:
                    # Unpack the received data into a float array
                    float_array = struct.unpack('10f', data)
                    print(float_array)
                    # Update the timestamp
                    link="link"+str(int(float_array[0]))
                    # tf_msg = create_tf_xyz_quat(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,float_array[4],float_array[5],float_array[6],float_array[7],'map',link)
                    tf_msg = create_tf_xyz_rpy(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,0,0,0,'map',link)
                
                    tf_msg.header.stamp = rospy.Time.now()

                    # Publish the TF message
                    self.tf_broadcaster.sendTransform(tf_msg)

            # print("Received float array:", float_array)

            # Close the socket
            # sock.close()
            # Update the timestamp
            self.tf_msg.header.stamp = rospy.Time.now()
            

            # Publish the TF message
            self.tf_broadcaster.sendTransform(self.tf_msg)

            # Sleep to maintain the publishing rate
            self.tf_publish_rate.sleep()

class Kinect2RobotInputProcessor:
    def __init__(self):
        self.start_time = datetime.datetime.now()
        self.end_time = datetime.datetime.now()
        rospy.init_node('tf_publisher')

        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # rospy.sleep(0.01)

        # Create a TransformStamped message
        
        # read tf


        self.tf_msg = create_tf_xyz_rpy(1,2,3,0,0,0,'map','camera_link')

    def calc_robot_input(self,ts, body):
        # print(ts)
        # Rs=[]
        body[:,1:4]=body[:,1:4]-body[0,1:4]
        p=body[0][1:]
        sn=body[1][1:]
        sc=body[2][1:]
        cla=body[4][1:]
        s = body[5][1:]
        e = body[6][1:]
        w = body[7][1:]

        cla_r=body[11][1:]
        s_r = body[12][1:]
        e_r = body[13][1:]
        w_r = body[14][1:]

        # tf_msg = create_tf_xyz_quat(p[0]/1000,p[1]/1000,p[2]/1000,*p[3:],'map',"p")
        tf_msg = create_tf_xyz_quat(p[0]/1000,p[1]/1000,p[2]/1000,1,0,0,0,'map',"p")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(sn[0]/1000,sn[1]/1000,sn[2]/1000,*sn[3:],'map',"sn")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(sc[0]/1000,sc[1]/1000,sc[2]/1000,*sc[3:],'map',"sc")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)

        tf_msg = create_tf_xyz_quat(cla[0]/1000,cla[1]/1000,cla[2]/1000,*cla[3:],'map',"cla")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(s[0]/1000,s[1]/1000,s[2]/1000,*s[3:],'map',"s")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(e[0]/1000,e[1]/1000,e[2]/1000,*e[3:],'map',"e")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(w[0]/1000,w[1]/1000,w[2]/1000,*w[3:],'map',"w")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)

        tf_msg = create_tf_xyz_quat(cla_r[0]/1000,cla_r[1]/1000,cla_r[2]/1000,*cla_r[3:],'map',"cla_r")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(s_r[0]/1000,s_r[1]/1000,s_r[2]/1000,*s_r[3:],'map',"s_r")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(e_r[0]/1000,e_r[1]/1000,e_r[2]/1000,*e_r[3:],'map',"e_r")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)
        tf_msg = create_tf_xyz_quat(w_r[0]/1000,w_r[1]/1000,w_r[2]/1000,*w_r[3:],'map',"w_r")
        tf_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(tf_msg)



        rospy.sleep(0.00)

    def process(self, data):
        # print(data['n'])
        if data['n'] > 0:
            body_data_with_index = data['body'][0]
            body_data = body_data_with_index[1:]
            body = np.array([body_data])
            body = body.reshape(32, 8)
            s = body[5]
            e = body[6]
            w = body[7]
            if s[0] > 1 and e[0] > 1 and w[0] > 1:
                end_time = datetime.datetime.now()
                time_interval = end_time - self.start_time

                # T_s=xyz_quaternion_to_homogeneous(*s[1:])
                # T_e = xyz_quaternion_to_homogeneous(*e[1:])
                # T_w = xyz_quaternion_to_homogeneous(*w[1:])

                self.calc_robot_input(time_interval,body)
                self.start_time=end_time

            # print(s, e, w)

        return 0

if __name__ == '__main__':
    try:
        # TFPublisher()
        my_processor=Kinect2RobotInputProcessor()
        rec = receiver.UdpReceiver(process_function=my_processor.process)
        
    except rospy.ROSInterruptException:
        # print()
        pass
