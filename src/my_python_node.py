# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys




def main():
    rospy.init_node('my_python_node')  # 初始化ROS节点
    pub = rospy.Publisher('my_topic', String, queue_size=10)  # 创建话题发布者
    rate = rospy.Rate(1)  # 设置发布频率为1Hz

    while not rospy.is_shutdown():
        msg = "Hello, ROS!"  # 准备要发布的消息
        rospy.loginfo(msg)  # 打印消息到终端

        rospy.loginfo(sys.version)

        pub.publish(msg)  # 发布消息到话题
        rate.sleep()  # 休眠以保持发布频率
        # break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
