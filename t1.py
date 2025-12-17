#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped

# 请确认这里和你的 rostopic list 完全一致（加上前面的 / 以防万一）
TOPICS = {
    "ROBOT":  "/vicon/VSWARM15/VSWARM15",
    "TARGET": "/vicon/VSWARM45/VSWARM45",
    "LEADER": "/vicon/VSWARM13/VSWARM13"
}

def callback(msg, name):
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    print(f"[{name}] X: {x:.3f} | Y: {y:.3f}")

def listener():
    rospy.init_node('vicon_debugger', anonymous=True)
    
    print("----- 开始监听 Vicon 原始坐标 -----")
    for name, topic in TOPICS.items():
        print(f"正在订阅: {topic}")
        rospy.Subscriber(topic, TransformStamped, callback, name)

    rospy.spin()

if __name__ == '__main__':
    listener()