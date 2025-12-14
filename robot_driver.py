
#!/usr/bin/env python3
"""
文件: robot_driver.py
功能: 极简 ROS 驱动层，负责向 /robot/velcmd 发送速度指令
"""

import rospy
from geometry_msgs.msg import Twist

class RobotDriver:
    def __init__(self, robot_id=8, ros_topic='/robot/velcmd'):
        self.ros_topic = ros_topic
        
        # 防止重复初始化节点
        if not rospy.core.is_initialized():
            rospy.init_node(f'robot_driver_{robot_id}', anonymous=True)
            
        # queue_size=1 保证只执行最新指令，低延迟
        self.pub = rospy.Publisher(self.ros_topic, Twist, queue_size=1)
        print(f"✅ [Driver] 驱动就绪 | 话题: {self.ros_topic}")

    def send_velocity_command(self, vx, vy, omega=0.0):
        """
        发送全向移动指令
        :param vx: 前后速度 (m/s) +前 -后
        :param vy: 左右速度 (m/s) +左 -右
        :param omega: 自转速度 (rad/s) +左转 -右转
        """
        if self.pub is None:
            return

        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(omega)

        self.pub.publish(msg)

    def stop(self):
        """安全停车"""
        self.send_velocity_command(0.0, 0.0, 0.0)