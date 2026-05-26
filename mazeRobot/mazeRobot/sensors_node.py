#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from custom_interfaces.srv import ReadSensors
from sensor_msgs.msg import LaserScan
# from rclpy.qos import qos_profile_sensor_data

class Sensors(Node):
    def __init__(self):
        super().__init__('sensors_node')
        self.get_logger().info('Sensors Node created')

        self.frontDist = 0.0
        self.rightDist = 0.0
        self.leftDist = 0.0
        self.backDist = 0.0
        self.frontleftDist = 0.0
        self.backleftDist = 0.0
        self.frontrightDist = 0.0
        self.backrightDist = 0.0

        self.serviceProvider = self.create_service(ReadSensors, 'sensor_read_service', self.sendSensorData)
        self.frontSub = self.create_subscription(LaserScan, 'scan_front', self.front_callback, 10)
        self.leftSub = self.create_subscription(LaserScan, 'scan_left', self.left_callback, 10)
        self.rightSub = self.create_subscription(LaserScan, 'scan_right', self.right_callback, 10)
        self.backSub = self.create_subscription(LaserScan, 'scan_back', self.back_callback, 10)
        self.frontleftSub = self.create_subscription(LaserScan, 'scan_frontleft', self.frontleft_callback, 10)
        self.frontrightSub = self.create_subscription(LaserScan, 'scan_frontright', self.frontright_callback, 10)
        self.backleftSub = self.create_subscription(LaserScan, 'scan_backleft', self.backleft_callback, 10)
        self.backrightSub = self.create_subscription(LaserScan, 'scan_backright', self.backright_callback, 10)

    def extract_center_distance(self, msg: LaserScan) -> float:
        if not msg.ranges:
            return 50.0

        center_index = len(msg.ranges) // 2
        r = float(msg.ranges[center_index])

        if r == float('inf'):
            return 50.0

        if r == float('-inf'):
            return 0.0

        if math.isnan(r):
            return 0.0

        return r
    

    def front_callback(self, msg: LaserScan):
        self.frontDist = self.extract_center_distance(msg)
           
    def right_callback(self, msg: LaserScan):
        self.rightDist = self.extract_center_distance(msg)
        
    def left_callback(self, msg: LaserScan):
        self.leftDist = self.extract_center_distance(msg)

    def back_callback(self, msg:LaserScan):
        self.backDist = self.extract_center_distance(msg)
        
    def frontleft_callback(self, msg: LaserScan):
        self.frontleftDist = self.extract_center_distance(msg)

    def frontright_callback(self, msg: LaserScan):
        self.frontrightDist = self.extract_center_distance(msg)

    def backleft_callback(self, msg: LaserScan):
        self.backleftDist = self.extract_center_distance(msg)

    def backright_callback(self, msg: LaserScan):
        self.backrightDist = self.extract_center_distance(msg)


    def sendSensorData(self, request, response):
        response.front = float(self.frontDist)
        response.right = float(self.rightDist)
        response.left = float(self.leftDist)
        response.back = float(self.backDist)
        response.frontleft = float(self.frontleftDist)
        response.frontright = float(self.frontrightDist)
        response.backleft = float(self.backleftDist)
        response.backright = float(self.backrightDist)
        return response

def main():
    rclpy.init()
    rclpy.spin(Sensors())
    rclpy.shutdown()

if __name__ == '__main__':
    main()