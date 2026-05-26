import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ReadPose
from geometry_msgs.msg import PoseArray
import math

class OdomReader(Node):
    def __init__(self):
        super().__init__('pose_reader_node')
        self.xPos = 0.0
        self.yPos = 0.0
        self.theta = 0.0
        self.poseSubscriber = self.create_subscription(PoseArray, 'pose_receiver', self.poseReadCallback, 10)
        self.readPoseService = self.create_service(ReadPose, 'read_pose_service', self.sendCoords)

    def poseReadCallback(self, msg: PoseArray):
        p = msg.poses[1] #FIXME see if this is maze_robot
        self.xPos = p.position.x
        self.yPos = p.position.y

        # 2D orientation (yaw only)
        qz = p.orientation.z
        qw = p.orientation.w
        theta = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        theta = math.degrees(theta) + 90.0
        
        if theta < 0.0:
            theta += 360.0
        self.theta = theta

    def sendCoords(self, request, response):
        response.x = self.xPos
        response.y = self.yPos
        response.theta = self.theta
        return response

def main():
    rclpy.init()
    odomNode = OdomReader()
    rclpy.spin(odomNode)
    rclpy.shutdown()

if __name__ == '__main__':
    main()