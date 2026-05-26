import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.srv import MoveMotor  

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.get_logger().info('Motor Controller Node created')
        self.serviceProvider = self.create_service(MoveMotor, 'motor_move_service', self.sendMotorCommand)
        self.velCmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pending_response = None
        self.RATE_SCALE = 12       # larger nums = longer time (18 seems to not be buggy)
        self.linearSpeed = 350.0 / self.RATE_SCALE
        self.angularSpeed = 261.7993878 / self.RATE_SCALE


    def sendMotorCommand(self, request, response):
        action = request.action
        velMsg = Twist()
        stopMsg = Twist()
        stopMsg.linear.x = 0.0
        stopMsg.linear.y = 0.0
        stopMsg.linear.z = 0.0
        stopMsg.angular.x = 0.0
        stopMsg.angular.y = 0.0
        stopMsg.angular.z = 0.0

        match action:
            case "0":  # backwards
                velMsg.linear.y = -self.linearSpeed
                velMsg.angular.z = 0.0
            case "1":  # forwards
                velMsg.linear.y = self.linearSpeed
                velMsg.angular.z = 0.0
            case "2":  # left
                velMsg.linear.y = 0.0
                velMsg.angular.z = self.angularSpeed
            case "3":  # right
                velMsg.linear.y = 0.0
                velMsg.angular.z = -self.angularSpeed

        response.action = "act"
        self.velCmd.publish(velMsg)
        time.sleep(0.001 * self.RATE_SCALE)
        self.velCmd.publish(stopMsg)
        time.sleep(0.01)
        return response


def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

