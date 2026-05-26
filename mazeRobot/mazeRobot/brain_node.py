#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MoveMotor, ReadSensors
from mazeRobot.DQLAgent import DQLAgent
from mazeRobot.Simulation import Simulation
import time
from custom_interfaces.srv import ReadPose
from ros_gz_interfaces.srv import SpawnEntity

class Brain(Node):
    def __init__(self):
        super().__init__('brain_node')
        self.motorClient = self.create_client(MoveMotor, 'motor_move_service')
        self.sensorClient = self.create_client(ReadSensors, 'sensor_read_service')
        self.spawnerClient = self.create_client(SpawnEntity, "/world/empty/create")
        self.poseClient = self.create_client(ReadPose, 'read_pose_service')
        self.episode = 0
        self.STATE_SIZE = 36
        self.ACTION_SIZE = 4
        self.mazeAgent = DQLAgent(self, self.STATE_SIZE, self.ACTION_SIZE)
        self.sim = Simulation(self.mazeAgent, self)
        self.runTrainingSession()
        

    def runTrainingSession(self):
        for episode in range(200):
            state = self.sim.reset()
            done = False
            totalReward = 0.0

            i = 1
            while not done:
                req = MoveMotor.Request()
                action = self.mazeAgent.getNextAction(state)
                req.action = str(action)
                nextState, reward, done = self.sim.performAction(req)

                if self.mazeAgent.epsilon <= 0.25:
                    self.sim.purposefulActions = True
                else:
                    self.sim.purposefulActions = False

                if i >= 3:
                    self.mazeAgent.storeEvent(state, action, reward, nextState, done)
                self.mazeAgent.updateOnlineNetwork()
                
                state = nextState
                totalReward += reward
                time.sleep(0.01)
                i += 1
                
            self.get_logger().info(f"purpose: {self.sim.purposefulActions}")
            if i > 1000 or (episode % 20 == 0):
                self.mazeAgent.updateTargetNetwork()
            
            self.get_logger().info(f'Total reward for episode {episode}: {totalReward}')
            self.episode += 1

            if episode == 20:
                self.mazeAgent.saveModel("maze_model_20.keras")
            elif episode == 50:
                self.mazeAgent.saveModel("maze_model_50.keras")
            elif episode == 70:
                self.mazeAgent.saveModel("maze_model_70.keras")
            elif episode == 90:
                self.mazeAgent.saveModel("maze_model_90.keras")


def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    node.get_logger().info('Training has been completed. Congrats!')
    node.mazeAgent.saveModel("maze_model_final.keras")
    node.get_logger().info('Model Saved. Congrats!')
    rclpy.shutdown()



if __name__ == '__main__':
    main()