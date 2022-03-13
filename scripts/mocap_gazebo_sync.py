#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from gazebo_msgs.msg import ModelState
import numpy as np

n_drones = 2
model_name = 'iris'
z_threshold = 0.05 # this is the z zero-error to solve the height issue

class Drone():
    def __init__(self,id,node):
        self.id = id
        self.state = ModelState()
        self.state.model_name = model_name + str(id)
        self.pub = node.create_publisher( ModelState,'/gazebo/set_model_state', 1)
        self.sub = node.create_subscription(PoseStamped,"/qualisys_node/cf" + str(id) + "/pose", self.callback,1)        

    def callback(self,data):
        self.state.pose.position.x = data.pose.position.x * 40
        self.state.pose.position.y = data.pose.position.y * 40

        if(data.pose.position.z < z_threshold):
            self.state.pose.position.z = 0.0
        else:
            self.state.pose.position.z = data.pose.position.z * 40

        self.state.pose.orientation = data.pose.orientation   
        self.pub.publish(msg)

class MocapGazeboSync(Node):

    def __init__(self):
        super().__init__('mocap_gazebo_sync')
        objs = list()
        for i in range(n_drones):
            ## self.pub = self.create_publisher('/gazebo/set_model_state', ModelState, 1)
            objs.append(Drone(i,self))
        
        
def main(args=None):
    rclpy.init(args=args)

    mocap_gazebo_sync = MocapGazeboSync()

    rclpy.spin(mocap_gazebo_sync)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mocap_gazebo_sync.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    
    
   