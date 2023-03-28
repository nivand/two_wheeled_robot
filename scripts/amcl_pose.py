#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses
from nav2_msgs.srv import LoadMap, ClearEntireCostmap, ManageLifecycleNodes, GetCostmap

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from math import *
from numpy import *
from numpy.linalg import *
from matplotlib.pyplot import *
from std_msgs.msg import Float32

class BasicNavigator(Node):
    def __init__(self):
        self.t1=time.time()
        self.t2=time.time()
        super().__init__(node_name='amcl_pose')
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.cov_x=[]
        self.i=0.0
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'robot1/amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        cov=msg.pose.covariance
        self.cov_x.append(cov[0,0])
    
        
        return  
    def plottest(self):
        
        
        dt1=self.t2-self.t1
        figure()
        title('covariance')
        plot(linspace(0.0, dt1, num=len(self.cov_x)),self.cov_x,'b')
        ylabel('Covariace X($m^2$)')
        xlabel('Time (s))')
        grid()  

    def pathplot(self):
        if self.i==150:
            self.t2=time.time()
            self.plottest()
            show()
        else:
            self.i+=1
            print('i',self.i)  
def main(args=None):
    rclpy.init(args=args)
    m=BasicNavigator()
    rclpy.spin(m)
    m.pathplot()
    show()
    m.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()        
