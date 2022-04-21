#! /usr/bin/env python

import rospy
import yaml
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point, PoseWithCovarianceStamped

class CreateYaml():
    def __init__(self):
        rospy.init_node("create_wp_yaml")
        
        self.changeWorkingDir()
        
        self.waypoint = {}
        self.WP = {}
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.ori_x = 0.0
        self.ori_y = 0.0
        self.ori_z = 0.0
        self.ori_w = 0.0
                
        self.yaml_name_str = "test"
        self.battery_charge_percentage = 10
        
        self.str_sub = rospy.Subscriber("create_wp", String, self.str_CB)
        self.odom_sub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped, self.odom_CB)
        
        rospy.spin()
    
    def changeWorkingDir(self):
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        
    def createYaml(self,str):
        
        tmp_str = '../params/' + str + '.yaml'
        
        with open(tmp_str, 'w') as f:
            yaml.dump(self.waypoint, f)
            
    def str_CB(self,data):
        
        if data.data == "end":
            
            self.waypoint["waypoint"] = self.WP
            
            self.createYaml(self.yaml_name_str)
            print("Now {}.yaml file is saved !!!!!!!!!!!".format(self.yaml_name_str))
            
            
        else:
            
            if len(self.WP) == 0:
                
                tmp_str = "WP01"
                
                self.odom_parsing()
                
                self.WP[tmp_str] = [self.pose_x, self.pose_y, self.pose_z, self.ori_x, self.ori_y, self.ori_z, self.ori_w]
                print("Now Location is saved and name : {} ".format(tmp_str))
                
            else:
                
                tmp_num = self.wp_sorting()
                
                if tmp_num < 9:
                    tmp_str = "WP0" + str(tmp_num+1)
                elif tmp_num < 99 and tmp_num >= 9:
                    tmp_str = "WP" + str(tmp_num+1)
                
                
                
                self.odom_parsing()
                
                self.WP[tmp_str] = [self.pose_x, self.pose_y, self.pose_z, self.ori_x, self.ori_y, self.ori_z, self.ori_w]
                print("Now Location is saved and name : {} ".format(tmp_str))
            
            
    def odom_CB(self,data):
        
        self.robot_pose = data
        
    def wp_sorting(self):
        
        tmp_wp_num = len(self.WP)
        
        return tmp_wp_num
    
    def odom_parsing(self):
        
        self.pose_x = self.robot_pose.pose.pose.position.x
        self.pose_y = self.robot_pose.pose.pose.position.y
        self.pose_z = self.robot_pose.pose.pose.position.z
        self.ori_x = self.robot_pose.pose.pose.orientation.x
        self.ori_y = self.robot_pose.pose.pose.orientation.y
        self.ori_z = self.robot_pose.pose.pose.orientation.z
        self.ori_w = self.robot_pose.pose.pose.orientation.w
        

if __name__ == "__main__":
    CreateYaml()