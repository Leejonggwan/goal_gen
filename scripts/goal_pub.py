#! /usr/bin/env python

from ast import Str
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point

class PatrolLIMO():
    def __init__(self):
        rospy.init_node("patrol_limo")
        
        self.waypoints = rospy.get_param("/waypoint")
        
        self.limo_pose = Odometry()
        self.tmp = String()
        
        self.old_moving_x = 0
        self.old_moving_y = 0
        self.old_moving_z = 0
        self.old_stop_x = 0
        self.old_stop_y = 0
        self.old_stop_z = 0
        
        self.count_number = 1000000
        
        self.now_wp = ""
        
        self.waypoint_sort()
        
        self.odom_sub = rospy.Subscriber("/odom",Odometry, self.odom_CB)
        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        
        
    def patrol(self):
        
        while not rospy.is_shutdown():
            
            self.find_now_wp()
        
            self.next_wp()
            self.pub_goal(self.final_wp)
            
            while not self.is_goal():
                pass

    def pub_goal(self,wp_final):
        
        tmp_pose = PoseStamped()
        tmp_pose.header.stamp = rospy.Time.now()
        tmp_pose.header.frame_id = "map"
        
        tmp_pose.pose.position.x = self.waypoints[wp_final][0]
        tmp_pose.pose.position.y = self.waypoints[wp_final][1]
        tmp_pose.pose.position.z = self.waypoints[wp_final][2]
        tmp_pose.pose.orientation.x = self.waypoints[wp_final][3]
        tmp_pose.pose.orientation.y = self.waypoints[wp_final][4]
        tmp_pose.pose.orientation.z = self.waypoints[wp_final][5]
        tmp_pose.pose.orientation.w = self.waypoints[wp_final][6]
        
        rospy.loginfo("Request to Go Position {} to {}".format(self.now_wp,self.final_wp))
        self.goal_pub.publish(tmp_pose)
        
    def waypoint_sort(self):
        
        # print(len(self.waypoints))
        self.wp_key = []
        
        for i in range(1,len(self.waypoints)+1):
            
            if i < 10:
                tmp_key_value = 'WP0' + str(i)
            elif i >= 10 and i < 100:
                tmp_key_value = 'WP' + str(i)
            
            self.wp_key.append(tmp_key_value)
            # print(self.wp_key)
    
    def odom_CB(self, data):
        self.robot_pose = data
        
    def next_wp(self):
        
        self.next_wp_num = 0
        
        for i, n in enumerate(self.wp_key):
            if n == self.now_wp:
                self.next_wp_num = i + 1
        
        if self.next_wp_num >= len(self.wp_key):
            self.next_wp_num = 1
        
        self.final_wp = self.wp_key[self.next_wp_num]
        
        
    def find_now_wp(self):
        
        self.init_dist = 10000
        
        for i in self.waypoints.keys():
            distance = self.calc_distance(i)
            
            if self.init_dist > distance:
                self.init_dist = distance
                self.now_wp = i
        
    def is_goal(self):
        
        tmp_x = self.robot_pose.pose.pose.position.x
        tmp_y = self.robot_pose.pose.pose.position.y
        if self.final_wp == "":
            distance = 5
        else:
            distance = self.calc_distance(self.final_wp)
        
        if self.is_stop() and distance <= self.DISTANCE_TH:
            return True
        else:
            return False
    
    
    def is_stop(self):
        
        tmp_x = self.robot_pose.pose.pose.position.x
        tmp_y = self.robot_pose.pose.pose.position.y
        tmp_z = self.robot_pose.pose.pose.orientation.z
        
        if tmp_x - self.old_stop_x == 0 and tmp_y - self.old_stop_y == 0 and tmp_z - self.old_stop_z == 0:
            self.old_stop_x = tmp_x
            self.old_stop_y = tmp_y
            self.old_stop_z = tmp_z
            return True
        else:
            self.old_stop_x = tmp_x
            self.old_stop_y = tmp_y
            self.old_stop_z = tmp_z
            return False
        
        
    
    def calc_distance(self, pose_A):
        x_diff = self.robot_pose.pose.pose.position.x - self.waypoints[pose_A][0]
        y_diff = self.robot_pose.pose.pose.position.y - self.waypoints[pose_A][1]
        return (x_diff ** 2 + y_diff ** 2) ** 0.5