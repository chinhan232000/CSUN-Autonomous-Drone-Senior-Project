#!/usr/bin/env python3

import rospy
import mavros
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np
from numpy import *
from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, NavSatFix
import message_filters
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time

# Define a global variable for the next state
global NEXT_STATE
NEXT_STATE = 'TAKEOFF'

# Set printing options for NumPy arrays
np.set_printoptions(threshold = np.inf)

# Function to arm or disarm the UAV
def arming_call(arm = False, time_delay = 5):
    rospy.loginfo("\n----------armingCall----------")
    rospy.wait_for_service("/mavros/cmd/arming")
    uav_arm = rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
    uav_arm(arm)
    rospy.sleep(time_delay)

# Function to switch flight modes
def switch_modes(next_mode, time_delay = 5):
    rospy.wait_for_service("/mavros/set_mode")
    modes = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    modes(custom_mode = next_mode)
    rospy.loginfo("\n----------mode:[%s]----------" % next_mode)
    rospy.sleep(time_delay)

# Function to initiate takeoff
def takeoff_call(lat, long, altitude, time_delay = 10):
    rospy.loginfo("\n----------takeoffCall----------")
    rospy.wait_for_service("/mavros/cmd/takeoff")
    takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
    takeoff(0, 0, lat, long, altitude)
    rospy.sleep(time_delay)

# Function to initiate landing
def land_call(lat, long, altitude, time_delay = 30):
    rospy.loginfo("\n----------landCall----------")
    rospy.wait_for_service("/mavros/cmd/land")
    land = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
    land(0, 0, lat, long, altitude)
    rospy.sleep(time_delay)

# Function to obtain objects detected by the lidar
def get_objects(localposition, lidar):
    lidar_ranges = np.asarray(lidar.ranges)
    increment_angle = 2*np.pi/len(lidar_ranges)
    drone_quaternions = [localposition.pose.orientation.x, 
                         localposition.pose.orientation.y, 
                         localposition.pose.orientation.z, 
                         localposition.pose.orientation.w]
    drone_angle = tf.transformations.euler_from_quaternion(drone_quaternions)[2]
    frame_angle = drone_angle - np.pi/2
    lidar_angle = np.arange(0, 2*np.pi, increment_angle, dtype = float)
    absolute_angle = lidar_angle + frame_angle
    origin_to_drone_distance = np.sqrt(localposition.pose.position.x**2 + localposition.pose.position.y**2)
    origin_to_drone_angle = np.arctan2(localposition.pose.position.y, localposition.pose.position.x)
    objects_x = np.multiply(lidar_ranges, np.cos(absolute_angle)) + origin_to_drone_distance*np.cos(origin_to_drone_angle)
    objects_y = np.multiply(lidar_ranges, np.sin(absolute_angle)) + origin_to_drone_distance*np.sin(origin_to_drone_angle)
    return objects_x, objects_y

# Function to check if the UAV has arrived at a target location
def get_arrived_status(target, localposition, tolerance):
    target_x, target_y = target[0], target[1]
    delta_x, delta_y = target_x - localposition.pose.position.x, target_y - localposition.pose.position.y
    target_distance = np.sqrt(delta_x**2 + delta_y**2)
    if target_distance < tolerance:
        ARRIVED_STATUS = True
    else:
        ARRIVED_STATUS = False
    return ARRIVED_STATUS

# Class for UAV navigation
class navigation:
    def __init__(self, points_x, points_y, points_yaw, diameter):
        # Initialize arrays to store block, travel, and path data
        self.block_x, self.block_y = np.array([]), np.array([])
        self.travel_x, self.travel_y = np.array([]), np.array([])
        
        # Initialize navigation parameters
        self.points_x, self.points_y = points_x, points_y
        self.points_yaw = points_yaw
        self.point_index = 0
        self.current_point_x, self.current_point_y = self.points_x[0], self.points_y[0]
        self.current_point_yaw = self.points_yaw[0]
        self.completed_points = False

        self.diameter = diameter
        self.jammed = False

        self.time_1 = time.time()
        self.time_2 = time.time()
        self.TIMEOUT = 60

    # Function to reject circles in the lidar data
    def reject_circles(self, circle_x, circle_y, circle_status):
        for status_index, (x, y, status) in enumerate(zip(circle_x, circle_y, circle_status)):
            if status == True:
                continue
            for avoid_index, (avoid_x, avoid_y) in enumerate(zip(self.avoid_x, self.avoid_y)):
                dx, dy = avoid_x - x, avoid_y - y
                distance = np.sqrt(dx**2 + dy**2)
                if distance <= self.diameter/2:
                    circle_status[status_index] = True
        return circle_status
    
    def reject_paths_from_gaps(self):
        for gap_index, (gap_x, gap_y,  gap_status) in enumerate(zip(self.gap_x, self.gap_y, self.gap_status)):
            if gap_status == False:
                continue
            for path_index, (path_x, path_y, path_status) in enumerate(zip(self.path_x, self.path_y, self.path_status)):
                if path_status == True:
                    continue
                dx, dy = path_x - gap_x, path_y - gap_y
                distance = np.sqrt(dx**2 + dy**2)
                if distance < 0.95*self.diameter:
                    self.path_status[path_index] = True
                    
    def select_best_path(self):
        dx, dy = self.current_point_x - self.path_x, self.current_point_y - self.path_y
        distances = np.sqrt(dx**2 + dy**2)
        distances_index = np.arange(0, len(distances))
        
        if np.sum(self.path_status) == self.path_length:
            rospy.logwarn("path_status: %s" % self.path_status)
            rospy.logwarn("Drone is Jammed")
            self.jammed = True
            return

        filtered_distances, filtered_distances_index = distances[~self.path_status], distances_index[~self.path_status]
        select_best_index = filtered_distances_index[np.argmin(filtered_distances)]
        self.target_x, self.target_y = self.path_x[select_best_index], self.path_y[select_best_index]
    
    def update_travel(self):
        self.travel_x = np.append(self.travel_x, self.east)
        self.travel_y = np.append(self.travel_y, self.north)
         
    def update_block(self):
        if len(self.travel_x) < 2:
            return
        
        dx, dy = self.current_point_x - self.travel_x[-2], self.current_point_y - self.travel_y[-2]
        previous_distance = np.sqrt(dx**2 + dy**2)
        dx, dy = self.current_point_x - self.travel_x[-1], self.current_point_y - self.travel_y[-1]
        current_distance = np.sqrt(dx**2 + dy**2)

        if current_distance >= previous_distance:
            rospy.logwarn("New Block Entry Added")
            self.block_x = np.append(self.block_x, self.travel_x[-2])
            self.block_y = np.append(self.block_y, self.travel_y[-2])
            
    def update_avoid(self):
        self.avoid_x = np.hstack([self.obj_x, self.block_x])
        self.avoid_y = np.hstack([self.obj_y, self.block_y])
        
    def get_target(self):
        self.path_status = np.zeros(self.path_length, dtype = bool)
        self.gap_status = np.zeros(self.gap_length, dtype = bool)

        self.gap_status = self.reject_circles(self.gap_x, self.gap_y, self.gap_status)
        self.reject_paths_from_gaps()
        self.path_status = self.reject_circles(self.path_x, self.path_y, self.path_status)
        self.select_best_path()

    def jammed_response(self):
        if self.jammed == True:
            self.jammed = False
            rospy.logwarn("Cleared All Block Entries and Retrying")
            self.block_x, self.block_y = np.array([]), np.array([])
            self.update_avoid()
            self.get_target()
            if self.jammed == True:
                rospy.logwarn("Drone is Completely Trapped")
                self.target_x, self.target_y = self.east, self.north

    def initalize_navigation(self, localposition, lidar):
        objects_x, objects_y = get_objects(localposition, lidar)
        UAV.update_target(localposition.pose.position.x, localposition.pose.position.y, objects_x, objects_y)
        
    def update_current_point(self):
        self.point_index += 1

        if self.point_index > len(self.points_x) - 1:
            self.completed_points = True
            return

        self.current_point_x = self.points_x[self.point_index]
        self.current_point_y = self.points_y[self.point_index]
        self.current_point_yaw = self.points_yaw[self.point_index]

    def initialize_reverse_travel(self):
        self.update_travel()
        self.reverse_target_index = len(self.travel_x) - 1
        self.reverse_target_x, self.reverse_target_y = self.travel_x[self.reverse_target_index], self.travel_y[self.reverse_target_index]

    def update_reverse_target(self):
        self.reverse_target_index -= 1
        self.reverse_target_x, self.reverse_target_y = self.travel_x[self.reverse_target_index], self.travel_y[self.reverse_target_index]

    def update_target(self, east, north, objects_x, objects_y):
        self.east, self.north = east, north
        self.obj_x, self.obj_y = objects_x, objects_y

        E, N, D = self.east, self.north, self.diameter
        self.path_x = np.array([D+E, D+E, E, -D+E, -D+E, -D+E, E, D+E], dtype = float)
        self.path_y = np.array([N, D+N, D+N, D+N, N, -D+N, -D+N, -D+N], dtype = float)
        self.path_length = len(self.path_x)
        
        self.gap_x = np.array([(D/2)+E, (-D/2)+E, (-D/2)+E, (D/2)+E], dtype = float)
        self.gap_y = np.array([(D/2)+N, (D/2)+N, (-D/2)+N, (-D/2)+N], dtype = float)
        self.gap_length = len(self.gap_x)
        
        self.update_travel()
        self.update_block()
        self.update_avoid()
        self.get_target()
        self.jammed_response()
    
global UAV
UAV = navigation(points_x = [0,50], points_y = [50,50], points_yaw = [180,90], diameter = 6)

def get_target_yaw(target, localposition):
    target_x, target_y = target[0], target[1] 
    target_yaw = np.arctan2(target_y - localposition.pose.position.y, target_x - localposition.pose.position.x)
    target_yaw = target_yaw*(180/np.pi)
    return target_yaw

def goto(target):
    target_x, target_y, target_z, target_yaw = target[0], target[1], target[2], target[3]
    target_yaw = target_yaw*(np.pi/180)
    
    cmd = PoseStamped()
    cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = target_x, target_y, target_z
    target_quaternion = tf.transformations.quaternion_from_euler(0, 0, target_yaw) 
    cmd.pose.orientation.x, cmd.pose.orientation.y = target_quaternion[0], target_quaternion[1]
    cmd.pose.orientation.z, cmd.pose.orientation.w = target_quaternion[2], target_quaternion[3]
    pub_targetpose.publish(cmd)

def flight_callback(globalposition, localposition, lidar):
    global NEXT_STATE, UAV
 
    if NEXT_STATE == 'TAKEOFF':
        NEXT_STATE = 'HOLD'
        rospy.loginfo("TAKEOFF-TRIGGERED")
        switch_modes("GUIDED")
        arming_call(arm = True)
        takeoff_call(globalposition.latitude, globalposition.longitude, altitude = 10)

    if NEXT_STATE == 'HOLD':
        NEXT_STATE = 'NAVIGATION'
        rospy.loginfo("HOLD-TRIGGERED")
        UAV.initalize_navigation(localposition, lidar)
        rospy.sleep(5)
        UAV.time_1 = time.time()

    if NEXT_STATE == 'LAND':
        NEXT_STATE = 'SHUTDOWN'
        rospy.loginfo("LAND-TRIGGERED")
        land_call(globalposition.latitude, globalposition.longitude, altitude = 0)

    if NEXT_STATE == 'SHUTDOWN':
        rospy.loginfo("SHUTDOWN-TRIGGERED")
        rospy.signal_shutdown("Visited-All-Points-And-Returned")
    
    if NEXT_STATE == 'NAVIGATION':
        NEXT_STATE = 'NAVIGATION'
        #rospy.loginfo("NAVIGATION-TRIGGERED")

        UAV.time_2 = time.time()
        time_difference = UAV.time_2 - UAV.time_1
        if time_difference > UAV.TIMEOUT:
            rospy.logwarn("TIMEDOUT")
            rospy.logwarn("SKIPPING-CURRENT-POINT")
            UAV.update_current_point()
            UAV.time_1 = time.time()

        if UAV.completed_points == True:
            NEXT_STATE = 'REVERSE_TRAVEL'
            rospy.logwarn("COMPLETED-ALL-POINTS")
            UAV.initialize_reverse_travel()
            return

        ARRIVED_STATUS = get_arrived_status([UAV.target_x, UAV.target_y], localposition, tolerance = 0.5)
        if ARRIVED_STATUS == True:
            rospy.logwarn("ARRIVED-AT-TARGET/GETTING-NEW-TARGET")

            DESTINATION_STATUS = get_arrived_status([UAV.current_point_x, UAV.current_point_y], localposition, tolerance = 3)
            if DESTINATION_STATUS == True:
                NEXT_STATE = 'YAW_MOVEMENT'
                rospy.logwarn("ARRVIED-AT-DESTINATION") 
                return

            objects_x, objects_y = get_objects(localposition, lidar)
            UAV.update_target(localposition.pose.position.x, localposition.pose.position.y, objects_x, objects_y)
        target_yaw = get_target_yaw([UAV.target_x, UAV.target_y], localposition)
        goto([UAV.target_x, UAV.target_y, localposition.pose.position.z, target_yaw])

    if NEXT_STATE == 'YAW_MOVEMENT':
        NEXT_STATE = 'NAVIGATION'
        rospy.loginfo('YAW_MOVEMENT-TRIGGERED')
        goto([localposition.pose.position.x, localposition.pose.position.y, localposition.pose.position.z, UAV.current_point_yaw])
        UAV.update_current_point()
        rospy.sleep(20)
        UAV.time_1 = time.time()
        
    if NEXT_STATE == 'REVERSE_TRAVEL':
        NEXT_STATE = 'REVERSE_TRAVEL'
        #rospy.loginfo("REVERSE_TRAVEL-TRIGGERED")

        ARRIVED_STATUS = get_arrived_status([UAV.reverse_target_x, UAV.reverse_target_y], localposition, tolerance = 0.5)
        if ARRIVED_STATUS == True:
            rospy.logwarn("ARRIVED-AT-TARGET/GETTING-NEW-TARGET")
            
            UAV.update_reverse_target()
            if UAV.reverse_target_index < 0:
                NEXT_STATE = 'LAND'
                return

        target_yaw = get_target_yaw([UAV.reverse_target_x, UAV.reverse_target_y], localposition)
        goto([UAV.reverse_target_x, UAV.reverse_target_y, localposition.pose.position.z, target_yaw])
        
if __name__ == '__main__':
    rospy.init_node("UAV_node")
    rospy.loginfo("UAV_node Started")
    
    pub_targetpose = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 1)

    sub_localposition = message_filters.Subscriber("/mavros/local_position/pose", PoseStamped)
    sub_globalposition = message_filters.Subscriber("/mavros/global_position/global", NavSatFix)
    sub_lidar = message_filters.Subscriber("/spur/laser/scan", LaserScan)
    ats = message_filters.ApproximateTimeSynchronizer([sub_globalposition, sub_localposition, sub_lidar], queue_size = 1, slop = 0.2)
    ats.registerCallback(flight_callback)

    rospy.spin()
    rospy.loginfo("\nUAV_node Closed")
