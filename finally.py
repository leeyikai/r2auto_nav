# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time

# from r2occupancy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import matplotlib.pyplot as plt
from PIL import Image
import scipy.stats

# constants
rotatechange = 0.15
speedchange = 0.15
occ_bins = [-1, 0, 51, 101]
stop_distance = 0.5
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

points_skip = 10
# ignore_angle = 5

unmap = 1
empty_space = 2
wall = 3

pad = 2

bfs_colour = 200
# bot_colour = 100

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        
        
    def padding(self, pad):
        # self.get_logger().info('In padding')
        arr = [[self.occdata[i,j] for j in range(len(self.occdata[0]))] for i in range(len(self.occdata))]
        row = len(arr)
        col = len(arr[0])
        

        for i in range(row):
            for j in range(col):
                if self.occdata[i,j] == wall:
                    for k in range(i-pad, i+pad+1):
                        for l in range(j-pad, j+pad+1):        
                            if 0<=k<=row and 0<=l<=col:
                                arr[k][l] == wall
                    
        self.occ_data = arr
        # print('done padding')
        return

    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        self.occdata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        self.occdata = self.occdata.astype(int)
        self.padding(pad)
        
        # print('done self.occdata')
        
        # get width and height of map
        # iwidth = msg.info.width
        # iheight = msg.info.height
        # calculate total number of bins
        # total_bins = iwidth * iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        global got_trans
        
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            got_trans = True
        except (LookupException, ConnectivityException, ExtrapolationException):
            got_trans = False
            self.get_logger().info('No transformation found')
            return
        
        global cur_pos
        global cur_rot
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %.2f, %.2f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, self.yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution
        global map_res
        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        global map_origin
        map_origin = msg.info.origin.position
        # get map grid positions for x, y position
        
        global grid_x
        global grid_y
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        self.get_logger().info('Grid X: %i Grid Y: %i, value: %i' % (grid_x, grid_y, self.occdata[grid_x, grid_y]))
        # self.get_logger().info('Grid Y: %i Grid X: %i, value: %i' % (grid_y, grid_x, self.occdata[grid_y, grid_x]))
        
        # print to file
        # np.savetxt(mapfile, self.occdata)
        
        # self.occdata = bot_colour
        img = Image.fromarray((self.occdata * 255).astype(np.uint8))
        # 
        
        # img = Image.fromarray(self.occdata)
        # omap = np.loadtxt(mapfile)
        plt.imshow(img, origin='lower')
        # plt.show()
        plt.draw_all()
        plt.pause(0.00000000001)
        



    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def moveforward(self):
        self.get_logger().info('moveforward')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)
    
    
    
    
    def turn_and_move_forward(self, i, j, x, y):
        self.get_logger().info('In turn_and_move_forward')
        
        i = i * map_res + map_origin.x
        x = x * map_res + map_origin.x
        y = y * map_res + map_origin.y
        y = y * map_res + map_origin.y
        # print(i,j,x,y)

        target = math.degrees(math.atan2((y-j),(x-i))) % 360
        yaw_degrees = math.degrees(self.yaw) % 360 
        angle = (target - yaw_degrees) % 360 

        distance = ((x-i)**2+(y-j)**2)**0.5
        # move_time = distance/speedchange
        # print(yaw_degrees, target, angle,distance, move_time)

        # rotate to that direction
        # if abs(yaw_degrees - target) > ignore_angle:
        self.get_logger().info('Rotating first, picked direction: %d %f m ' % (angle, distance))
        self.rotatebot(float(angle))
        # self.yaw = math.radians(target)
        # else:
        #     self.get_logger().info('Current angle: %d, desired angle: %d, angle negligible, not rotating' % (math.degrees(self.yaw), target))
            
        
        self.moveforward()
        
        

    ## go to next location directly 
    def goto(self, i, j, x, y):
        print(i,j,x,y)
        i = i * map_res + map_origin.x
        x = x * map_res + map_origin.x
        j = j * map_res + map_origin.y
        y = y * map_res + map_origin.y
        
        self.get_logger().info('In goto')
        print(i,j,x,y)
        # self.get_logger().info(' X: %i  Y: %i, Target  X: %i  Grid Y: %i' % (i, j, x, y))

        target = math.degrees(math.atan2((y-j),(x-i)))  % 360
        yaw_degrees = math.degrees(self.yaw) % 360 
        angle = (target - yaw_degrees) % 360 

        distance = ((x-i)**2+(y-j)**2)**0.5
        move_time = distance/speedchange
        print(yaw_degrees, target, angle,distance, move_time)

        # rotate to that direction
        # if abs(yaw_degrees - target) > ignore_angle:
        self.get_logger().info('Rotating first, picked direction: %d %.2f m ' % (angle, distance))
        self.rotatebot(float(angle))
        # self.yaw = math.radians(target)
        # else:
        #     self.get_logger().info('Current angle: %d, desired angle: %d, angle negligible, not rotating' % (math.degrees(self.yaw), target))
            
        # 
        if distance > 0:
            # start moving
            # self.get_logger().info('Start moving')
            print('In goto, Start moving')
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            
            time.sleep(1)
            self.publisher_.publish(twist)
            # startTime = time.time()
            print('moving')
            
            
            # move for this amount of time
            time.sleep(move_time)
            # endTime = time.time()
            # print('duration moving', endTime - startTime)
            
            self.get_logger().info('In goto, Stop moving')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
                   
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            # time.sleep(1)
            self.publisher_.publish(twist)
        
        print('done moving')



    def getAdjacentSpaces(self, maze, space, visited):
         ''' Returns all legal spaces surrounding the current space
         :param space: tuple containing coordinates (row, col)
         :return: all legal spaces
         '''
         spaces = list()
         spaces.append((space[0]-1, space[1]))  # Up
         spaces.append((space[0]+1, space[1]))  # Down
         spaces.append((space[0], space[1]-1))  # Left
         spaces.append((space[0], space[1]+1))  # Right
         spaces.append((space[0]-1, space[1]+1))  # Up right 
         spaces.append((space[0]+1, space[1]-1))  # Down left 
         spaces.append((space[0]-1, space[1]-1))  # Up Left 
         spaces.append((space[0]+1, space[1]+1))  # Down Right
         
         # spaces.append((space[1]-1, space[0]))  # Up
         # spaces.append((space[1]+1, space[0]))  # Down
         # spaces.append((space[1], space[0]-1))  # Left
         # spaces.append((space[1], space[0]+1))  # Right
         # spaces.append((space[1]-1, space[0]+1))  # Up right 
         # spaces.append((space[1]+1, space[0]-1))  # Down left 
         # spaces.append((space[1]-1, space[0]-1))  # Up Left 
         # spaces.append((space[1]+1, space[0]+1))  # Down Right
         
         row = len(maze)
         col = len(maze[0])
         
         # print('hi', spaces)
         final = list()
         for i in spaces:
             # print(i[0], i[1], row, col )
             if 0 <= i[0] < row and 0 <= i[1] < col:
                 # print('test', maze[i[0]][i[1]])
                 if maze[i[0], i[1]] != wall and i not in visited:
                     final.append(i)
         # print('done adj')
         return final
 
    
    def bfs(self, maze, start, endCond):
        self.get_logger().info('In bfs')
        # maze = self.occdata

        basic_operations = 0
        # print('in bfs')
        '''"Brute-Force Search"
        :param maze(list): the maze to be navigated
        :param start(tuple): the starting coordinates (row, col)
        :param end(tuple): the end coordinates (row, col)
        :return: shortest path from start to end
        '''
        queue = [(start)]
        visited = set()
    
        while len(queue) != 0:
            if queue[0] == start:
                path = [queue.pop(0)]  # Required due to a quirk with tuples in Python
            else:
                path = queue.pop(0)
            front = path[-1]
            # print(front)
            if maze[front[0], front[1]] == endCond:
                return path
            elif front not in visited:
                for adjacentSpace in self.getAdjacentSpaces(maze, front, visited):
                    newPath = list(path)
                    newPath.append(adjacentSpace)
                    queue.append(newPath)
    
                    basic_operations += 1
                visited.add(front)
        return None




    def pick_direction(self):
        self.get_logger().info('In pick_direction')
        
        if self.occdata.size == 0:
            # if self.occdata.size == 0 or not got_trans:
            self.get_logger().info('No self.occdata, moving forward')
            return self.moveforward()
        
        #to make sure the starting point isnt unmap for whatever reason `
        # self.occdata[grid_y, grid_x] = empty_spaceS
        # path = self.bfs(self.occdata, (grid_y, grid_x), unmap)
        
        self.occdata[grid_x, grid_y] = empty_space
        # print(self.occdata[grid_y, grid_x])
        
        path = self.bfs(self.occdata, (grid_x, grid_y), unmap)
        print('path', path)
        
        if len(path) < points_skip:
            print('short path, going directly')
            x = path[-1][0] 
            y = path[-1][1] 
            
            # return self.turn_and_move_forward(cur_pos.x, cur_pos.y, x, y)
            return self.turn_and_move_forward(cur_pos.y, cur_pos.x, -y, x)
        
        
        print('long path, going indirectly')
        for count in range(len(path)):
            x = path[count][0] 
            y = path[count][1]
            self.occdata[x,y] = bfs_colour
        
        plt.figure()
        img = Image.fromarray((self.occdata * 255).astype(np.uint8))
        # img = Image.fromarray(self.occdata)
        # omap = np.loadtxt(mapfile)
        plt.imshow(img, origin='lower')
        plt.show()
        # # plt.draw_all()
        # # plt.pause(0.00000000001)
        plt.pause(2)
        
        
        for count in range(0, len(path), points_skip):
            i = path[count][0]
            j = path[count][0]
            x = path[count+1][0] 
            y = path[count+1][1] 

            self.goto(j, i, -y, x)
            # self.goto(i, j, x, y)
            # self.goto(cur_pos.x, -cur_pos.y, x, -y)
            
            # print('[LOOK HERE] gotoBFS:', count, grid_x, grid_y, cur_pos.x, cur_pos.y)
            # print('[LOOK HERE] gotoBFS:', path[count][0], path[count][1], x, y)
            # print()
            
            # cur_pos.x = x
            # cur_pos.y = y
        
        
        self.moveforward()

        print('done pick_direction')
        return 



    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()

            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()