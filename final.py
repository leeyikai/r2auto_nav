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
# import copy


# constants
rotatechange = 0.05
speedchange = 0.2
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

map_bg_color = 1
points_skip = 10
ignore_angle = 5

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

    def padding(self):
        # self.get_logger().info('In padding')
        arr = [[self.occdata[i,j] for j in range(len(self.occdata[0]))] for i in range(len(self.occdata))]
        row = len(arr)
        col = len(arr[0])
        
        # pad surrounding 8, 3 times the area
        # print("In padding")
        for i in range(row):
            for j in range(col):
                if self.occdata[i,j] == 1:
                    check = [(i+1,j), (i+1,j+1), (i+1,j-1), (i-1,j), (i-1,j-1), (i-1,j+1), (i,j+1), (i,j-1)]         
                    for loc in check:
                        x = loc[0]
                        y = loc[1]
                        if 0<=x<=row and 0<=y<=col:
                            arr[x][y] == 1
                    
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
        # get width and height of map
        iwidth = msg.info.width
        iheight = msg.info.height
        # calculate total number of bins
        # total_bins = iwidth * iheight
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('No transformation found')
            return
        
        global cur_pos
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
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
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        self.occdata = np.array(binnum.reshape(msg.info.height,msg.info.width))
        
        # print to file
        np.savetxt('maptry.txt', self.occdata)
        # print('saved file')
        
        # print(self.occdata)
        self.occdata = self.occdata - 2
        
        with open("before padding.txt", "wb") as f:
            print('writing to file')
            np.savetxt(f, self.occdata.astype(int), fmt='%i', delimiter=",")
            
        # print('before padding:', self.occdata)
        self.padding()
        self.padding()
        # self.padding()
        
        with open("after padding.txt", "wb") as f:
            print('writing to file')
            np.savetxt(f, self.occdata.astype(int), fmt='%i', delimiter=",")
        # print('after padding:', self.occdata)
        
        # print to file
        np.savetxt(mapfile, self.occdata)
        # print('saved file')
        

        # set current robot location to 0
        odata[grid_y][grid_x] = 0
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/ 
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)
            
        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)
            
        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
        img_transformed.paste(img, (left, top))

        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)

        # show the image using grayscale map
        # plt.imshow(img, cmap='gray', origin='lower')
        # plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
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
        self.get_logger().info('In rotatebot')
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
        
        
##--------------------------------------------------
##--------------------------------------------------
##--------------------------------------------------

    ## checking neighbours
    def neighbours(self, arr, i, j):
        self.get_logger().info('In neighbours')
        
        # (row, col) = self.occdata.shape
        row = len(arr)
        col = len(arr[0])
        
        # unmap_count = 0
        # empty_count = 0
        count = 0

        # check = [[i+1,j], [i+1,j+1], [i+1,j-1], [i-1,j], [i-1,j-1], [i-1,j+1], [i,j+1], [i,j-1]]
        check = [[i+1,j], [i-1,j], [i,j+1], [i,j-1]]
        for loc in check:
            x = loc[0]
            y = loc[1]
            if 0<=x<=row and 0<=y<=col and arr[x][y] == -1:
                count += 1
                
        if count == 2:
            self.get_logger().info('Found neighbours')
            print(i,j, arr[i][j])
            # check = [[i+1,j], [i+1,j+1], [i+1,j-1], [i-1,j], [i-1,j-1], [i-1,j+1], [i,j+1], [i,j-1]]
            check = [[i+1,j], [i-1,j], [i,j+1], [i,j-1]]
            for loc in check:
                x = loc[0]
                y = loc[1]
                if 0<=x<=row and 0<=y<=col:
                    print(x,y, arr[x][y])
            
            return (False, i,j)
        
        return (True, i, j)
    
    
            # if 0<=x<row and 0<=y<col:
            #     if arr[x][y] == -1:
            #     # print('found -1')
            #         unmap_count += 1
            #     if arr[x][y] == 0:
            #     # print('found -1')
            #         empty_count += 1
            # if unmap_count > 2 and empty_count > 3:
            #     self.get_logger().info('Found destination')
            #     return (False, i,j)
            
        return (True, i, j)
    
##--------------------------------------------------
##--------------------------------------------------
##--------------------------------------------------
        
        
    ## mapping algo
    ## looping through occdata to find next location to go to
    def pick_direction(self):
        self.get_logger().info('In pick_direction')
        
        if self.occdata.size == 0:
            self.get_logger().info('No self.occdata, moving forward')
            
            # start moving
            self.get_logger().info('Start moving')
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.publisher_.publish(twist)
            return 
        
        # arr = self.occdata  
        # print('hi')
        (row, col) = self.occdata.shape
        # print('hi')
        arr = [[self.occdata[i,j] for j in range(col)] for i in range(row)]
        # print('hi')
        # print('printing arr in pick_direction')
        # print(arr)
       
        # print('row, col:', row, col)
        
        # print('before double for loop')
        
        for i in range(row):
            for j in range(col):
                # print('in double for loop of pick_direction', i, j, arr[i, j])
                if arr[i][j] == 0:
                    # print("cell is zero, calling neighbours")
                    (boolean,x,y) = self.neighbours(arr, i, j)
                    if boolean == False:
                        # print('hello')
                        print('curr:', cur_pos.x, cur_pos.y, grid_x, grid_y)
                        print('destination:', x*map_res+map_origin.x , y*map_res+map_origin.y, x,y, arr[x][y])
                        foo = self.gotoBFS(arr, grid_x, grid_y, x, y)
                        if foo == False:
                            continue 
                        else:
                            return 

        ## stop mapping function 
        # return 

##--------------------------------------------------
##--------------------------------------------------
##--------------------------------------------------


    ## go to next location directly 
    def goto(self, i, j, x, y):
        self.get_logger().info('In goto')
        print(i,j,x,y)
        # foo = (y-j)/(x-i)
        # print(y-j, x-i, foo, math.atan(foo), math.degrees(math.atan(foo)))
        angle = math.degrees(math.atan2((y-j),(x-i))) % 360
        
        # angle = (math.degrees(math.atan(foo))) % 360
        distance = ((x-i)**2+(y-j)**2)**0.5
        move_time = distance/speedchange
        print(angle,distance, move_time)

        # rotate to that direction
        if angle > ignore_angle:
            self.get_logger().info('Rotating first, picked direction: %d %f m ' % (angle, distance))
            self.rotatebot(float(angle))
            

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
        
        # if self.laser_range.size != 0:
        #     # check distances in front of TurtleBot and find values less
        #     # than stop_distance
        #     lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
        #     # self.get_logger().info('Distances: %s' % str(lri))

        #     # if the list is not empty
        #     if(len(lri[0])>0):
        #         # stop moving
        #         return 
                        
        self.get_logger().info('In goto, Stop moving')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
               
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)
        print('done moving')
        



##--------------------------------------------------
##--------------------------------------------------
##--------------------------------------------------
    ## BFS and flooding algorithm 
    def bfs(self, a, start, end):
        self.get_logger().info('In bfs')
        print(start,a[start[0]][start[1]] ,end, a[end[0]][end[1]])
        # print to file
        # np.savetxt('gotomap.txt', self.occdata.astype(np.int))
        
        with open("bfs_map.txt", "wb") as f:
            print('writing to file')
            np.savetxt(f, self.occdata.astype(int), fmt='%i', delimiter=",")
            
        def make_step(m, k):
            for i in range(len(m)):
                for j in range(len(m[i])):
                    print('in double loop')
                    if m[i][j] == k:
                        print('in check')
                        if i>0 and m[i-1][j] == 0 and a[i-1][j] == 0:
                          m[i-1][j] = k + 1
                          print('1step')
                        if j>0 and m[i][j-1] == 0 and a[i][j-1] == 0:
                          m[i][j-1] = k + 1
                          print('2step')
                        if i<len(m)-1 and m[i+1][j] == 0 and a[i+1][j] == 0:
                          m[i+1][j] = k + 1
                          print('3step')
                        if j<len(m[i])-1 and m[i][j+1] == 0 and a[i][j+1] == 0:
                           m[i][j+1] = k + 1
                           print('4step')
                           
        def print_m(m):
            for i in range(len(m)):
                for j in range(len(m[i])):
                    print( str(m[i][j]).ljust(2),end=' ')
                print()
        
        print('create arr')
        
        m = []
        for i in range(len(a)):
            m.append([])
            for j in range(len(a[i])):
                m[-1].append(0)
        i,j = start
        m[i][j] = 1
        
        print_m(m)
        
        print('starting to step')
        k = 0
        while m[end[0]][end[1]] == 0:
            print('step', k)
            k += 1
            make_step(m, k)
        
        
        print('backtracking')
        i, j = end
        k = m[i][j]
        the_path = [(i,j)]
        while k > 1:
          if i > 0 and m[i - 1][j] == k-1:
            i, j = i-1, j
            the_path.append((i, j))
            k-=1
          elif j > 0 and m[i][j - 1] == k-1:
            i, j = i, j-1
            the_path.append((i, j))
            k-=1
          elif i < len(m) - 1 and m[i + 1][j] == k-1:
            i, j = i+1, j
            the_path.append((i, j))
            k-=1
          elif j < len(m[i]) - 1 and m[i][j + 1] == k-1:
            i, j = i, j+1
            the_path.append((i, j))
            k -= 1
            
            
        # print_m(m)
        print(the_path)
        print('done')
        return the_path
    


##--------------------------------------------------
##--------------------------------------------------
##--------------------------------------------------

    ## go to next location via bfs
    def gotoBFS(self, arr, i, j, x, y):
        self.get_logger().info('In gotoBFS')
        # print(i,j, arr[i,j],x,y, arr[x,y])
        path = self.bfs(arr, (i,j), (x,y))
        
        if path == False:
            return False
        
        print('length path', len(path))

        ## calibrate last value in range for point skipping 
        print('going')
        for count in range(points_skip,len(path), points_skip):
            x = path[count][0]
            y = path[count][1]
            print(count, cur_pos.x, cur_pos.y, x, y)
            self.goto(cur_pos.x, cur_pos.y, x, y)
            cur_pos.x = x
            cur_pos.y = y


        # last iteration to reach destination 
        x = path[-1][0]
        y = path[-1][1]
        self.goto(i, j, x, y)
        print("done gotoBFS")
        return True
        



##--------------------------------------------------
##--------------------------------------------------
##--------------------------------------------------

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
