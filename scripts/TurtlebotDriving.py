import rospy
import tf
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import math
import random
import numpy as np
import cv2, cv_bridge

class Pose2():
    x = 0
    y = 0
    theta = 0

class TurtlebotDriving:
    def __init__(self):
        self.x_history = []
        self.y_history = []
        rospy.init_node('PythonControl')
        self.pose = Pose2()
        self.scan = LaserScan()
        self.image = None
        self.depth_img = None
        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('camera/depth/image_raw', Image, self.depth_callback)
        rospy.sleep(2)

# ---------------------------------------- Driving ----------------------------------------

    def walk(self, distance, speed=0.2):
        self.previousX = self.pose.x 
        self.previousY = self.pose.y

        rate = rospy.Rate(10)

        twist = Twist()
        twist.linear.x = speed
    
        travelled_distance = 0
        
        while(travelled_distance < distance):
            self.cmd_vel_pub.publish(twist)
            print(travelled_distance)
            dx = self.pose.x - self.previousX
            dy = self.pose.y - self.previousY
            travelled_distance = math.sqrt(dx*dx + dy*dy)
            rate.sleep()
        
        self.wait(1)



    def rotate(self, alpha, speed=0.2):
        self.previousT = self.pose.theta
        angle  = 2* math.pi * alpha/360 
        
        twist = Twist()
        twist.angular.z = speed    
        rate = rospy.Rate(10)

        travelled_angle = 0

        while travelled_angle < angle:
            travelled_angle = self.pose.theta - self.previousT
            if self.pose.theta + 3 < self.previousT:
                travelled_angle += 6.28
            self.cmd_vel_pub.publish(twist)
            print(self.pose.theta)
            rate.sleep()
        self.wait(1)



    def wait(self, duration):
        wait = Twist()
        time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)

        while rospy.Time.now().to_sec() - time <= rospy.Duration(duration).to_sec():
            self.cmd_vel_pub.publish(wait)
            rate.sleep() 



    def move(self, path):
        for i in range(len(path)-1):
            current = path[i]
            next = path[i+1]

            difference = tuple(map(lambda i,j: i-j, next, current))

            if difference[0]:
                if i == 0:
                    self.walk(0.5)
                else:
                    self.rotate(-90)
                    self.walk(0.5)

            if difference[1]:
                self.rotate(90)
                self.walk(0.5)
                
                

    def orient(self, desired_angle):
        self.previousT = self.pose.theta
        desired_angle = 2* math.pi * desired_angle/360
            

    def plot_trajectory(self):
        plt.plot(self.x_history, self.y_history)
        plt.xticks(np.arange(-2, 3, 1.0))
        plt.yticks(np.arange(-2, 3, 1.0))
        plt.show()

# ---------------------------------------- Callbacks ----------------------------------------

    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.x_history.append(self.pose.x)
        self.y_history.append(self.pose.y)
    

    def scan_callback(self, msg):
        self.scan = msg
        self.scan.ranges = np.array(self.scan.ranges)


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        image_resized = cv2.resize(image, (w/4,h/4))
        self.image = image_resized


    def depth_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,'passthrough')
        (h, w) = image.shape[:2]
        image_resized = cv2.resize(image, (w/4,h/4))
        self.depth_img = image_resized
        

# ---------------------------------------- Misc. ----------------------------------------

    def checkObstacle(self, degree=75, distance=0.5):
        position = degree//2
        check_range = np.append(self.scan.ranges[0:position], self.scan.ranges[-position:])
        check_range[check_range == 0] = np.inf
        if (check_range < distance).any():
            print("There is an obstacle")
            return True
        return False

    
    def avoidObstacle(self, degree=75, distance=0.5):
        rate = rospy.Rate(10)
        print('avoidObstacle triggered.')
        position = degree//2

        if np.sum(self.scan.ranges[0:position]) > np.sum(self.scan.ranges[-position:]):
            direction = 1
        else:
            direction = -1

        check_range = np.append(self.scan.ranges[0:position], self.scan.ranges[-position:])
        check_range[check_range == 0] = np.inf
        has_obstacle = True

        speed = 0.2
        twist = Twist()
        twist.angular.z = speed * direction

        while has_obstacle:
            self.cmd_vel_pub.publish(twist)
            check_range = np.append(self.scan.ranges[0:position], self.scan.ranges[-position:])
            check_range[check_range == 0] = np.inf
            has_obstacle = False
            if (check_range < distance).any():
                has_obstacle = True
            rate.sleep()
            
        print("No Obstacle detected in this direction")
    

    def checkRightWall(self, distance):
        if self.scan.ranges[270]< distance:
            print("Right Wall Detected.")
            return True


    def followRightWall(self, distance):
        twist = Twist()
        error = distance - self.scan.ranges[270]
        turn = 0.7
        rate = rospy.Rate(10)

        if error < distance:
            error = distance - self.scan.ranges[270]
            final_speed = error * turn
            twist.angular.z = final_speed
            twist.linear.x = 0.05
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        elif error > distance:
            pass

        print("Lost the right wall")

        self.wait(1) 


    def relaunch(self):
        service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        service()
        self.__init__()
