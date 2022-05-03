from dis import dis
from tabnanny import check
import rospy
import tf
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class Pose2D():
    x = 0
    y = 0
    theta = 0


class TurtlebotDriving:
    def __init__(self):
        self.x_history = []
        self.y_history = []

        rospy.init_node('PythonControl')

        self.pose = Pose2D()
        self.scan = LaserScan()
        self.rate = rospy.Rate(10)
        self.kp = 1.5
       
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.sleep(2)

# ---------------------------------------- Driving ----------------------------------------

    
    def turn(self, angle, speed = 0.2):
        angle  = 2* math.pi * angle/360 
        num_secs = angle / speed
        twist = Twist()
        twist.angular.z = speed

        rate = rospy.Rate(10)
        time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - time < rospy.Duration(num_secs).to_sec():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.wait(1)


    def rotate(self, angle, speed = 0.7):
        angle = angle % (2*math.pi)
        print("Rotating to "+str(angle)+"radian...")
        msg = Twist()

        while abs(angle - self.pose.theta) > 0.002 and not rospy.is_shutdown():

            if angle - self.pose.theta < 0:
                # if desired angle is 0
                if angle == 0:
                    if self.pose.theta > math.pi:
                        msg.angular.z = min(self.kp * abs(angle - self.pose.theta),speed)

                    else:
                        msg.angular.z = max(self.kp *(angle - self.pose.theta),-speed)

                else:
                    msg.angular.z = max(self.kp *(angle - self.pose.theta),-speed)

            else:
                msg.angular.z = min(self.kp *(angle - self.pose.theta),speed)

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
            
        self.wait(1)


    def forward(self, distance, speed=0.3):
        
        print("Moving forward...")

        msg = Twist()
        msg.linear.x = speed

        num_secs = distance / speed
        avoid_d = 0.5
 
    
        time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - time < rospy.Duration(num_secs).to_sec() and not rospy.is_shutdown():

            check_range = np.append(self.scan.ranges[:89],self.scan.ranges[-89:])

            if (check_range < avoid_d).any():
                index = np.where(check_range < avoid_d)[0]
                index = (index - 89)/3000.0
                avoid_error = index.sum()
                msg.angular.z = avoid_error
                
            else:
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

        self.wait(1)



    def wait(self, duration):
        wait = Twist()
        time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - time <= rospy.Duration(duration).to_sec():
            self.cmd_vel_pub.publish(wait)
            self.rate.sleep() 



    def move(self, current, waypoint):

        difference = tuple(map(lambda i,j: i-j, waypoint, current))
        inc_x, inc_y = difference
        angle = math.atan2(inc_y, inc_x)
        angle = angle - math.pi
        distance = math.sqrt((waypoint[0] - current[0])**2 + (waypoint[1] - current[1])**2)

        self.rotate(angle)
        self.forward(distance)
            

    def plot_trajectory(self, algorithm):
        plt.plot(self.y_history, self.x_history)
        plt.title('Robot Trajectory Solved With '+str(algorithm))
        plt.axis([10, -10, -10, 10])
        plt.show()

# ---------------------------------------- Callbacks ----------------------------------------

    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw % (2*math.pi)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.x_history.append(self.pose.x)
        self.y_history.append(self.pose.y)
    

    def scan_callback(self, msg):
        self.scan = msg
        self.scan.ranges = np.array(self.scan.ranges)

        front = np.append(self.scan.ranges[0:90], self.scan.ranges[-90:])

        self.region = {
            'front': min(min(front),10),
            'left': min(min(msg.ranges[67:113]),10),
            'right': min(min(msg.ranges[246:293]),10),
        }


# ---------------------------------------- Misc. ----------------------------------------



    def checkObstacle(self, degree=90, distance=0.4):
        position = degree//2
        check_range = np.append(self.scan.ranges[0:position], self.scan.ranges[-position:])
        check_range[check_range == 0] = np.inf
        if (check_range < distance).any():
            print("Obstacle found")
            return True
        return False

    
    def avoidObstacle(self, degree=90, distance=0.4):
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
            if (check_range > distance).any():
                has_obstacle = False
            rate.sleep()
            
        print("No Obstacle detected in this direction")

    

    def checkRightWall(self, distance):
        if self.region['right'] < distance:
            print("Right Wall Detected.")
            return True


    def followRightWall(self, distance=0.5):

        print("Following right wall..")
        msg = Twist()

        while self.region['front'] < 10 and not rospy.is_shutdown():

            if self.scan.ranges[270] < distance and self.scan.ranges[0] > distance:
                # move straiight
                print("straight")
                msg.linear.x = 0.2
                msg.angular.z = 0
                self.cmd_vel_pub.publish(msg)
                self.rate.sleep() 
                

            elif self.scan.ranges[270] < distance and self.scan.ranges[0] < distance:
                # turn left
                print("left")
                self.turn(angle=90)

            elif self.scan.ranges[270] > distance:
                # turn right
                print("right")
                self.turn(angle=-90)

            else:
                pass


        self.wait(1) 


    def relaunch(self):
        service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        service()
        self.__init__()
