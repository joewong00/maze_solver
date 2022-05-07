
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
        self.kp = 1.6
       
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.sleep(2)

# ---------------------------------------- Driving ----------------------------------------


    def turn(self, angle, speed = 0.7):
        """Rotate the turtlebot for a certain angle

        Args

        """

        # angle (+) = counterclockwise
        # angle (-) = clockwise

        print("Rotating "+str(angle)+"radian...")

        theta_init = self.pose.theta
        msg = Twist()
        travelled_angle = 0
        clockwise = np.sign(angle)
        angle = abs(angle)

        while abs(angle - travelled_angle) > 0.0015 and not rospy.is_shutdown():

            msg.angular.z = clockwise * min(self.kp * abs(angle - travelled_angle) , speed)

            # If clockwise
            if clockwise < 0:

                travelled_angle = theta_init - self.pose.theta

                # If pass through angle 0
                if self.pose.theta > theta_init:
                    travelled_angle += 2*math.pi

            # Counter-clockwise
            else:
                travelled_angle = self.pose.theta - theta_init

                # If pass through angle 0
                if self.pose.theta < theta_init:
                    travelled_angle += 2*math.pi

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

        self.wait(1)


    def rotateTo(self, angle, speed = 0.7):
        angle = angle % (2*math.pi)
        print("Rotating to "+str(angle)+"radian...")

        direction = 1
        angle_to_turn = abs(angle - self.pose.theta)

        # Minimum turning angle
        if angle_to_turn > math.pi:
            angle_to_turn = 2*math.pi - angle_to_turn


        # Determine turning direction
        if angle != 3*math.pi/2:
            if angle < self.pose.theta <= angle + 3.14:
                direction = -1

        else:
            if self.pose.theta > angle or self.pose.theta < angle - math.pi:
                direction = -1

        # Turn for certain angle
        self.turn(direction * angle_to_turn, speed)



    def forward(self, distance, speed=0.3):
        
        print("Moving forward "+str(distance)+"m ...")

        msg = Twist()

        # Robot's initial position
        x_init = self.pose.x
        y_init = self.pose.y

        avoid_d = 0.5
        travelled_distance = 0

 
        # While not up to desired distance
        while distance - travelled_distance > 0.005 and not rospy.is_shutdown():
        
            # Avoid obstacle on the front
            check_range = np.append(self.scan.ranges[:89],self.scan.ranges[-89:])
            msg.linear.x = min(self.kp * abs(distance - travelled_distance) ,speed)

            if (check_range < avoid_d).any():
                index = np.where(check_range < avoid_d)[0]
                index = (index - 89)/3000.0
                avoid_error = index.sum()
                msg.angular.z = avoid_error
                
            else:
                msg.angular.z = 0

            # Calculate travelled distance
            travelled_distance = math.sqrt((self.pose.x - x_init)**2 + (self.pose.y - y_init)**2)
            self.cmd_vel_pub.publish(msg)

            # print(travelled_distance)
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

        self.rotateTo(angle)
        self.forward(distance)
            

    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0

        self.cmd_vel_pub.publish(msg)
        self.rate.sleep()
    

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

        self.scan_max_value = msg.range_max


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


    def forward_kinematics(self, w_l, w_r):
        c_l = 0.033 * w_l
        c_r = 0.033 * w_r
        v = (c_l + c_r)/2
        a = 0.5 * (c_r - c_l)/ 0.08

        return (v,a)


    def plot_trajectory(self, algorithm):
        plt.plot(self.y_history, self.x_history)
        plt.title('Robot Trajectory with '+str(algorithm))
        plt.axis([10, -10, -10, 10])
        plt.show()


    def relaunch(self):
        service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        service()
        self.__init__()

