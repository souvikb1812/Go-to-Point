#!/usr/bin/env python3

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pi
 
# angle and distant constraints
# you can adjust the values for better performance
angle_eps = 0.03
dis_eps = 0.07

# setting the maximum velocities
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:
    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        self._sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates
    # Remember Figure 1 in Project 2!!!
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        if self.theta < 0:
            self.theta += 2*pi

def main():

    global pub

    # initialize ROS node
    rospy.init_node("go_to_point")
    
    # run stop function when this node is killed
    rospy.on_shutdown(stop)

    # initialize odom class
    odom = odomReader()
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency 
    r = rospy.Rate(1000)

    # Setting the goal points
    goals = list()
    goals.append(Point(2,2,0))
    goals.append(Point(-2,2,0))
    goals.append(Point(0,0,0))

    # Starting point
    # Remember to update this point when you reach
    # one of the goal points.
    #init_point = Point(0,0,0)
    

    # TODO: Implement an upgraded version of Algorithm 1
    # which works for multiple destinations.

    for goal in goals:
        while not rospy.is_shutdown():
        
            '''
            Hint: 
            Here's a hint on how to use the "speed" variable.
            Remember about Figure 1, and try to apply the following:
            linear velocity -> speed.linear.(axis of linear movement, should be either x, y, or z)
            angular velocity -> speed.angular.(axis of rotation, should be either x, y, or z)
            '''
            # find the x,y distance to the goal from current position
            inc_x=goal.x-odom.x
            inc_y=goal.y-odom.y

            # find the angle of the goal point to the initial point
            # where the robot starts to rotate on the global frame
            angle_to_goal=atan2(inc_y,inc_x)
            if (angle_to_goal<0):
                angle_to_goal+=2*pi

            # find the angle difference between breakthe robot's sensor(odometry)
            # and the one you found above
            angle_diff=(angle_to_goal-odom.theta)
            

            # find the Euclidean distance between the current robot's location
            # and the current destinations
            dist_diff=sqrt((inc_x*inc_x)+(inc_y*inc_y))

            # write an if-else statement which determine whether 
            # angular or linear velocity is applied to the robot
            # and when it reaches a goal point, update the initial point
            # and move on to the next goal point.
            
            if (angle_diff>0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=0.02 # rotating anti-clockwise

            elif (angle_diff<-0.1):

                speed.linear.x=0.0 # stopping linear movement
                speed.angular.z=-0.02 # rotating clockwise
                

            elif(dist_diff>=0.2):

                speed.linear.x=0.11 # setting linear velocity
                speed.angular.z=0.0 # stopping rotation

            else:

                speed.linear.x = 0.0 # stopping linear movement
                speed.angular.z = 0.0 # stopping rotation
                break
    
	
            # print the data for debugging purposes
            print("x: %.3f, \t y: %.3f, \ttheta: %.3f, \t ang_diff: %.3f, \tdist_diff: %.3f" 
            %(odom.x, odom.y, odom.theta, angle_diff, dist_diff))

            # when you are done making adjustmenst to the speed publish the speed
            pub.publish(speed)
            # ROS functionality that will maintain constant loop frequency
            r.sleep() 


# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)


if __name__=='__main__':
	main()
