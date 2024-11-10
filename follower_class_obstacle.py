#!/usr/bin/env python3
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import math
from math import radians

class Follower:
  def __init__(self,Kp=0.01,
		Ki=0.0006,
		Kd=0.005,
		threshold=0.4,
		previous_error=0,
		integral=0,
		err=0,
		b=1,
		flag=0): #flag to switch control between line follow and obstacle avoidance
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.image_callback)
                                      
    self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
                                       
    self.scan_sub=rospy.Subscriber('/scan',LaserScan,self.scan_callback)
    self.twist = Twist()
    self.integral=integral
    self.previous_error=previous_error
    self.Kp=Kp
    self.Ki=Ki
    self.Kd=Kd
    self.threshold=threshold
    self.flag=flag
    self.err=err
    self.b=b
  
  
  #defining function for line follow control
  def line_follow_control(self,err):
   
    rate = rospy.Rate(5)


    while not rospy.is_shutdown():
      if self.b > self.threshold:
        self.integral+=self.err
        derivative=self.err-self.previous_error
        angular_z=self.Kp*self.err + self.Ki*self.integral + self.Kd*derivative
        self.previous_error = self.err
        self.twist.linear.x = 0.1
        self.twist.angular.z = -angular_z/4
        self.cmd_vel_pub.publish(self.twist)
        
        rate.sleep()
      else:
        self.flag=1
        break
    
      
    
   
   
#defining function for obstacle avoidance  
  def avoid_obstacle(self):
  
    
   
    self.twist.linear.x=0
    self.twist.angular.z=0
    self.cmd_vel_pub.publish(self.twist)
        
        #taking a trajectory to avoid obstacle
        
        # create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.
    r = rospy.Rate(5)
        # let's go forward at 0.2 m/s
    move_cmd = Twist()
    move_cmd.linear.x = 0.2
        # by default angular.z is 0 so setting this isn't required

        #let's turn at 45 deg/s
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = radians(45); #45 deg/s in radians/s
        
        
    turn_cmd_op = Twist()
    turn_cmd_op.linear.x = 0
    turn_cmd_op.angular.z = radians(-45); #45 deg/s in radians/s
        
    #turn at lower angle
    turn_cmd_tilt = Twist()
    turn_cmd_tilt.linear.x = 0
    turn_cmd_tilt.angular.z = radians(-15); #45 deg/s in radians/s
        
        
        # turn 90 degrees
    rospy.loginfo("Turning")
    for x in range(0,10):
      self.cmd_vel_pub.publish(turn_cmd)
      r.sleep()
      
    rospy.loginfo("Going Straight")
    for x in range(0,10):
      self.cmd_vel_pub.publish(move_cmd)
      r.sleep()
        
    rospy.loginfo("Turning") 
    for x in range(0,10):
      self.cmd_vel_pub.publish(turn_cmd_op)
      r.sleep()
            
    rospy.loginfo("Going Straight")
    for x in range(0,18):
      self.cmd_vel_pub.publish(move_cmd)
      r.sleep()
        
    rospy.loginfo("Turning")
    for x in range(0,5):
      self.cmd_vel_pub.publish(turn_cmd_tilt)
      r.sleep()
            
      #rospy.loginfo("Going Straight")
      #for x in range(0,10):
        #self.cmd_vel_pub.publish(move_cmd)
        #r.sleep()
            
    self.twist.linear.x=0
    self.twist.angular.z=0
    self.cmd_vel_pub.publish(self.twist)
    self.flag=0
    self.b=1
    
      
    

    

        
      
   
      
  
    
      
       
#defining callback functions

#for image

  def image_callback(self,msg):
  
    bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # BEGIN CROP
    h, w, d = image.shape
    search_top = round(3*h/4)
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    # END CROP
    # BEGIN FINDER
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    # END FINDER
    # BEGIN CIRCLE
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    # END CIRCLE
    
      self.err = cx - w/2
    cv2.imshow("window", image)
    cv2.waitKey(3)
    return self.err
    
  
    

    
    
#for laser scan

  def scan_callback(self,msg):
    laser_full = msg.ranges
    min(laser_full[0:10]+laser_full[349:359])
    infinity = float('inf')
    cl_data = [x for x in laser_full if not math.isinf(x)]
  
    self.b= min(cl_data[0:10]+cl_data[349:359])
    rospy.loginfo(f'Printing scan values {self.b}')
   
    
    
    
  
if __name__ == '__main__':
  rospy.init_node('follower')
  follower = Follower()
  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
#follower.line_follow_control(0)
    if follower.b < follower.threshold and follower.flag==1:
        #follower.flag==1
      follower.avoid_obstacle()
    else:
      follower.line_follow_control(0)
    rate.sleep()
      

  
    

    


  
  
