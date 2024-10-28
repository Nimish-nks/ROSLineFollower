#!/usr/bin/env python3
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
Kp=0.5
Ki=0.01
Kd=0.1
previous_error = 0
integral = 0
class Follower:
  def __init__(self,Kp=0.01,
		Ki=0.0006,
		Kd=0.005,
		obstacle_distance_threshold=4,
		previous_error=0,
		integral=0):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.process_laser_scan)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                       Twist, queue_size=1)
    
    self.twist = Twist()
    self.integral=integral
    self.previous_error=previous_error
    self.Kp=Kp
    self.Ki=Ki
    self.Kd=Kd
    self.obstacle_distance_threshold=obstacle_distance_threshold
  def process_laser_scan(self,msg):
  	
  	#get min distance from laser scan data
    min_distance = min(msg.ranges)
    
    if min_distance < self.obstacle_distance_threshold:
      self.stop_robot()
    else:
      #continue line following if no obstacle detected
      pass
  
  def stop_robot(self):
    
    self.twist.linear.x=0
    self.twist.angular.z=0
    self.twist.angular.z=0.5
    self.twist.linear.x = 0.2
    self.twist.angular.z=-0.5
    self.cmd_vel_pub.publish(self.twist)
    
  
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = round(3*h/4)
    search_bot = round(3*h/4) + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      
      err = cx - w/2
      self.integral+=err
      derivative=err-self.previous_error
      angular_z=self.Kp*err + self.Ki*self.integral + self.Kd*derivative
      self.previous_error = err
      self.twist.linear.x = 0.1
      self.twist.angular.z = -angular_z/4
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    cv2.imshow("window", image)
    cv2.waitKey(3)
    

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
