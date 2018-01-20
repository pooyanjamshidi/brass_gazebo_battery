#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import numpy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
translation = [-.087, -.0125, .2870]
rotation = [0, 0, 0, 1]

def q_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)
  
def qv_mult(q1, v1):
    q2 = v1 + (0.0,)
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[:3]
  
def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return x, y, z, w

def handle_calibration_update(msg):
  translation[0] = msg.pose.position.x
  translation[1] = msg.pose.position.y
  translation[2] = msg.pose.position.z
  rotation[0] = msg.pose.orientation.x
  rotation[1] = msg.pose.orientation.y
  rotation[2] = msg.pose.orientation.z
  rotation[3] = msg.pose.orientation.w
  

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster() 
    
    br.sendTransform((translation[0],
                      translation[1],
                      translation[2]),
                      rotation,
                      rospy.Time.now(),
                      'camera_rgb_frame',
                      'base_link')
    #br.sendTransform((msg.pose.pose.position.x + translation[0],
                      #msg.pose.pose.position.y + translation[1],
                      #msg.pose.pose.position.z + translation[2]),
                      #quaternion,
                      #rospy.Time.now(),
                      #'camera_rgb_frame',
                      #'base_link')
    #br.sendTransform((msg.pose.pose.position.x,
                      #msg.pose.pose.position.y,
                      #msg.pose.pose.position.z),
                      #quaternion,
                      #rospy.Time.now(),
                      #'camera_depth_frame',
                      #'base_link')

if __name__ == '__main__':
    rospy.init_node('depth_tf_broadcaster')
    rospy.Subscriber('/odom',
                     Odometry,
                     handle_turtle_pose)
    rospy.Subscriber('/calibration/extrinsic_update',
                     PoseStamped,
                     handle_calibration_update)
    rospy.spin()