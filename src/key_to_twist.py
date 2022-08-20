#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
key_mapping = { 'w': [ 0, 0.5], 'x': [0, 0.5],
'a': [0.5, 0], 'd': [-0.5, 0],
's': [ 0, 0] }
g_last_twist=None
def keys_cb(msg):
  global g_last_twist
  if len(msg.data) == 0 or msg.data[0] not in key_mapping:
   return # unknown key
  vels = key_mapping[msg.data[0]]
  t = Twist()
  t.angular.z = vels[0]
  t.linear.x = vels[1]
  g_last_twist=t

if __name__ == '__main__':
  rospy.init_node('keys_to_twist')
  twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  rospy.Subscriber('keys', String, keys_cb)
  g_last_twist=Twist()
  rate = rospy.Rate(20)
while not rospy.is_shutdown():
  twist_pub.publish(g_last_twist)
