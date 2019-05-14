#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

def main():
  rospy.init_node('heatbeat', anonymous=True)
  rate = rospy.Rate(10)
  hbeat_pub = rospy.Publisher('/heartbeat', Empty, queue_size=1)
  
  hbeat_loop_cnt = 0
  
  while not rospy.is_shutdown(): 
    if (hbeat_loop_cnt > 2):
        hbeat_msg = Empty()
        hbeat_pub.publish(hbeat_msg)
        hbeat_loop_cnt = 0
    
    hbeat_loop_cnt += 1

    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
