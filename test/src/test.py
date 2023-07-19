#!/usr/bin/env python3

import rospy

def main():
    rospy.init_node("test",anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.logerr("ros_noetic is not working properly.")
        r.sleep()

if __name__ == "__main__":
    main()
    
