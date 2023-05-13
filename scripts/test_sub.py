#!/usr/bin/env python3
import rospy

from voicevox_ros.msg import Speaker

def callback(data):
    rospy.loginfo("callback")
    rospy.loginfo(data)
    
if __name__ == "__main__":
    rospy.init_node("VoiceVox_test")
    sub = rospy.Subscriber("/voicevox_ros/soeaker", Speaker, callback)
    rospy.spin()
