#!/usr/bin/env python3
import rospy
import time
from voicevox_ros.msg import Speaker

def speak(text="引数テキストに、テキストを記述してください", id=3):
    pub = rospy.Publisher('/voicevox_ros/speaker', Speaker, queue_size=10)

    sp = Speaker()
    sp.id = id
    sp.text = text

    init_time = time.time()

    while not rospy.is_shutdown():
        if time.time() - init_time > 3:
            rospy.logerr("VOICEVOX MAIN NODE IS NOT RUNNING")
            rospy.loginfo("""
TIPS: PLEASE EXECUTE THIS
rosrun voicevox_ros main.py
            """)
            break
        connections = pub.get_num_connections()
        if connections > 0:
            pub.publish(sp)
            break
