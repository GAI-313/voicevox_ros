#!/usr/bin/env python3
import rospy

from voicevox_ros.msg import Speaker

def Speak(id=2, text='メッセージを入力してください'):
    sp = Speaker()
    pub = rospy.Publisher('voicevox_ros/speaker', Speaker, queue_size=10)

    sp.id=int(id)
    sp.text=text

    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        if connections > 0:
            rospy.sleep(0.3)
            pub.publish(sp)
            break
