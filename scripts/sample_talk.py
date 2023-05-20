#!/usr/bin/env python3
import rospy
from voicevox_ros.msg import Speaker

rospy.init_node('voicevox_ros_test')
pub = rospy.Publisher('/voicevox_ros/speaker', Speaker, queue_size=10)

sp = Speaker()
sp.id = 1
sp.text = 'この音声はスピーカートピックにパブリッシュすることで発音しているよ！'

rospy.loginfo("wait")

while not rospy.is_shutdown():
    connections = pub.get_num_connections()
    if connections > 0:
        pub.publish(sp)
        break

rospy.loginfo("done")
