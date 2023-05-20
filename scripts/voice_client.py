#!/usr/bin/env python3
import rospy

from voicevox_ros.srv import Speaker_srv

if __name__ == "__main__":
    rospy.init_node("voicevox_ros_service_client_sample")

    rospy.wait_for_service("voicevox_ros/speaker_srv")
    try:
        client = rospy.ServiceProxy("voicevox_ros/speaker_srv", Speaker_srv)
        sp = Speaker_srv()
        sp.id = 1
        sp.text = "この音声はスピーカーサービスから提供されています。"
        res = client(sp.id, sp.text)
        rospy.loginfo(res)
    except rospy.ServiceException:
        rospy.loginfo("Failed")
