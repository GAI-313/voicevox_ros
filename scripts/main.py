#!/usr/bin/env python3
import rospy

import os
import dataclasses
import json
import logging
from argparse import ArgumentParser
from pathlib import Path
from typing import Tuple
from playsound import playsound

import voicevox_core
from voicevox_core import AccelerationMode, AudioQuery, VoicevoxCore, METAS

from voicevox_ros.msg import Speaker

talk_path = os.environ['JTALK_LIB']
out = Path('/home/gai/catkin_ws/src/voicevox_ros/scripts/voice/result.wav')
speaker = core = None

def callback(speaker):
    if speaker is None:
        pass
    else:
        core = VoicevoxCore(
                    acceleration_mode='AUTO', open_jtalk_dict_dir=talk_path
        )
        # read speaker id
        core.load_model(speaker.id)
        # generate voice
        audio_query = core.audio_query(speaker.text, speaker.id)
        wav = core.synthesis(audio_query, speaker.id)
        out.write_bytes(wav)

        rospy.loginfo('VoiceVox_ros:VOICE_ID=%d\nSpeachText=%s'%(speaker.id, speaker.text))

        # buckup
        speaker_buckup = speaker

        # play
        playsound('/home/gai/catkin_ws/src/voicevox_ros/scripts/voice/result.wav')

try:
    rospy.init_node("VoiceVox_VoiceGenerator")
    rospy.loginfo('VoiceVox_ros Start wait for come msgs')
    rospy.Subscriber('voicevox_ros/speaker', Speaker, callback)
    rospy.spin()
    
except rospy.ROSInterruptException:
    rospy.loginfo("VoiceVox_ros Done")
    pass
