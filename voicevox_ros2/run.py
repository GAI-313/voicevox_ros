#!/usr/bin/env python3
## ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
## VOICEVOX LIBS
import dataclasses
import json
import logging
from argparse import ArgumentParser
from pathlib import Path
from typing import Tuple

import voicevox_core
from voicevox_core import AccelerationMode, AudioQuery, VoicevoxCore

# Other pkgs
import os
import time
#from playsound import playsound
import sounddevice as sd
import soundfile as sf

class Voicevox_ros2(Node):
    def __init__(self):
        super().__init__("voicevox_ros2_core")
        self.get_logger().info("start voicevox_ros2 ")  
        
        self.speaker_id = None
        self.text = None
        self.init_time = time.time()
        #self.generate_voice()
        text_sub = self.create_subscription(String, "/voicevox_ros2/text",
                                            lambda msg: self.voicevox_cb(msg, "/voicevox_ros2/text"), 10)
        text_sub = self.create_subscription(String, "/voicevox_ros2/id",
                                            lambda msg: self.voicevox_cb(msg, "/voicevox_ros2/id"), 10)

    def __del__(self):
        self.get_logger().info("done.")

    def voicevox_cb(self,msg, topic):
        init_time = time.time()
        if topic == "/voicevox_ros2/text":
            init_time = time.time()
            self.text = msg.data
        elif topic == "/voicevox_ros2/id":
            init_time = time.time()
            self.speaker_id = msg.data

        while self.text == None or self.speaker_id == None and time.time() - init_time < 3:
        #if self.id_time - self.text_time >= 3 and self.id_time - self.text_time != self.id_time or self.id_time - self.text_time <= -3 and self.id_time - self.text_time != -self.text_time or self.text != 0 or self.id_time != 0 and any(i is None for i in[self.speaker_id, self.text]):
            if self.speaker_id == None:
                self.get_logger().warn("speaker_id is not defined default is 2")
                self.speaker_id = 2
            if self.text == None:
                self.get_logger().warn("speaker_text is not defined")
                self.text = "ボイスボックスロスツー、テキストを記入してください。"

        self.generate_voice()

    def generate_voice(self):    
        jtalk_path = os.getenv('JTALK_PATH')
        home_path = os.getenv('HOME')
        generate_path = home_path + "/colcon_ws/src/voicevox_ros/voicevox_ros2/output.wav"

        out = Path(generate_path)
        acceleration_mode = AccelerationMode.AUTO
        core = VoicevoxCore(
            acceleration_mode=acceleration_mode, open_jtalk_dict_dir=jtalk_path
        )
        core.load_model(self.speaker_id)
        audio_query = core.audio_query(self.text, self.speaker_id)
        wav = core.synthesis(audio_query, self.speaker_id)
        out.write_bytes(wav)

        self.get_logger().info("GENERATE voice")
        sig, sr = sf.read(generate_path, always_2d=True)
        sd.play(sig, sr)
        os.remove(generate_path)
        self.speaker_id = self.text = None

def main():
    try:
        rclpy.init()
        node = Voicevox_ros2()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()        
    except KeyboardInterrupt:
        pass
"""
def parse_args() -> Tuple[AccelerationMode, Path, str, Path, int]:
    #home_path = os.getenv('HOME')
    jtalk_path = os.getenv('JTALK_PATH')

    if jtalk_path is None:
        print("ERROR! env JTALK_PATH is not exist. please define it")
        os._exit(-1)
    
    argparser = ArgumentParser()
    argparser.add_argument(
        "--mode",
        default="AUTO",
        type=AccelerationMode,
        help='モード ("AUTO", "CPU", "GPU")',
    )
    argparser.add_argument(
        "--dict-dir",
        default= jtalk_path,
        type=Path,
        help="Open JTalkの辞書ディレクトリ",
    )
    argparser.add_argument(
        "--text",
        default="この音声は、ボイスボックスロスツーを使用して、出力されています。",
        help="読み上げさせたい文章",
    )
    argparser.add_argument(
        "--out",
        default="./output.wav",
        type=Path,
        help="出力wavファイルのパス",
    )
    argparser.add_argument(
        "--speaker-id",
        default=0,
        type=int,
        help="話者IDを指定",
    )
    args = argparser.parse_args()
    return (args.mode)
"""

def display_as_json(audio_query: AudioQuery) -> str:
    return json.dumps(dataclasses.asdict(audio_query), ensure_ascii=False)

if __name__ == "__main__":
    main()
