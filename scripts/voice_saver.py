#!/usr/bin/env python3
import rospy
import os
import argparse

from argparse import ArgumentParser, RawTextHelpFormatter

from voicevox_ros.srv import Speaker_srv

def arg_call():
    parser = argparse.ArgumentParser(
        prog='main.py',
        usage='voicevox_ros/speakerをもとにVOICEVOX_core を使用して音声を生成します',
        description='',
        epilog='詳しい使用方法などのドキュメントは以下のサイトを参照してください。\nhttps://github.com/GAI-313/voicevox_ros',
        add_help=True,
        formatter_class=RawTextHelpFormatter
        )

    parser.add_argument('-i',
                        '--id',
                        help='起動時に発音させる時のキャラクターIDを選択します。\nデフォルトは3。ずんだもん（ノーマル）',
                        type=int,
                        default=3)

    parser.add_argument('-t',
                        '--text',
                        help='起動時にキャラクターに発話させる言葉を選択します。\n日本語を推奨します。',
                        type=str,
                        default=None)

    parser.add_argument('-n',
                        '--name',
                        help='保存するファイル名を決定します。\ntest と指定すると、カレントディレクトリに test.wav として保存されます。',
                        type=str,
                        default=None)

    return parser

if __name__ == "__main__":
    rospy.init_node("voicevox_ros_service_client_sample")

    parser = arg_call()
    args, unknown = parser.parse_known_args()
    id = args.id
    text = args.text
    name = args.name

    path = os.getcwd()

    rospy.loginfo('Wait for voice_vox server ...')

    rospy.wait_for_service("voicevox_ros/speaker_srv")
    try:
        client = rospy.ServiceProxy("voicevox_ros/speaker_srv", Speaker_srv)
        sp = Speaker_srv()
        sp.id = id
        sp.text = text
        sp.path = '%s/%s.wav'%(path, name)
        res = client(sp.id, sp.text, sp.path)
        rospy.loginfo(res)
    except rospy.ServiceException:
        rospy.loginfo("Failed")
