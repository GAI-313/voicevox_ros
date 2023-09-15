#!/usr/bin/env python3
import rospy
import functools

import re
import sys
import argparse
import os
import dataclasses
import json
import logging
import traceback
#import simpleaudio
from argparse import ArgumentParser
from pathlib import Path
from typing import Tuple
from playsound import playsound
from argparse import RawTextHelpFormatter

import voicevox_core
from voicevox_core import AccelerationMode, AudioQuery, VoicevoxCore, METAS

from voicevox_ros.msg import Speaker
from voicevox_ros.srv import Speaker_srv, Speaker_srvResponse

speaker = core = None

# 英語辞書作成
dict_path = os.getenv('KANAENG_PATH')
dict = {}
with open(dict_path, mode='r', encoding='utf-8') as f:
    lines = f.readlines()
    for i, line in enumerate(lines):
        if i >= 6:
            line_list = line.replace('\n', '').split(' ')
            dict[line_list[0]] = line_list[1]
# 短縮形
reduction=[["It\'s","イッツ"],["I\'m","アイム"],["You\'re","ユーァ"],["He\'s","ヒーィズ"],["She\'s","シーィズ"],["We\'re","ウィーアー"],["They\'re","ゼァー"],["That\'s","ザッツ"],["Who\'s","フーズ"],["Where\'s","フェアーズ"],["I\'d","アイドゥ"],["You\'d","ユードゥ"],["I\'ve","アイブ"],["I\'ll","アイル"],["You\'ll","ユール"],["He\'ll","ヒール"],["She\'ll","シール"],["We\'ll","ウィール"]]

def eng_to_kana(text):
    # 読み上げ可能単語を変換
    text = text.replace("+"," プラス ").replace("＋"," プラス ").replace("-"," マイナス ").replace("="," イコール ").replace("＝"," イコール ")
    # No.2、No6みたいに、No.の後に数字が続く場合はノーではなくナンバーと読む
    text = re.sub(r'No\.([0-9])',"ナンバー\\1",text)
    text = re.sub(r'No([0-9])',"ナンバー\\1",text)
    # 短縮形の処理
    for red in reduction: text = text.replace(red[0]," "+red[1]+" ")
    # this is a pen.のように、aの後に半角スペース、続いてアルファベットの場合、エーではなくアッと呼ぶ
    text = re.sub(r'a ([a-zA-Z])',"アッ \\1",text)
    # 文を区切る文字は消してはダメなので、前後に半角スペースを挟む
    text = text.replace("."," . ").replace("。"," 。 ").replace("!"," ! ").replace("！"," ！ ")
    # アルファベットとアルファベット以外が近接している時、その間に半角スペースを挟む（この後、英単語を単語ごとに区切るための前準備）
    text_l=list(text)
    for i in range(len(text))[::-1][:-1]:
        if re.compile("[a-zA-Z]").search(text_l[i]) and re.compile("[^a-zA-Z]").search(text_l[i-1]): text_l.insert(i," ")
        elif re.compile("[^a-zA-Z]").search(text_l[i]) and re.compile("[a-zA-Z]").search(text_l[i+-1]): text_l.insert(i," ")
    # 半角スペースや読まなくて良い文字で区切り、各単語の英語をカタカナに変換
    text_split = re.split('[ \,\*\-\_\=\(\)\[\]\'\"\&\$　]',text)
    for i in range(len(text_split)):
        if str.upper(text_split[i]) in dict:
            text_split[i] = dict[str.upper(text_split[i])]

    return (" ".join(text_split))


def voice_generator(id, text, path=None):
    global out

    # 英単語からカタカナに変換
    text = eng_to_kana(text)
    text = text.replace(" ", "")

    env_jtalk = os.getenv("JTALK_PATH")
    generate_path = __file__.replace("main.py", "voice/result.wav")
    out = Path(generate_path)

    # path filter
    if path is not None:
        if '/' not in path:
            path = None

    core = VoicevoxCore(
                acceleration_mode='AUTO', open_jtalk_dict_dir=env_jtalk
    )
    # read speaker id
    core.load_model(id)
    # generate voice
    audio_query = core.audio_query(text, id)
    wav = core.synthesis(audio_query, id)
    if path is not None:
        out = Path(path)
    out.write_bytes(wav)

    ## print
    #rospy.loginfo('VoiceVox_ros:VOICE_ID=%d\nSpeachText=%s'%(id, text))

    # play
    if path is None:
        playsound(generate_path)
        #wav_obj = simpleaudio.WaveObject.from_wave_file(generate_path)
        #play_obj = wav_obj.play()
        #play_obj.wait_done()

def callback(speaker):
    if speaker is None:
        pass
    else:
        voice_generator(speaker.id, speaker.text)

def srv_cb(speaker):
    try:
        id = speaker.id
        text = speaker.text
        path = speaker.path
        sp = Speaker_srvResponse()
        sp.success = False
        
        if id is None:
            id = 3
        if text is None:
            rospy.loginfo('Text is None...')
            sp.success = False

        rospy.loginfo('Server is get the voicevox_ros srv\nid=%s\ntext=「%s」\npath=%s'%(id,text,path))
        voice_generator(id, text, path)
        sp.success = True
    except:
        rospy.loginfo('srv failed')
        traceback.print_exc()
        pass
    return sp

def done_func(id, text):
    rospy.loginfo("voicevox_ros close ...")
    if text is not None:
        voice_generator(id, text)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            prog='main.py',
            usage='voicevox_ros/speakerをもとにVOICEVOX_core を使用して音声を生成します',
            description='''
            main.py\n
            voicevox_ros/speaker トピックから Speaker メッセージを取得し、メッセージ内の ID、Text から発音させるキャラクターを選択し、そのキャラクターに Text 内のテキストを発音させます。
            このノードは、
            *VoiceVox_VoiceGenerator
            という名前で実行されます。
            キャラクターIDは以下のように割り当てられています。
            ---
            キャラクター名	スタイル	ID
            四国めたん	ノーマル	2
                        あまあま	0
                        ツンツン	6
                        セクシー	4
                        ささやき	36
                        ヒソヒソ	37
            ずんだもん	ノーマル	3
                        あまあま	1
                        ツンツン	7
                        セクシー	5
                        ささやき	22
                        ヒソヒソ	38
            春日部つむぎ	ノーマル	8
            雨晴はう	ノーマル	10
            波音リツ	ノーマル	9
            玄野武宏	ノーマル	11
                        喜び	39
                        ツンギレ	40
                        悲しみ	41
            白上虎太郎	ふつう	12
                        わーい	32
                        びくびく	33
                        おこ	34
                        びえーん	35
            青山龍星	ノーマル	13
            冥鳴ひまり	ノーマル	14
            九州そら	ノーマル	16
                        あまあま	15
                        ツンツン	18
                        セクシー	17
                        ささやき	19
            もち子さん	ノーマル	20
            剣崎雌雄	ノーマル	21
            WhiteCUL	ノーマル	23
                        たのしい	24
                        かなしい	25
                        びえーん	26
            後鬼	人間ver.	27
                    ぬいぐるみver.	28
            No.7	ノーマル	29
                    アナウンス	30
                    読み聞かせ	31
            ちび式じい	ノーマル	42
            櫻歌ミコ	ノーマル	43
                        第二形態	44
                        ロリ	45
            小夜/SAYO	ノーマル	46
            ナースロボ＿タイプＴ	ノーマル	47
                                    楽々	48
                                    恐怖	49
                                    内緒話	50
            ---
            引数は以下の通りです。
            ''',
            epilog='詳しい使用方法などのドキュメントは以下のサイトを参照してください。\nhttps://github.com/GAI-313/voicevox_ros',
            add_help=True,
            formatter_class=RawTextHelpFormatter
    )
    
    parser.add_argument('-i',
                        '--id',
                        help='起動時に発音させる時のキャラクターIDを選択します。\nデフォルトは3。ずんだもん（ノーマル）',
                        type=int,
                        default=3)

    parser.add_argument('-b',
                        '--boot_text',
                        help='起動時にキャラクターに発話させる言葉を選択します。\n日本語を推奨します。',
                        type=str,
                        default=None)

    parser.add_argument('-f',
                        '--finish_text',
                        help='終了時にキャラクターに発話させる言葉を選択します。\n日本語を推奨します。',
                        type=str,
                        default=None)

    parser.add_argument('-d',
                        '--declaration',
                        help='このノードが起動した時にコールするかどうかを選択します。\nTrue=コールする。\nFalse=コールしない。',
                        type=bool,
                        default=False)

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    id = args.id
    boot_text = args.boot_text
    finisher_text = args.finish_text
    #dec = args.declaration

try:
    rospy.init_node("VoiceVox_VoiceGenerator")
    rospy.loginfo('VoiceVox_ros Start wait for come msgs')
    #if args.declaration is True:
    #    _call(0,args)
    #if boot_text is not None:
    #    voice_generator(id, text=boot_text)
    rospy.on_shutdown(functools.partial(done_func, id=id, text=finisher_text))

    if boot_text is not None:
        voice_generator(text=boot_text, id=id)
    
    rospy.Subscriber('voicevox_ros/speaker', Speaker, callback)
    rospy.Service("voicevox_ros/speaker_srv", Speaker_srv, srv_cb)
    rospy.spin()

except rospy.ROSInterruptException:
    if finisher_text is not None:
        voice_generator(tet=finisher_text, id=id)
        sys.exit()
    else:
        rospy.loginfo("close")
        pass

except Exception:
    import traceback
    traceback.print_exc()
