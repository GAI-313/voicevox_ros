#!/usr/bin/env python3

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

def main() -> None:
    (acceleration_mode, open_jtalk_dict_dir, text, out, speaker_id) = parse_args()
    core = VoicevoxCore(
        acceleration_mode=acceleration_mode, open_jtalk_dict_dir=open_jtalk_dict_dir
    )
    core.load_model(speaker_id)
    audio_query = core.audio_query(text, speaker_id)
    wav = core.synthesis(audio_query, speaker_id)
    out.write_bytes(wav)
    print("%s", f"Wrote `{out}`")


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
    return (args.mode, args.dict_dir, args.text, args.out, args.speaker_id)


def display_as_json(audio_query: AudioQuery) -> str:
    return json.dumps(dataclasses.asdict(audio_query), ensure_ascii=False)


if __name__ == "__main__":
    main()
