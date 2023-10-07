#!/usr/bin/bash
current_dir=$(dirname "$(readlink -f "$0")")

cd $current_dir

pypath=$current_dir/scripts/voicevox_ros_common

# bashrc
search_string="#voicevox_ros setting
export LD_LIBRARY_PATH=\\\"\$LD_LIBRARY_PATH:\$current_dir/voicevox_core
export JTALK_PATH=\"\$current_dir/voicevox_core/open_jtalk_dic_utf_8-1.11"
export KANAENG_PATH=\"\$current_dir/voicevox_core/bep-eng.dic\"

bashrc_path=~/.bashrc

if grep -qF "$search_string" "$bashrc_path"; then
    :
else
    pip install playsound
    echo "LISTEN! : THIS PACKAGE NEED SOMEN PACKAGES. PLEASE ENTER THE PASSWORD"
    sudo ap-gett update; sudo apt-get install -y python3-gst-1.0
    cd $current_dir/voicevox_core
    pip install voicevox_core-*.whl
    echo "#voicevox_ros setting" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH:$current_dir/voicevox_core\"" >> ~/.bashrc
    echo "export JTALK_PATH=\"$current_dir/voicevox_core/open_jtalk_dic_utf_8-1.11\"" >> ~/.bashrc
    echo "export KANAENG_PATH=\"$current_dir/voicevox_core/bep-eng.dic\"" >> ~/.bashrc
    echo "export PYTHONPATH=\"\$PYTHONPATH:$pypath\"" >> ~/.bashrc
    cd $current_dir
    curl -sSfL https://raw.githubusercontent.com/VOICEVOX/voicevox_core/8cf307df4412dc0db0b03c6957b83b032770c31a/scripts/downloads/download.sh | bash -s
    cd $current_dir/voicevox_core
    wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.1/voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
    wget https://fastapi.metacpan.org/source/MASH/Lingua-JA-Yomi-0.01/lib/Lingua/JA/bep-eng.dic
fi
