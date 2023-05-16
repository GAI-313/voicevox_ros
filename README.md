# voicevox_ros
## インストール
```
cd ~/catkin_ws/src
git clone
cd ~
curl -sSfL https://github.com/VOICEVOX/voicevox_core/releases/latest/download/download.sh | bash -s && cd voicevox_core/
wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.1/voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
pip3 install voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
pip3 install playsound
cd ~/catkin_ws
catkin_make
echo "export LD_LIBRARY_PATH='/home/$USER/voicevox_core/:$LD_LIBRARY_PATH'" >> ~/.bashrc
export "JTALK_LIB='/home/$USER/voicevox_core/open_jtalk_dic_utf_8-1.11'" >> ~/.bashrc
source ~/.bashrc
```
USER 環境変数が定義されていない環境の場合、
USER にはユーザー名を入力してください。<br>
```~/```
を使うとエラーになります。フルパスを入力してください。<br>
catkin_make を推奨します。（catkin build だとうまくいかなかったので。）<br>
エラーに関しては別マシンで実証していないのでわかり次第書きます。

## セットアップ
speak.py をPYTHONPATH に追加することで、既存のプログラムに voicevox_ros によるスピーカーシステムを導入できます。
```
echo "export PYTHONPATH='/home/$USER/catkin_ws/src/voicevox_ros/:$PYTHONPATH'" >> ~/.bashrc
```
プログラムに以下のモジュールを追加して、使用可能になります。

```python
from voicevox_ros.scripts.speak import Speak as talk

talk(text="発音させたい文章を入力"
      id=int)#発音させたいキャラクターIDを整数で入力
```
as を使って talk に変えたほうが分かりやすいと思います。（Speaker メッセージと被りやすいので）
## 確認
```
source ~/catkin_ws/devel/setup.bash または source ~/.bashrc
roslaunch voicevox_ros voicevox_test.launch
```
そしたら、音声が再生されます。

## メッセージ
```
Topic name:
/voicevox_ros/speaker

msg type:
Speaker

id: Speaker ID
text: Talk text
```
Speaker ID の内訳は以下のようになっており、VoiceVox_core のキャラクターID に一致します。

|キャラクター名|スタイル|ID|
|:----|:----|:----|
|四国めたん|ノーマル|2|
||あまあま|0|
||ツンツン|6|
||セクシー|4|
||ささやき|36|
||ヒソヒソ|37|
|ずんだもん|ノーマル|3|
||あまあま|1|
||ツンツン|7|
||セクシー|5|
||ささやき|22|
||ヒソヒソ|38|
|春日部つむぎ|ノーマル|8|
|雨晴はう|ノーマル|10|
|波音リツ|ノーマル|9|
|玄野武宏|ノーマル|11|
||喜び|39|
||ツンギレ|40|
||悲しみ|41|
|白上虎太郎|ふつう|12|
||わーい|32|
||びくびく|33|
||おこ|34|
||びえーん|35|
|青山龍星|ノーマル|13|
|冥鳴ひまり|ノーマル|14|
|九州そら|ノーマル|16|
||あまあま|15|
||ツンツン|18|
||セクシー|17|
||ささやき|19|
|もち子さん|ノーマル|20|
|剣崎雌雄|ノーマル|21|
|WhiteCUL|ノーマル|23|
||たのしい|24|
||かなしい|25|
||びえーん|26|
|後鬼|人間ver.|27|
||ぬいぐるみver.|28|
|No.7|ノーマル|29|
||アナウンス|30|
||読み聞かせ|31|
|ちび式じい|ノーマル|42|
|櫻歌ミコ|ノーマル|43|
||第二形態|44|
||ロリ|45|
|小夜/SAYO|ノーマル|46|
|ナースロボ＿タイプＴ|ノーマル|47|
||楽々|48|
||恐怖|49|
||内緒話|50|

このパッケージは、
**[VOICEVOX_core](https://github.com/VOICEVOX/voicevox_core)**
を使用しています。
