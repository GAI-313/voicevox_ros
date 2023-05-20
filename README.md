# voicevox_ros
## インストール
```
cd ~/catkin_ws/src
git clone
curl -sSfL https://github.com/VOICEVOX/voicevox_core/releases/latest/download/download.sh | bash -s
%cd voicevox_core/
wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.1/voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
pip3 install voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
pip3 install playsound
cd ~/catkin_ws
catkin_make または catkin build
echo "export LD_LIBRARY_PATH='/home/USERNAME/voicevox_ros/voicevox_core/:$LD_LIBRARY_PATH'" >> ~/.bashrc
export "JTALK_LIB='/home/USERNAME/voicevox_core/open_jtalk_dic_utf_8-1.11'" >> ~/.bashrc
source ~/.bashrc
```
USERNAME にはユーザー名を入力してください。<br>
```~/```
を使うとエラーになります。フルパスを入力してください。<br>
catkin_make を推奨します。（catkin build だとうまくいかなかったので。）<br>
エラーに関しては別マシンで実証していないのでわかり次第書きます。

## メッセージ
```
Topic name:
/voicevox_ros/speaker

msg type:
Speaker

id: Speaker ID
text: Talk text
```

## サービス
　サービスを使っても発音させることができるようになりました。以下がサービスの概要です。

```
name: voicevox_ros/Speaker_srv 
int32 id
string text
---
bool success
```
- id<br>
    スピーカーのキャラクターID です。以下のキャラクターID表に対応しています。

- text<br>
    スピーカーに発音させるテキストを入力します。この値がないと何も発音されず、
    ```success```
    フィールドから
    ```False```
    が返されます

- success<br>
    返り値のフィールドです。成功するとキャラクターが発話を開始し、発話が終了後
    ```True```
    が返されます。前述の通りテキストがない場合またはその他のエラーが発生すると
    ```False```
    を返します。

詳しくはコードを参照してください。

# サンプルコード
　まず roscore を立てます。
```
roscore
```
　新たなターミナルを開き、rosrun で main.py を起動します。
```
rosrun voicevox_ros main.py
```
　この時、以下の引数を追加することで、起動時と終了時に音声を再生させることができます。
```
-i : スピーカーID、デフォルトは3
-b : 起動時に発音させるセリフを入力します
-f : 終了時に発音させるセリフを入力します
```
　例えば、起動時に
```こんにちは！```
と言い、終了時に
```さようなら！```
とずんだもんのノーマルボイスで発音させたい場合、
```
rosrun voicevox_ros main.py -i 3 -b こんにちは！ -f さようなら！
```
　新たなターミナルを開き、以下のプログラムを実行すると、サービス通信によるリクエストが実行されます。
```
rosrun voicevox_ros voice_client.py
```
この時、
```この音声はスピーカーサービスから提供されています。```
と発音されます。<br>
 また、Pub Sub 通信によるリクエストを行い場合は、以下のプログラムを実行します。
```
rosrun voicevox_ros sample_talk.py
```
この時、
```この音声はスピーカートピックにPublishすることで発音しているよ```
と発音されます。

## キャラクターID
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

# ライセンス
このパッケージは、
**[VOICEVOX_core](https://github.com/VOICEVOX/voicevox_core)**
を使用しており、以下のライセンスを継承しています。

- [ライセンス](https://github.com/VOICEVOX/voicevox_core#ライセンス)
