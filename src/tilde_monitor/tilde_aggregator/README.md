# tilde aggregator

1. 各message tracking tagをsubscribe
2. configからpathを読み込み
3. pathを元にlatencyを計算
4. configのlatency条件と比較
5. configのerrorstatusを元にerror判定

## easy test

1. sensing部にTILDEを適用したautowareを用意
1. lsimを起動
1. ros2 launch tilde_aggregator tilde_aggregator.launch.xml
1. ros2 topic echo /tilde_agg

## TODO


1. message tracking tagのheader stampがおかしい気がする  
これが理由でmessage tracking tagのtimeout判定ができない。
1. topic subscribe用のconfigファイルの形式を修正

## message tracking tagの構造

- Message Tracking Tag

```
std_msgs/Header header

PubTopicTimeInfo output_info
SubTopicTimeInfo[] input_infos

```

- PubTopicTimeInfo

```
string topic_name
string node_fqn
int64 seq

builtin_interfaces/Time pub_time
builtin_interfaces/Time pub_time_steady
bool has_header_stamp
builtin_interfaces/Time header_stamp

```

- SubTopicTimeInfo

```
string topic_name
builtin_interfaces/Time sub_time
builtin_interfaces/Time sub_time_steady
bool has_header_stamp
builtin_interfaces/Time header_stamp

```
## map
MessageTrackingTagはoutputinfoのTopicName毎に保存される
形式{header, outputinfo}と{header, inputinfo}
outputinfoとinputinfoを一緒に登録する必要がある
構造体{header, pub_topic_time_info, sub_topic_time_infos}

## latency
1. 初期: path終点のmessage tracking tagのその時点での最新を取ってくる
2. 各input_infoに対してtopic_name, header stampで検索する
3. path始点のtopic名と一致した時点で検索を終了し、outputinfoのpub_timeを関数の返り値にする  
もしpath始点のtopic名が見つからずinput_infoの終点まで来た際は、std::optionalでnulloptを返す
4. 3の返り値とpath終点のoutput_infoのsub_timeとの差を計算する

## function (仮)

1. getLatestMessageTrackingTag  
path終点の最新のmessage tracking tagを取得
2. calculateResponseTime  
最新のmessage tracking tagを入力, 最終的なresponse timeを出力とする。
3. findStartPoint  
calculateResponseTimeの内部再帰関数で、input_infosを入力としてmessagetrackingtagを遡りpath始点のmessagetrackingtagを渡す。
4. judgeTildeDiagnosticStatus  
calculateResponseTimeで得たresponse timeとconfigの時間条件を比較してDiagnosticStatus msgを作成する。
タイマごとにrequired pathに含まれるpathそれぞれに対してDiagnostic statusを作成する。
5. appendTildeDiag
judgeTildeDiagnosticStatusの内部関数で、required\_pathに含まれるDiagnosticStatusをmsgに追加する。
6. getTildeDiagLevel
calculateはこの中で行う。responsetimeとrequired_pathのdeadlineを比較して条件判定。
6. publish

