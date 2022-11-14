# TEST

```
source autoware/install/setup.zsh
ros2 launch dummy_diag_publisher dummy_diag_publisher.launch.xml  ## terminal 1
ros2 launch system_error_monitor system_error_monitor.launch.xml  ## terminal 2
```

/diagnostic_aggにmessageがpublishされる
dummy_diagの場合1message12.67KB 1secで126.7KB

## /diagnostic_agg
msg型はdiagnostic_msgs/DiagnosticArray

- Raw Message Definition
```
# This message is used to send diagnostic information about the state of the robot
Header header #for timestamp
DiagnosticStatus[] status # an array of components being reported on
```
- Compact Message Definition
```
std_msgs/Header header
diagnostic_msgs/DiagnosticStatus[] status
```

### message例
```

header:
  stamp:
    sec: 1659339775
    nanosec: 86554355
  frame_id: ''
status:
- level: "\x03"
  name: /autoware/control/autonomous_driving/node_alive_monitoring/topic_status
  message: Stale
  hardware_id: ''
  values: []
- level: "\x03"
  name: /autoware/control/autonomous_driving/node_alive_monitoring
  message: Stale
  hardware_id: ''
  values:
  - key: topic_status
    value: Stale
- level: "\0"
  name: /autoware/control/autonomous_driving/performance_monitoring/lane_departure
  message: OK
  hardware_id: ''
  values:
  - key: 'dummy_diag_publisher_asana17_t4_17288_5525568804189871433: lane_departure'
    value: OK
- level: "\0"
  name: /autoware/control/autonomous_driving/performance_monitoring/lane_departure/dummy_diag_publisher_asana17_t4_17288_552556880418987...
```

### /diagnostic
msg型はdiagnostic_msgs/DiagnosticStatus


- Raw Message Definition
```
# This message holds the status of an individual component of the robot.
# 

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status
```

- Compact Message Definition
```
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3
byte level
string name
string message
string hardware_id
diagnostic_msgs/KeyValue[] values
```

# TODO

## system_error_monitor
system_error_monitorで/diagnostics_aggをどう処理しているか？  
`~/autoware/src/universe/autoware/system/system_error_monitor` 以下

一旦mapにdiagを登録 AutowareErrorMonitor::onDiagArray 各node名（diag.name）に対してheaderとdiag (diagnostic_aggのstatus1つ)に登録

judgeHazardStatus→ getLatestDiag(required moduleに含まれるdiag.nameをキーにforで各ノードのlatest_diagを取得)→ appendHazardDiag(ここで各ノードのhazardlevelを配列に追加し、hazard_statusを決定している)→ getHazardLevel→ isOverLevel(ここでdiagのlevelとhazardlevelを比較。failureを判定)

## core.cpp
/diagnostic_aggへのsubscription 参考: core.cppの252行目


## diagを保存するmapのデータ構造

### 現在の構造
(header,DiagBuffer)のmap  
DiagBuffer == std:deque<DiagStamped>
DiagStamped == struct<header, diag status>

タイマでheaderに関係なくstatusを送るならこちらが都合が良い

### 

現在の時間とdiagのheaderを比較して一定時間以上遅延がある場合、errorをwatchdogにpublishするのがよいかも。  
データ構造は流用してもよさそう。

