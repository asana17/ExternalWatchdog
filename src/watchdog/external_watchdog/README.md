# external watchdog

## 実装

emergency_handler→ vehicle_cmd_gate→ raw_vehicle_cmd_converter→ vehicle interface(pacmod interface)までを実装
  

  一旦は急停止できればよい。

今後を考えるとpacmod_interfaceはある程度残しておいたほうが良いかも
各種cmdとして停止を送る。

## autoware内のref

### emergency_handler
hazard_statusをsubscribeして、`AckermannControlCommand`, `HazardLightsCommand`, `GearCommand`, `EmergencyState`をpublish  
AckermanControlCommand等はemergency stopを行うためのパラメタに設定
またEmergency stateをpublish(`MRM_OPERATING`, `MRM_SUCCEEDED`, `MRM_FAILED`)
1. AckermannControlCommandのpublish ok
2. HazardLightsCommandのpublish  
emergency_holdをonのときはhazard lightをつけっぱなしにする機能。一旦off
3. GearCommandのpublish  
止まった後parkingを使う機能。一旦off
4. EmergencyStateのpublish  
hazard_statusがtimeoutしたときはMRM\_OPERATINGに一旦設定 MRM\_OPERATINGの後停止した場合は、MRM_SUCCEEDEDに移行




### vehicle_cmd_gate
emergencyの`AckermanControlCommand`, `HazardLightsCommand`, `GearCommand`をsubscribe それぞれの`onEmergencyCmd`を実行  
`onEmergencyCmd`実行時、Emergency stateが届いていて`is_emergency_state_`がtrueなら`vehicle_cmd_emergency`, `control_cmd`をpublishする
タイマ毎に`is_emergency_state_`がtrueならemergencyの`turn_indicator`, `hazard_light`, `gear`をpublish

### raw_vehicle_cmd_converter
`control_cmd`から`actuation_cmd`に変換

### pacmod_interface

`vehicle_cmd_gate`, `raw_vehicle_cmd_converter`がpublishするtopicに加えてpacmodからもtopicをsubscribeする（現在の状態をpacmod経由で取得）
出力はpacmodへの各種topicと、autowareへのstatus情報
`pacmod_interface.cpp`内にemergency stopがある emergencyのときは関数`publishCommands()`内でパラメタ`desired_brake`を`emergency_brake_`に設定してpacmodにpublish


## vehicle_interfaceの改変
external_watchdogからpacmodにtopicをpublishする場合, vehicle_interfaceからはpublishされないようにしたい。  
watchdogからis_direct_mrm_operatedというtopicをpublishし、vehicle_interface内で判定する。


## refactoring on 3/8
pacmod依存になっている部分を修正  
emergency_handler→ vehicle_cmd_gateまでをExternal watchdogに実装 直接CANに出すためのinterfaceは別で実装することにする。  
最終的に以下のtopicに相当するものをwatchdogからpublishできればよい
1. /control/command/control_cmd
1. /control/command/gear_cmd
1. /control/command/emergency_cmd
1. /control/command/actuation_cmd
1. /control/command/turn_indicators_cmd
1. /control/command/hazard_lights_cmd
