# how to run

## test TILDE

ros2 launch tilde_sample multi_publisher_relay.launch.py

- `ros2 topic list`

```

/parameter_events
/relay_with_stamp
/relay_with_stamp/message_tracking_tag
/relay_without_stamp
/relay_without_stamp/message_tracking_tag
/rosout
/topic_with_stamp
/topic_with_stamp/message_tracking_tag
/topic_without_stamp
/topic_without_stamp/message_tracking_tag

```

- `ros2 topic echo /relay_with_stamp/message_tracking_tag`
```

header:
  stamp:
    sec: 1664878834
    nanosec: 65759017
  frame_id: ''
output_info:
  topic_name: /relay_with_stamp
  node_fqn: ''
  seq: 62
  pub_time:
    sec: 1664878834
    nanosec: 65762356
  pub_time_steady:
    sec: 123096
    nanosec: 583892717
  has_header_stamp: true
  header_stamp:
    sec: 1664878834
    nanosec: 39636070
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1664878834
    nanosec: 39891008
  sub_time_steady:
    sec: 123096
    nanosec: 558020559
  has_header_stamp: true
  header_stamp:
    sec: 1664878834
    nanosec: 39636070
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1664878834
    nanosec: 40028986
  sub_time_steady:
    sec: 123096
    nanosec: 558158544
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
---
header:
  stamp:
    sec: 1664878835
    nanosec: 65751118
  frame_id: ''
output_info:
  topic_name: /relay_with_stamp
  node_fqn: ''
  seq: 63
  pub_time:
    sec: 1664878835
    nanosec: 65754701
  pub_time_steady:
    sec: 123097
    nanosec: 583899703
  has_header_stamp: true
  header_stamp:
    sec: 1664878835
    nanosec: 39611382
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1664878835
    nanosec: 39830166
  sub_time_steady:
    sec: 123097
    nanosec: 557974398
  has_header_stamp: true
  header_stamp:
    sec: 1664878835
    nanosec: 39611382
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1664878835
    nanosec: 39969432
  sub_time_steady:
    sec: 123097
    nanosec: 558113664
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
---
```

- `ros2 topic echo /topic_with_stamp/message_tracking_tag`

```

header:
  stamp:
    sec: 1664878948
    nanosec: 37982224
  frame_id: ''
output_info:
  topic_name: /topic_with_stamp
  node_fqn: ''
  seq: 176
  pub_time:
    sec: 1664878948
    nanosec: 37985285
  pub_time_steady:
    sec: 123210
    nanosec: 557781584
  has_header_stamp: true
  header_stamp:
    sec: 1664878948
    nanosec: 37972034
input_infos: []
---
header:
  stamp:
    sec: 1664878949
    nanosec: 37895864
  frame_id: ''
output_info:
  topic_name: /topic_with_stamp
  node_fqn: ''
  seq: 177
  pub_time:
    sec: 1664878949
    nanosec: 37897472
  pub_time_steady:
    sec: 123211
    nanosec: 557708114
  has_header_stamp: true
  header_stamp:
    sec: 1664878949
    nanosec: 37888664
input_infos: []
---
```
