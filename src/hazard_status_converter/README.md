# Hazard Status Converter

## description
Convert hazard_status in autoware_auto_system_msgs to hazard_status in watchdog msgs. This conversion adds `self_recoverable` paramter to previous one.

## `self_recoverable` paramter
This parameter express whether the ECU can run MRM on the self ECU when corresponding fault/error occurs. If `self_recoverable` equals to false, this means the voter need to operate MRM on different correctly working ECU.

## config
`self_recoverable` paramter is set to true on default. If the fault/error name is in the config, `self_recoverable` is set to false.
