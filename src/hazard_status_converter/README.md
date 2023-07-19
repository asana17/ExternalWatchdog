# Hazard Status Converter

## Description
Convert the hazard_status of autoware_auto_system_msgs to that of watchdog msgs. This conversion adds the `self_recoverable` parameter to the previous one.

## `self_recoverable` Paramter
This parameter expresses whether the ECU can run MRM on the self ECU when a corresponding fault/error occurs. If `self_recoverable` is false, this means the voter need to operate MRM on a different ECU.

## Config
`self_recoverable` parameter is set to `true` on default. If the fault/error name is in the config, `self_recoverable` is set to false.
