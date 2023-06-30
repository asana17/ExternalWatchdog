uint16 UNKNOWN = 0

# For State
uint16 NORMAL = 1
uint16 SUPERVISOR_STOP = 2
uint16 COMFORTABLE_STOP= 3
uint16 MRM_SUCCEEDED = 4
uint16 MRM_FAILED = 5

uint16 state

# For ECU
uint16 MAIN = 1
uint16 SUB = 2
uint16 SUPERVISOR = 3
uint16 NONE = 4
uint16 fault_ecu
uint16 mrm_ecu
bool external_detected
bool comfortable_stop_after_switch
