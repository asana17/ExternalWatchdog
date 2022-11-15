uint8 NO_FAULT = 0
uint8 SAFE_FAULT = 1
uint8 LATENT_FAULT = 2
uint8 SINGLE_POINT_FAULT = 3

uint8 level
bool emergency
watchdog_system_msgs/TildeDiagnosticStatus[] tilde_diag_no_fault
watchdog_system_msgs/TildeDiagnosticStatus[] tilde_diag_safe_fault
watchdog_system_msgs/TildeDiagnosticStatus[] tilde_diag_latent_fault
watchdog_system_msgs/TildeDiagnosticStatus[] tilde_diag_single_point_fault
