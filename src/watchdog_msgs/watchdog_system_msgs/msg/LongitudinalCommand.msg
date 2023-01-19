# Longitudinal control message for all vehicle types
builtin_interfaces/Time stamp
float32 speed # Desired platform speed in meters per second. A positive value indicates movement in the positive X direction of the vehicle
float32 acceleration # Desired platform acceleration in meters per second squared. A positive value indicates acceleration while a negative value indicates deceleration.
float32 jerk # Desired platform jerk in meters per second cubed
