# Lateral control message for Ackermann-style platforms
# Note regarding tires: If the platform has multiple steering tires, the commands
# given here are for a virtual tire at the average lateral position of the steering tires.

builtin_interfaces/Time stamp
float32 steering_tire_angle # Desired angle of the steering tire in radians left (positive) or right (negative) of center (0.0)
float32 steering_tire_rotation_rate # Desired rate of change of the steering tire angle in radians per second
