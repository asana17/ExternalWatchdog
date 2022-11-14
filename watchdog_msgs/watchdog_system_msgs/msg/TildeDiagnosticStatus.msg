# Possible levels of operations
uint8 OK = 0
uint8 WARN= 1
uint8 ERROR = 2
uint8 STALE = 3

uint8 level
string start_point # a start point topic name of path
string end_point # an end point topic name of path
string message # a description of the status

