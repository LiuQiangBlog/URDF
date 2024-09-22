# The contents of this message includes the full state of the robot including, if it exists, the floating base joint. It is intended to be the full
# state in the correct order per the RDL model state element ordering. The ordering is critical here. If the order is incorrect, the result will
# also be incorrect

std_msgs/Header header

float64[] position
float64[] velocity
float64[] acceleration
float64[] effort
