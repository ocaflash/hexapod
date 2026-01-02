std_msgs/Header header

uint8 NO_ERROR = 0
uint8 SERVO_ERROR = 1

string servo_error_code
uint8 error_code
string servo_max_temperature
float32 max_temperature   # °C (not available on Maestro)
string servo_max_voltage
float32 max_voltage       # V (not available on Maestro)
string servo_min_voltage
float32 min_voltage       # V (not available on Maestro)
