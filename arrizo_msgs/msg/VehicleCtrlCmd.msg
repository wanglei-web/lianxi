##

uint8 set_driverlessMode
bool set_remoteStart
bool set_handBrake
bool set_turnLight_R
bool set_turnLight_L
bool set_lowBeam
bool set_reverseLight
bool set_brakeLight
bool set_horn

uint8 GEAR_INITIAL = 0
uint8 GEAR_DRIVE   = 1
uint8 GEAR_REVERSE = 9
uint8 GEAR_NEUTRAL = 10
uint8 GEAR_PARKING = 11

uint8 set_gear
float32 set_speed
float32 set_brake
float32 set_accelerate
float32 set_roadWheelAngle   #CCW(-), CW(+)

