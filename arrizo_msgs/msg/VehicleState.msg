# vehicle state reported from wired-chassis 

uint8   GEAR_DRIVE=11
uint8   GEAR_NEUTRAL=12
uint8   GEAR_REVERSE=13
uint8   GEAR_PAKING = 14
uint8   GEAR_INVALID = 200

bool    driverless
uint8   gear
bool    hand_brake
float32 speed            #km/h

float32 roadwheel_angle  #deg ccw(-) cw(+)

## light
bool turnlight_l
bool turnlight_r
bool brake_light
bool low_beam
bool high_beam

bool horn


