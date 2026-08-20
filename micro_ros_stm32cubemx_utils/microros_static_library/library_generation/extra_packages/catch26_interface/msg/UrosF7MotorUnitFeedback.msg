DeviceInfo info
float32 position
float32 velocity
float32 current
uint8 state
uint8 unit_message_code

uint8 STATE_DISABLE = 0
uint8 STATE_POSITION = 1
uint8 STATE_VELOCITY = 2
uint8 STATE_CURRENT = 3

uint8 CODE_NONE = 0
uint8 CODE_NORMAL = 1
uint8 CODE_DISCONNECTION = 2
uint8 CODE_ERROR = 3