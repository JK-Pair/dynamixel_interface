ADDR_ID_MOTOR               = 7
ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_LED_RED                = 65
ADDR_GOAL_CURRENT           = 102
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_LOAD           = 126
ADDR_PRESENT_VELOCITY       = 128

LEN_ONE_BYTE                = 1         # Data Byte Length
LEN_GOAL_POSITION           = 4         # Data Byte Length
LEN_PRESENT_POSITION        = 4         # Data Byte Length
LEN_PRESENT_LOAD            = 2         # Data Byte Length
LEN_PRESENT_VELOCITY        = 4         # Data Byte Length

DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

PARAM_ENABLE               = 1                 # Value for enabling the torque
PARAM_DISABLE              = 0                 # Value for disabling the torque

INIT_RAW_DXL = 2048

ADDR_DICT = {
    "ID" : [ADDR_ID_MOTOR, LEN_ONE_BYTE],
    "LED" : [ADDR_LED_RED, LEN_ONE_BYTE],
    "Torque Enable" : [ADDR_TORQUE_ENABLE, LEN_ONE_BYTE],
    "Present Load" : [ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD],
    "Present Velocity" : [ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY],
    "Present Position" : [ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION]
}
