uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id              # unique device ID for the sensor that does not change between power cycles

float32[2] pixel_flow         # raw optical flow in radians where a positive value is produced by a RH rotation about the X or Y body axis

float32 dt                    # flow integration time in microseconds

uint8 quality                 # quality, 0: bad quality, 255: maximum quality

uint32 error_count

float32 max_flow_rate         # Magnitude of maximum angular which the optical flow sensor can measure reliably
float32 min_ground_distance   # Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance   # Maximum distance from ground at which the optical flow sensor operates reliably

uint8 MODE_UNKNOWN        = 0
uint8 MODE_BRIGHT         = 1
uint8 MODE_LOWLIGHT       = 2
uint8 MODE_SUPER_LOWLIGHT = 3

uint8 mode

bool delta_angle_available
float32[3] delta_angle
