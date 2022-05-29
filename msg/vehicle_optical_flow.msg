# Optical flow in XYZ body frame in SI units.

uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id        # unique device ID for the sensor that does not change between power cycles

float32 pixel_flow_x_integral # accumulated optical flow in radians where a positive value is produced by a RH rotation about the X body axis
float32 pixel_flow_y_integral # accumulated optical flow in radians where a positive value is produced by a RH rotation about the Y body axis

float32 gyro_x_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the X body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
float32 gyro_y_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the Y body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
float32 gyro_z_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the Z body axis. Set to NaN if flow sensor does not have 3-axis gyro data.

float32 ground_distance_m	# Altitude / distance to ground in meters

uint32 integration_timespan	# accumulation timespan in microseconds

uint32 integration_timespan_gyro

uint8 quality	# Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

float32 max_flow_rate # Magnitude of maximum angular which the optical flow sensor can measure reliably
float32 min_ground_distance # Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance # Maximum distance from ground at which the optical flow sensor operates reliably



float32[2] flow_vel_body
float32 height
