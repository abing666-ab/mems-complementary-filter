# mems_complementary_filter #

## References ##
- [Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs](https://www.mdpi.com/1424-8220/15/8/19302)
- [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools/tree/noetic/imu_complementary_filter)


## Component functions ##
+ Use measurements of gyroscope, acceleration and magnetic to estimate imu sensor's space orientation.
+ This is an improved version of stability and easy C API.

## Data structure ##

+ **mems_complementary_filter:** &nbsp; Object of filter.
```c
    typedef struct mems_complementary_filter mems_complementary_filter_t;
```

+ **imu_sample_t:** &nbsp; A Wrapper of imu input sensor data.
```c
    typedef struct imu_sample {
        //! In nanoseconds
        int64_t timestamp;

        //! Acceleration in meters per second squared (m/s²)
        float ax, ay, az;

        //! Gyro in radians per second (rad/s)
        float wx, wy, wz;
    } imu_sample_t;
```

+ **mag_sample_t:** &nbsp; A Wrapper of magnetic input sensor data.
```c
typedef struct mag_sample {
    //! In nanoseconds
    int64_t timestamp;

    //! Magnetic in GS(GAUSS).
    float mx, my, mz;
} mag_sample_t;
```

+ **quaternion_t:** &nbsp; A wrapper of output estimated oreientation in quaternion.
```c
typedef struct quaternion {
    //! In nanoseconds
    int64_t timestamp;

    float x;              //  The x element of a quaternion.
    float y;              //  The y element of a quaternion.
    float z;              //  The z element of a quaternion.
    float w;              //  The w element of a quaternion.
} quaternion_t;
```

+ **parameters_t:** &nbsp; A wrapper of filter's working parameter.
```c
typedef struct parameters {
    float gravity;                 // Default is constant 9.81.
    float angular_velocity_threshold;  //  A threshold which decide filter's state, default is 0.005.
    float acceleration_threshold;  // Default is 0.15.
    float delta_angular_velocity_threshold; // Default is 0.01.
    float gain_acc;                //  Accelerometer gain for the complementary filter, belongs in [0, 1]. Default is 0.01.
    float gain_mag;                //  Magnetometer gain for the complementary filter, belongs in [0, 1]. Default is 0.01.
    float bias_alpha;              //  Gain of the bias estimator, belongs in [0, 1]. Default is 0.01.
    float default_gyro_bias[3];    //  The initial bias of gyroscope, which can accelerate convergence. Default is [0, 0, 0]. 
    float default_acc_bias[3];     //  The initial bias of accelerometer. Reserved yet. Default is [0, 0, 0].
    bool do_bias_estimation;       //  Whether to do bias estimation of the angular velocity (gyroscope readings) or not. Default is true.
    bool do_adaptive_gain;         //  Whether to do adaptive gain or not. Default is true.
    bool use_magnetometer;         //  Whether to use magnetometer.
    bool output_relative_yaw;      //  Output orientation relative to start yaw angle, otherwise absolute to ENU.
} parameters_t;
```
## API ##

### filter_create ###
```c
    bool filter_create(const parameters_t* const _config, mems_complementary_filter_t** _filter)
```
+ **Parameter:**

	`_config`: Parameters needed by filter's initialization, For details, see 'parameters_t'.

    `_filter`: Created filter object.

+ **Return:**

	`bool`: True, create success. False: create failed.

+ **Usage scenarios:**
	Already has imu sensor data, and want to create a filter to estimate the space orientation of this sensor.

+ **Attention:**

	None.

&nbsp;

### filter_destroy ###
```c
    bool filter_destroy(mems_complementary_filter_t* _filter)
```
+ **Parameter:**

	`_filter`：Used up filter you want to destroy.

+ **Return:**

	`bool`：True, destory success. False， destory failed.

+ **Usage scenarios:**

	Recycling resources of used up filter.

+ **Attention:**

	None.

&nbsp;

### filter_reset ###
```c
    bool filter_reset(const parameters_t* const _config, mems_complementary_filter_t* const _filter)
```
+ **Parameter:**

	`_config`: Parameters needed by filter's initialization, For details, see 'parameters_t'.

	`_filter`: The filter that needs reset.

+ **Return:**

	`bool`：True, reset success. False, reset failed.

+ **Usage scenarios:**

	If you wants to restart filter from a new zero points, or wants to use a new config.

+ **Attention:**
	None.


&nbsp;


### filter_add_imu_measurement ###
```c
bool filter_add_imu_measurement(mems_complementary_filter_t* const _filter, const imu_sample_t* const _imu_sample)
```
+ **Parameter:**

	`_filter`: The filter that needs to update status.

	`_imu_sample`: IMU data needed by update operation.

+ **Return:**

	`bool`: True, update success. False, update failed.

+ **Usage scenarios:**

	Update filter status by imu sampling data.

+ **Attention:**
	None.


&nbsp;

### filter_add_mag_measurement ###
```c
    bool filter_add_mag_measurement(mems_complementary_filter_t* const _filter, const mag_sample_t* const _mag_sample)
```
+ **Parameter:**

	`_filter`: The filter that needs to update status.

	`_mag_sample`: Magnetic data needed by update operation.

+ **Return:**

	`bool`：True, success. False, failed.

+ **Usage scenarios:**

	Update filter status by magnetic sampling data.

+ **Attention:**
    None.

&nbsp;

### filter_add_measurement ###
```c
    bool filter_add_measurement(mems_complementary_filter_t* const _filter,
                                const imu_sample_t* const _imu_sample, const mag_sample_t* const _mag_sample)
```
+ **Parameter:**

	`_filter`: The filter that needs to update status.

	`_imu_sample`: IMU data needed by update operation.

    `_mag_sample`: Magnetic data needed by update operation.

+ **Return:**

	`bool`：True, success. False, failed.

+ **Usage scenarios:**

	Update filter status by imu and magnetic simultaneously.

+ **Attention:**
	+ IMU and magnetic data need do synchronize, then input into filter, timestamp in imu and mag should be same.

&nbsp;

### filter_get_orientation ###
```c
    bool filter_get_orientation(mems_complementary_filter_t* const _filter, quaternion_t* const _quat)
```
+ **Parameter:**

	`_filter`: The filter that needs to get current working status.

	`_quat`：FILTER_STATES_T type structure.

+ **Return:**

	`bool`：True, success. False, failed.

+ **Usage scenarios:**

	Getting filter's states at current time.

+ **Attention:**
	+ The orientation returns in this function is a transform form imu body frame to an external frame. (Might be robot or world, dependent on installation.), which is qwb in formula. If you want an orientation in imu body frame, please do inverse. 

<br/></br>

## Referrence parameters about using case ##
### BMI088 in sweep robot, IMU freq 100Hz. ###
```c
	const parameters_t parameters {
		.gravity = 9.81f,
		.angular_velocity_threshold = 0.005f,
		.acceleration_threshold = 0.1,
		.delta_angular_velocity_threshold = 0.01f,
		.gain_acc = 0.01f,
		.bias_alpha = 0.01f,
		.default_gyro_bias = {0.f, 0.f, 0.f},
		.default_acc_bias = {0.f, 0.f 0.f},
        .do_bias_estimation = true,
        .do_adaptive_gain = true,
        .use_magnetometer = false,
        .output_relative_yaw = false
	};
```

### TDK in AR/VR, IMU freq 1000Hz. ###
```c
    const parameters_t parameters = {
        .gravity = 9.81f,
        .angular_velocity_threshold = 0.008f,
        .acceleration_threshold = 0.15f,
        .delta_angular_velocity_threshold = 0.005f,
        .gain_acc = 0.0005f,
        .gain_mag = 0.002f,
        .bias_alpha = 0.002f,
        .default_gyro_bias = {gyro_bias.x, gyro_bias.y, gyro_bias.z},
        .default_acc_bias = {accel_bias.x, accel_bias.y, accel_bias.z},
        .do_bias_estimation = true,
        .do_adaptive_gain = true,
        .use_magnetometer = false,
        .output_relative_yaw = false
	};
```

