# LSM6DS3 Driver for ESP32-P4

This project aims to provide a comprehensive ESP-IDF driver for the STMicroelectronics LSM6DS3 6-axis IMU sensor, featuring sensor fusion capabilities for computing absolute ground angles (roll and pitch). The driver is designed to be easy to integrate and use, providing both low-level sensor access and high-level orientation estimation functionality. It includes comprehensive calibration support, angle zero reference management, and NVS persistence for maintaining settings across reboots.

## Features

This driver implementation provides the following capabilities:

- **Complete sensor support**: Full implementation for LSM6DS3 accelerometer and gyroscope functionality
- **Multiple interfaces**: Support for both I2C and SPI communication protocols
- **Advanced sensor fusion**: Implementation of the Madgwick filter (quaternion-based) for high-accuracy orientation estimation
- **Lightweight sensor fusion**: Complementary filter implementation for applications requiring lower computational overhead
- **Ground angle computation**: Direct calculation of absolute ground angles (roll and pitch) relative to gravity, with 0-180° angle from vertical calculation
- **Calibration support**: Comprehensive calibration capabilities including automatic calibration routines, single-axis offset adjustment, and hardware offset registers
- **NVS persistence**: Non-volatile storage support for calibration data and angle zero references, ensuring settings persist across reboots
- **Angle zero reference**: Set and store reference angles for roll, pitch, and yaw, with GPIO input support for easy zero-setting
- **Modular design**: ESP-IDF component structure enabling straightforward integration into existing projects

## Hardware Setup

### I2C Connection (Default)

Connect the LSM6DS3 to your ESP32-P4:

- VDD → 3.3V
- GND → GND
- SCL → GPIO 22 (configurable)
- SDA → GPIO 21 (configurable)
- SDO/SA0 → GND (for address 0x6A) or VDD (for address 0x6B)

### GPIO Inputs (Example Application)

The example application uses the following GPIO pins for angle zero reference buttons:

- **GPIO 4** → Roll Zero button (connect to GND when pressed, pull-up enabled)
- **GPIO 5** → Pitch Zero button (connect to GND when pressed, pull-up enabled)
- **GPIO 18** → Yaw Zero button (connect to GND when pressed, pull-up enabled)

These GPIO pins are configurable in `main/main.c` by modifying the `GPIO_ROLL_ZERO`, `GPIO_PITCH_ZERO`, and `GPIO_YAW_ZERO` definitions.

### SPI Connection

For SPI mode, connect:
- CS → GPIO (configurable)
- SCL/SCLK → GPIO (configurable)
- SDA/SDI → GPIO (configurable)
- SDO → GPIO (configurable)

## Building and Flashing

1. Set the target to ESP32-P4:
```bash
idf.py set-target esp32p4
```

2. Configure the project (optional):
```bash
idf.py menuconfig
```

3. Build the project:
```bash
idf.py build
```

4. Flash and monitor:
```bash
idf.py flash monitor
```

## Usage

### Basic Example

The provided example application (`main/main.c`) demonstrates the following capabilities:

1. Initializing the LSM6DS3 sensor
2. Reading accelerometer and gyroscope data
3. Using Madgwick filter for sensor fusion
4. Using complementary filter for sensor fusion
5. Computing and displaying roll and pitch angles
6. Calculating absolute angle from vertical (0-180°)
7. Loading calibration data from NVS
8. Setting angle zero references via GPIO inputs (GPIO 4=Roll, GPIO 5=Pitch, GPIO 18=Yaw)
9. Persisting angle zero references to NVS

### Sensor Fusion

This implementation provides two distinct sensor fusion algorithms, each suited for different application requirements:

#### Madgwick Filter

The Madgwick filter offers superior accuracy at the cost of increased computational requirements. This implementation provides quaternion output that can be converted to Euler angles (roll, pitch, yaw), making it ideal for applications requiring precise orientation tracking.

```c
lsm6ds3_madgwick_filter_t filter;
lsm6ds3_madgwick_init(&filter, 0.1f, 104.0f);  // beta=0.1, sample_rate=104Hz

// In your loop:
lsm6ds3_madgwick_update(&filter, accel_mg, gyro_dps, dt);
lsm6ds3_euler_angles_t angles;
lsm6ds3_madgwick_get_euler(&filter, &angles);
// angles.roll and angles.pitch contain absolute ground angles
```

#### Complementary Filter

The complementary filter provides a computationally efficient alternative, offering direct roll and pitch angle output. This implementation is well-suited for applications where processing resources are limited or where real-time performance is critical.

```c
lsm6ds3_complementary_filter_t filter;
lsm6ds3_complementary_init(&filter, 0.98f, 104.0f);  // alpha=0.98, sample_rate=104Hz

// In your loop:
lsm6ds3_complementary_update(&filter, accel_mg, gyro_dps, dt);
float roll, pitch;
lsm6ds3_complementary_get_angles(&filter, &roll, &pitch);
// roll and pitch contain absolute ground angles
```

### API Reference

#### Initialization

```c
lsm6ds3_config_t config = {
    .interface = LSM6DS3_INTERFACE_I2C,
    .bus.i2c.bus_handle = i2c_bus_handle,
    .bus.i2c.address = 0x6A,
};
lsm6ds3_init(&handle, &config);
```

#### Reading Sensor Data

```c
float accel_mg[3];
float gyro_mdps[3];
lsm6ds3_read_accel(&handle, accel_mg);
lsm6ds3_read_gyro(&handle, gyro_mdps);
```

#### Configuration

```c
lsm6ds3_set_accel_odr(&handle, LSM6DS3_XL_ODR_104Hz);
lsm6ds3_set_accel_full_scale(&handle, LSM6DS3_2g);
lsm6ds3_set_gyro_odr(&handle, LSM6DS3_GY_ODR_104Hz);
lsm6ds3_set_gyro_full_scale(&handle, LSM6DS3_2000dps);
```

#### Calibration

The driver provides comprehensive calibration capabilities to improve sensor accuracy:

**Automatic Calibration (Recommended):**

```c
lsm6ds3_calibrate_accel(&handle, 100, 10);
lsm6ds3_calibrate_gyro(&handle, 100, 10);
```

**Manual Offset Setting:**

```c
float accel_offset[3] = {12.5f, -8.3f, 15.2f};
float gyro_offset[3] = {125.0f, -83.0f, 152.0f};
lsm6ds3_set_accel_offset(&handle, accel_offset);
lsm6ds3_set_gyro_offset(&handle, gyro_offset);
```

**Single-Axis Offset Adjustment:**

```c
lsm6ds3_set_accel_offset_axis(&handle, 0, 12.5f);
lsm6ds3_set_accel_offset_axis(&handle, 1, -8.3f);
lsm6ds3_set_accel_offset_axis(&handle, 2, 15.2f);
```

**Hardware Offset Registers (Accelerometer Only):**

```c
lsm6ds3_set_accel_hw_offset(&handle, 10, -5, 8);
lsm6ds3_set_accel_hw_offset_axis(&handle, 0, 10);
```

**Retrieving Calibration Data:**

```c
float accel_offset[3];
float gyro_offset[3];
lsm6ds3_get_accel_offset(&handle, accel_offset);
lsm6ds3_get_gyro_offset(&handle, gyro_offset);
```

**Clearing Calibration:**

```c
lsm6ds3_clear_accel_calibration(&handle);
lsm6ds3_clear_gyro_calibration(&handle);
```

**Saving/Loading Calibration to NVS:**

```c
nvs_flash_init();
lsm6ds3_save_calibration_to_nvs(&handle, "lsm6ds3");
lsm6ds3_load_calibration_from_nvs(&handle, "lsm6ds3");
```

**Calibration Guidelines:**

- **Accelerometer calibration**: Place the sensor on a flat, stable surface. The Z-axis offset accounts for gravity (1000 mg expected when stationary).
- **Gyroscope calibration**: Keep the sensor completely stationary during calibration. Any movement will affect the bias calculation.
- **Sample count**: Use 50-200 samples for good accuracy. More samples provide better averaging but take longer.
- **Calibration is applied automatically**: Once calibrated, offsets are automatically subtracted from all sensor readings.
- **Storage**: Calibration data is stored in RAM by default. Use NVS functions to persist across reboots.

#### Angle Zero Reference

The driver provides functionality to set reference angles (zero points) for roll, pitch, and yaw, allowing you to establish a custom reference orientation:

**Setting Angle Zero References:**

```c
lsm6ds3_angle_zero_t zero_ref = {0};
lsm6ds3_set_roll_zero(&zero_ref, current_roll);
lsm6ds3_set_pitch_zero(&zero_ref, current_pitch);
lsm6ds3_set_yaw_zero(&zero_ref, current_yaw);
```

**Applying Zero References to Angles:**

```c
float roll = 45.0f, pitch = 30.0f, yaw = 10.0f;
lsm6ds3_apply_angle_zero(&zero_ref, &roll, &pitch, &yaw);
```

**Saving/Loading Angle Zero References to NVS:**

```c
lsm6ds3_save_angle_zero_to_nvs(&zero_ref, "lsm6ds3");
lsm6ds3_load_angle_zero_from_nvs(&zero_ref, "lsm6ds3");
```

**GPIO Input Support (Example Application):**

The example application includes GPIO input support for setting zero angles:
- **GPIO 4**: Set Roll Zero (active low, pull-up enabled)
- **GPIO 5**: Set Pitch Zero (active low, pull-up enabled)
- **GPIO 18**: Set Yaw Zero (active low, pull-up enabled)

Pressing a GPIO button captures the current angle and sets it as the zero reference, automatically saving to NVS.

#### Ground Angle from Vertical (0-180°)

Calculate the absolute angle from vertical using roll and pitch:

```c
float roll = 30.0f;
float pitch = 45.0f;
float angle_from_vertical = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
// Returns: 0° = vertical up, 90° = horizontal, 180° = vertical down
```

This function computes the angle between the device's Z-axis and the vertical (gravity) direction:
- **0°**: Device pointing straight up (Z-axis aligned with gravity)
- **90°**: Device horizontal (Z-axis perpendicular to gravity)
- **180°**: Device pointing straight down (Z-axis opposite to gravity)

The function uses the formula: `acos(cos(roll) × cos(pitch))` to compute the angle from vertical.

## Output Format

The example program provides sensor data output in the following format:

```
Accel[mg] | Gyro[dps] | Madgwick[deg] | Complementary[deg] | Ground
X    Y    Z  |  X    Y    Z  | Roll Pitch Yaw | Roll Pitch | Angle
----------------------------------------------------------------------------
  12.3  45.6  78.9 |  0.12  0.34  0.56 |  2.34  5.67  0.00 |  2.45  5.78 |  5.89
```

- **Accel**: Accelerometer data in milli-g (mg), automatically corrected with calibration offsets if calibrated
- **Gyro**: Gyroscope data in degrees per second (dps), automatically corrected with calibration offsets if calibrated
- **Madgwick**: Euler angles from Madgwick filter (roll, pitch, yaw in degrees), adjusted for angle zero references
- **Complementary**: Roll and pitch angles from complementary filter (degrees), adjusted for angle zero references
- **Ground Angle**: Absolute angle from vertical (0-180°), where 0° = vertical up, 90° = horizontal, 180° = vertical down

## Important Considerations

- **Yaw angle limitations**: The LSM6DS3 is a 6DoF IMU without magnetometer capabilities. As such, yaw angle cannot be accurately determined without an external reference. Applications using the Madgwick filter should expect yaw drift over time.
- **Ground angle measurement**: Roll and pitch angles represent the absolute orientation relative to gravity, providing the essential information required for ground angle measurement applications.
- **Calibration storage**: Calibration offsets are stored in RAM by default and will be lost on power cycle. Use `lsm6ds3_save_calibration_to_nvs()` to persist calibration data. Hardware offset registers are also volatile and do not persist.
- **Angle zero references**: Zero references allow you to establish a custom reference frame. These are stored in RAM and should be saved to NVS for persistence. The example application demonstrates GPIO-based zero-setting with automatic NVS storage.
- **Filter parameter tuning**: 
  - **Madgwick `beta` parameter**: Lower values (0.01-0.1) increase reliance on accelerometer data, while higher values favor gyroscope measurements
  - **Complementary `alpha` parameter**: Higher values (0.95-0.99) increase trust in gyroscope data, while lower values place greater emphasis on accelerometer readings

## Technical Notes

### Data Reading Implementation

The driver uses a struct (`lsm6ds3_reg_t`) to hold sensor data, which contains both `uint8_t u8bit[6]` and `int16_t i16bit[3]` arrays. Since this is a struct (not a union), the arrays occupy separate memory locations. When reading raw sensor data, bytes are written to `u8bit`, but the `i16bit` array does not automatically update.

To ensure correct data interpretation, the driver **manually reconstructs** the `int16_t` values from the `uint8_t` bytes using little-endian byte order:

```c
// After reading bytes into u8bit[6]:
data_raw.i16bit[0] = (int16_t)((u8bit[1] << 8) | u8bit[0]);
data_raw.i16bit[1] = (int16_t)((u8bit[3] << 8) | u8bit[2]);
data_raw.i16bit[2] = (int16_t)((u8bit[5] << 8) | u8bit[4]);
```

This reconstruction is performed in:
- `lsm6ds3_read_accel()` and `lsm6ds3_read_accel_raw()`
- `lsm6ds3_read_gyro()` and `lsm6ds3_read_gyro_raw()`

**Important**: If modifying the driver code, ensure that `int16_t` values are always manually reconstructed from the byte array before use. Directly accessing `i16bit` after reading into `u8bit` will result in incorrect sensor readings and calibration failures.

### I2C Communication

The driver supports both I2C and SPI interfaces. For I2C:
- Default address: **0x6A** (SA0 pin LOW)
- Alternative address: **0x6B** (SA0 pin HIGH)
- The driver automatically probes both addresses during initialization if the default address fails

The I2C implementation reads sensor registers byte-by-byte in a loop to ensure reliable communication, matching the behavior of other ESP-IDF I2C drivers.

## Project Structure

```
.
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   └── main.c                 # Example application
└── components/
    └── lsm6ds3/
        ├── CMakeLists.txt
        ├── include/
        │   ├── lsm6ds3.h      # Main driver API
        │   └── lsm6ds3_fusion.h  # Sensor fusion API
        ├── lsm6ds3.c          # Driver implementation
        ├── lsm6ds3_fusion.c   # Sensor fusion implementation
        └── driver/
            ├── lsm6ds3_reg.h  # Register definitions
            └── lsm6ds3_reg.c  # Register functions
```

## License

This project is licensed under the MIT License. Please refer to [LICENSE](LICENSE) for complete license details.

Portions of this software incorporate code derived from STMicroelectronics' STMems_Standard_C_drivers, which are licensed under the BSD-3-Clause license. Additional information regarding these components can be found in [LICENSE_ST.txt](LICENSE_ST.txt).

### License Summary

- **Main code (wrapper, fusion, examples)**: MIT License
  - Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
  
- **STMicroelectronics driver files** (`components/lsm6ds3/driver/lsm6ds3_reg.*`): BSD-3-Clause License
  - Copyright (c) 2018-2025 STMicroelectronics

## Additional Documentation

- **[Ground Angle Computation Guide](GROUND_ANGLE_GUIDE.md)**: Detailed guide on computing absolute ground angles using sensor fusion
- **[LSM6DS3 vs MPU-6050 Comparison](LSM6DS3_VS_MPU6050.md)**: Comprehensive comparison with the popular MPU-6050 IMU
- **[Ground Truth Calculation Review](GROUND_TRUTH_REVIEW.md)**: Review of common ground truth calculation methods and best practices

## References

- [STMicroelectronics LSM6DS3 Datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3.pdf)
- [STMicroelectronics Standard C Drivers](https://github.com/STMicroelectronics/STMems_Standard_C_drivers)

