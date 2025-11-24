# Ground Angle Computation Guide

This guide explains how to use the LSM6DS3 driver to compute absolute ground angles (roll and pitch) using sensor fusion.

## What are Ground Angles?

Ground angles represent the orientation of your device relative to the ground (gravity):
- **Roll**: Rotation around the X-axis (left/right tilt)
- **Pitch**: Rotation around the Y-axis (forward/backward tilt)

These angles are computed by fusing accelerometer and gyroscope data to get accurate, drift-free orientation.

## Quick Start

### Using Complementary Filter (Recommended for Ground Angles)

The complementary filter is simpler, faster, and perfect for ground angle measurement:

```c
#include "lsm6ds3_fusion.h"

lsm6ds3_complementary_filter_t filter;
lsm6ds3_complementary_init(&filter, 0.98f, 104.0f);

float accel_mg[3];
float gyro_mdps[3];
float roll, pitch;

while (1) {
    lsm6ds3_read_accel(&sensor_handle, accel_mg);
    lsm6ds3_read_gyro(&sensor_handle, gyro_mdps);
    
    float gyro_dps[3] = {
        gyro_mdps[0] / 1000.0f,
        gyro_mdps[1] / 1000.0f,
        gyro_mdps[2] / 1000.0f
    };
    
    float dt = 1.0f / 104.0f;
    lsm6ds3_complementary_update(&filter, accel_mg, gyro_dps, dt);
    lsm6ds3_complementary_get_angles(&filter, &roll, &pitch);
    
    printf("Roll: %.2f°, Pitch: %.2f°\n", roll, pitch);
    
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

### Using Madgwick Filter

The Madgwick filter provides more accurate results but is computationally more intensive:

```c
#include "lsm6ds3_fusion.h"

lsm6ds3_madgwick_filter_t filter;
lsm6ds3_madgwick_init(&filter, 0.1f, 104.0f);

float accel_mg[3];
float gyro_mdps[3];
lsm6ds3_euler_angles_t angles;

while (1) {
    lsm6ds3_read_accel(&sensor_handle, accel_mg);
    lsm6ds3_read_gyro(&sensor_handle, gyro_mdps);
    
    float gyro_dps[3] = {
        gyro_mdps[0] / 1000.0f,
        gyro_mdps[1] / 1000.0f,
        gyro_mdps[2] / 1000.0f
    };
    
    float dt = 1.0f / 104.0f;
    lsm6ds3_madgwick_update(&filter, accel_mg, gyro_dps, dt);
    lsm6ds3_madgwick_get_euler(&filter, &angles);
    
    printf("Roll: %.2f°, Pitch: %.2f°\n", angles.roll, angles.pitch);
    
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

## Filter Tuning

### Complementary Filter Alpha Parameter

- **Higher alpha (0.95-0.99)**: More trust in gyroscope
  - Better for dynamic movements
  - Less affected by vibration
  - May drift over long periods
  
- **Lower alpha (0.90-0.95)**: More trust in accelerometer
  - Better for static measurements
  - Self-correcting (no drift)
  - More affected by vibration/acceleration

**Recommended**: Start with `0.98` for general use.

### Madgwick Filter Beta Parameter

- **Lower beta (0.01-0.05)**: More trust in accelerometer
  - Better for slow movements
  - More stable
  
- **Higher beta (0.1-0.2)**: More trust in gyroscope
  - Better for fast movements
  - More responsive

**Recommended**: Start with `0.1` for general use.

## Important Notes

1. **Sample Rate**: Match your filter sample rate to your sensor ODR (Output Data Rate)
   - If sensor is at 104 Hz, use `sample_rate_hz = 104.0f`
   - Calculate `dt = 1.0f / sample_rate_hz`

2. **Units**: 
   - Accelerometer: milli-g (mg) - already provided by `lsm6ds3_read_accel()`
   - Gyroscope: degrees per second (dps) - convert from mdps: `dps = mdps / 1000.0f`

3. **Calibration**: For best results, calibrate the sensor when the device is stationary:
   - Place device on flat surface
   - Use `lsm6ds3_calibrate_accel()` to automatically calculate offsets
   - Use `lsm6ds3_calibrate_gyro()` while device is completely still
   - Save calibration to NVS for persistence: `lsm6ds3_save_calibration_to_nvs()`

4. **Angle Zero Reference**: Set a custom reference orientation:
   - Use `lsm6ds3_set_roll_zero()` / `lsm6ds3_set_pitch_zero()` to establish zero angles
   - Apply zero references: `lsm6ds3_apply_angle_zero()` before using angles
   - Save to NVS: `lsm6ds3_save_angle_zero_to_nvs()` for persistence
   - Example application includes GPIO buttons for easy zero-setting

5. **Yaw Angle**: The LSM6DS3 is a 6DoF IMU (no magnetometer), so yaw cannot be accurately determined. Only roll and pitch are meaningful for ground angle measurement.

## Angle Zero Reference

Set a custom reference orientation so angles are relative to your desired zero point:

```c
lsm6ds3_angle_zero_t zero_ref = {0};

lsm6ds3_load_angle_zero_from_nvs(&zero_ref, "lsm6ds3");

lsm6ds3_euler_angles_t angles;
lsm6ds3_madgwick_get_euler(&filter, &angles);

float roll = angles.roll;
float pitch = angles.pitch;
float yaw = angles.yaw;

lsm6ds3_apply_angle_zero(&zero_ref, &roll, &pitch, &yaw);

printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", roll, pitch, yaw);
```

## Ground Angle from Vertical (0-180°)

Calculate the absolute angle from vertical using roll and pitch angles:

```c
float roll, pitch;
lsm6ds3_complementary_get_angles(&filter, &roll, &pitch);

float angle_from_vertical = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
printf("Angle from vertical: %.2f°\n", angle_from_vertical);
```

**Output Range:**
- **0°**: Device pointing straight up (vertical, Z-axis aligned with gravity)
- **90°**: Device horizontal (Z-axis perpendicular to gravity)
- **180°**: Device pointing straight down (vertical, Z-axis opposite to gravity)

**Formula:** `angle = acos(cos(roll) × cos(pitch))`

This function is useful for applications that need to know how far the device has tilted from vertical, regardless of the direction of tilt.

## Example: Level Indicator

```c
void check_level(float roll, float pitch, float threshold_deg)
{
    if (fabs(roll) < threshold_deg && fabs(pitch) < threshold_deg) {
        printf("Device is level!\n");
    } else {
        printf("Tilt detected: Roll=%.2f°, Pitch=%.2f°\n", roll, pitch);
    }
}
```

## Example: Angle from Vertical Indicator

```c
void check_vertical_angle(float roll, float pitch)
{
    float angle = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
    
    if (angle < 5.0f) {
        printf("Device is nearly vertical (pointing up): %.2f°\n", angle);
    } else if (angle > 175.0f) {
        printf("Device is nearly vertical (pointing down): %.2f°\n", angle);
    } else if (fabs(angle - 90.0f) < 5.0f) {
        printf("Device is nearly horizontal: %.2f°\n", angle);
    } else {
        printf("Device angle from vertical: %.2f°\n", angle);
    }
}
```

## Troubleshooting

**Problem**: Angles drift over time
- **Solution**: Lower the alpha/beta parameter to trust accelerometer more

**Problem**: Angles are noisy/jumpy
- **Solution**: Increase the alpha/beta parameter to trust gyroscope more, or lower the sensor ODR

**Problem**: Angles don't match expected values
- **Solution**: Check sensor orientation and coordinate system. You may need to swap axes or negate values based on your mounting. Consider setting angle zero references to establish your desired reference frame.

**Problem**: Need to establish a custom reference orientation
- **Solution**: Use angle zero reference functions (`lsm6ds3_set_roll_zero()`, etc.) to set the current orientation as zero. This is particularly useful when mounting the sensor in a non-standard orientation.

