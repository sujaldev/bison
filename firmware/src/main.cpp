// https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// https://RandomNerdTutorials.com/esp32-load-cell-hx711/

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "HX711.h"
#include "soc/rtc.h"

Adafruit_MPU6050 mpu;

HX711 scale;
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

void setup() {
    Serial.begin(1000000);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    // MPU6050
    if (!mpu.begin()) {
        // Failed to find MPU6050 on I2C bus, retrying...
        while (true) {
            delay(10);
        }
    }
    // Found MPU6050, begin initialization.
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    // HX711
    rtc_cpu_freq_config_t config;
    rtc_clk_cpu_freq_get_config(&config);
    rtc_clk_cpu_freq_set_config_fast(&config);

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale(23.65); // Calibration factor
    scale.tare(); // Reset the scale to 0.

    Serial.println("");
    delay(500);
}

void loop() {
    sensors_event_t acceleration, rotation, temp;
    mpu.getEvent(&acceleration, &rotation, &temp);

    Serial.print(acceleration.acceleration.x);
    Serial.print(" ");
    Serial.print(acceleration.acceleration.y);
    Serial.print(" ");
    Serial.print(acceleration.acceleration.z);
    Serial.print(" ");

    Serial.print(rotation.gyro.x);
    Serial.print(" ");
    Serial.print(rotation.gyro.y);
    Serial.print(" ");
    Serial.print(rotation.gyro.z);
    Serial.print(" ");

    Serial.println(scale.get_units());
    delay(100);
}
