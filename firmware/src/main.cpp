// https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(500000);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    // Try to initialize!
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

    Serial.println("");
    delay(500);
}

void loop() {
    // Get new sensor events with the readings
    sensors_event_t acceleration, rotation, temp;
    mpu.getEvent(&acceleration, &rotation, &temp);

    // Print out the values
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

    Serial.println("");
    delay(100);
}
