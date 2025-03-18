#include <Wire.h>   // Ensure Wire library is included
#include <SD.h>     // Include SD library if using SD card
#include <SPI.h>    // Include SPI library if using SD card
#include <MPU9250.h>  // Include MPU9250 sensor library

MPU9250 imu;  // Declare IMU sensor object

// Define pin numbers for haptic motors
const int haptic1 = 9;  // Adjust these values as per your wiring
const int haptic2 = 10; 

// Define SD card chip select pin
const int chipSelect = 4; // Adjust according to your board

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Started!");  // Debug message

    Wire.begin();
    Serial.println("Wire initialized."); 
    delay(2000);  

    // Initialize MPU9250
    Serial.println("Initializing MPU9250...");
    if (!imu.setup(0x68)) {  // Corrected check
        Serial.println("MPU9250 Initialized successfully.");
    } else {
        Serial.println("MPU connection failed! Skipping IMU setup...");
    }

    // Initialize SD Card
    Serial.println("Initializing SD Card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("SD Card Error! Check connections.");
    } else {
        Serial.println("SD Card Ready.");
    }

    // Initialize Haptic Motors
    pinMode(haptic1, OUTPUT);
    pinMode(haptic2, OUTPUT);
    Serial.println("Haptic motors initialized.");

    Serial.println("Setup Complete!");
}

void loop() {
    if (imu.update()) {  // Check if new data is available
        Serial.print("Accel X: "); Serial.print(imu.getAccX());
        Serial.print(" | Accel Y: "); Serial.print(imu.getAccY());
        Serial.print(" | Accel Z: "); Serial.println(imu.getAccZ());

        Serial.print("Gyro X: "); Serial.print(imu.getGyroX());
        Serial.print(" | Gyro Y: "); Serial.print(imu.getGyroY());
        Serial.print(" | Gyro Z: "); Serial.println(imu.getGyroZ());

        Serial.print("Mag X: "); Serial.print(imu.getMagX());
        Serial.print(" | Mag Y: "); Serial.print(imu.getMagY());
        Serial.print(" | Mag Z: "); Serial.println(imu.getMagZ());

        Serial.println("----------");
    }
    delay(500);  // Adjust sampling rate as needed
}

