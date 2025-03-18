#include <Wire.h>      // Ensure Wire library is included
#include <MPU9250.h>   // Include MPU9250 sensor library

#ifndef PI
#define PI 3.14159265358979323846
#endif

// IMU Sensor
MPU9250 imu;

// Define Pins
const int haptic1 = 9;  // Adjust these values as per your wiring
const int haptic2 = 10; 
const int flexPins[5] = {A0, A1, A2, A3, A4};  // Flex Sensors
const int fsrPins[5] = {A5, A6, A7, A8, A9};   // Force Sensors (adjust based on board!)

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Started!");  // Debug message

    Wire.begin();
    Serial.println("Wire initialized."); 
    delay(2000);  

    // Initialize MPU9250
    Serial.println("Initializing MPU9250...");
    if (imu.setup(0x68)) {  // Corrected check (should return true if successful)
        Serial.println("MPU9250 Initialized successfully.");
    } else {
        Serial.println("MPU connection failed! Skipping IMU setup...");
    }

    // Initialize Haptic Motors
    pinMode(haptic1, OUTPUT);
    pinMode(haptic2, OUTPUT);
    Serial.println("Haptic motors initialized.");

    Serial.println("Setup Complete!");
}

// Function to read sensor values
void readSensors(int flexData[], int fsrData[]) {
    for (int i = 0; i < 5; i++) {
        flexData[i] = analogRead(flexPins[i]);
        fsrData[i] = analogRead(fsrPins[i]);
    }
}

// Function to handle haptic feedback
void handleHapticFeedback(int fsrData[]) {
    digitalWrite(haptic1, fsrData[0] > 500 ? HIGH : LOW);
    digitalWrite(haptic2, fsrData[1] > 500 ? HIGH : LOW);
}

void loop() {
    int flexData[5], fsrData[5];
    readSensors(flexData, fsrData);

    if (!imu.update()) {  // Ensure we have valid IMU data
        Serial.println("IMU update failed. Skipping this cycle...");
        delay(50);
        return;  
    }

    

    // Retrieve IMU Data
    float ax = imu.getAccX();
    float ay = imu.getAccY();
    float az = imu.getAccZ();
    float gx = imu.getGyroX();
    float gy = imu.getGyroY();
    float gz = imu.getGyroZ();
    float mx = imu.getMagX();
    float my = imu.getMagY();
    float mz = imu.getMagZ();

    // Print Sensor Data
    Serial.println("=== Sensor Data ===");
    for (int i = 0; i < 5; i++) {
        Serial.print("Flex Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(flexData[i]);
    }
    for (int i = 0; i < 5; i++) {
        Serial.print("FSR Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(fsrData[i]);
    }

    // Print IMU Data
    Serial.println("--- IMU Data ---");
    Serial.print("Accel (X, Y, Z): ");
    Serial.print(ax, 2);
    Serial.print(", ");
    Serial.print(ay, 2);
    Serial.print(", ");
    Serial.println(az, 2);
    
    Serial.print("Gyro (X, Y, Z): ");
    Serial.print(gx, 2);
    Serial.print(", ");
    Serial.print(gy, 2);
    Serial.print(", ");
    Serial.println(gz, 2);
    
    Serial.print("Mag (X, Y, Z): ");
    Serial.print(mx, 2);
    Serial.print(", ");
    Serial.print(my, 2);
    Serial.print(", ");
    Serial.println(mz, 2);
    
    Serial.println("======================\n");

    // Format CSV Data
    String dataString;
    for (int i = 0; i < 5; i++) {
        dataString += String(flexData[i]) + ",";
    }
    for (int i = 0; i < 5; i++) {
        dataString += String(fsrData[i]) + ",";
    }
    dataString += String(ax) + "," + String(ay) + "," + String(az) + ",";
    dataString += String(gx) + "," + String(gy) + "," + String(gz) + ",";
    dataString += String(mx) + "," + String(my) + "," + String(mz);

    Serial.println(dataString);

    // Handle Haptic Feedback
    handleHapticFeedback(fsrData);
    
    delay(50);  // Adjust sampling rate as needed
}

