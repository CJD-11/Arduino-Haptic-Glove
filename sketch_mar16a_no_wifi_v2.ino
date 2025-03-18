#include <Wire.h>      // Ensure Wire library is included
#include <MPU9250.h>   // Include MPU9250 sensor library

#ifndef PI
#define PI 3.14159265358979323846
#endif

// IMU Sensor
MPU9250 imu;

// Define Pins
const int haptic1 = 30;  // Haptic motor 1
const int flexPins[2] = {A0, A1};  // Flex Sensors
const int fsrPins[2] = {A5, A6};   // Force Sensors (adjust based on board!)

void setup() {
    Serial.begin(115200);
    Serial.println("Setup Started!");  

    Wire.begin();
    Serial.println("Wire initialized."); 
    delay(2000);  

    // Initialize MPU9250
    Serial.println("Initializing MPU9250...");
    if (imu.setup(0x68)) {  
        Serial.println("MPU9250 Initialized successfully.");
    } else {
        Serial.println("MPU connection failed! Skipping IMU setup...");
    }

    // Initialize Haptic Motors
    pinMode(haptic1, OUTPUT);
    Serial.println("Haptic motors initialized.");

    Serial.println("Setup Complete!");
}

// Function to read sensor values
void readSensors(int flexData[], int fsrData[]) {
    for (int i = 0; i < 2; i++) {
        flexData[i] = analogRead(flexPins[i]);
        fsrData[i] = analogRead(fsrPins[i]);
    }
}

// Function to handle PWM-based haptic feedback from force sensors
void handleHapticFeedback(int fsrData[]) {
    int intensity1 = map(fsrData[0], 0, 1023, 0, 255);  
    int intensity2 = map(fsrData[1], 0, 1023, 0, 255);  

    analogWrite(haptic1, intensity1);
}

// Optional: Function to handle haptic feedback based on motion
void handleIMUHapticFeedback(float ax, float ay, float az) {
    float totalAccel = sqrt(ax * ax + ay * ay + az * az);  // Calculate total acceleration
    int intensity = map(totalAccel, 0, 10, 0, 255);  // Scale IMU response to motor intensity
    intensity = constrain(intensity, 0, 255);  // Ensure within valid range

    analogWrite(haptic1, intensity);
}

void loop() {
    int flexData[2], fsrData[2];
    readSensors(flexData, fsrData);

    if (!imu.update()) {  
        Serial.println("IMU update failed. Skipping this cycle...");
        delay(50);
        return;  
    }

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
    for (int i = 0; i <2; i++) {
        Serial.print("Flex Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(flexData[i]);
    }
    for (int i = 0; i < 2; i++) {
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
    for (int i = 0; i < 2; i++) {
        dataString += String(flexData[i]) + ",";
    }
    for (int i = 0; i < 2; i++) {
        dataString += String(fsrData[i]) + ",";
    }
    dataString += String(ax) + "," + String(ay) + "," + String(az) + ",";
    dataString += String(gx) + "," + String(gy) + "," + String(gz) + ",";
    dataString += String(mx) + "," + String(my) + "," + String(mz);

    Serial.println(dataString);

    // Handle Haptic Feedback
    handleHapticFeedback(fsrData);       // Force sensor-based feedback
    handleIMUHapticFeedback(ax, ay, az); // Motion-based feedback (optional)

    delay(50);  // Adjust sampling rate as needed
}
