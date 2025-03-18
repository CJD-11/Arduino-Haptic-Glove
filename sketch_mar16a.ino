#include <Wire.h>
#include "MPU9250.h"

MPU9250 imu;
const int flex1 = A0;
const int flex2 = A1;
const int fsr1 = A2;
const int fsr2 = A3;
const int haptic1 = 9;

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

    // Print CSV Headers
    Serial.println("Flex1,Flex2,FSR1,FSR2,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ");
    
    Serial.println("Setup Complete!");
}

void readSensors(int *flex, int *fsr) {
    flex[0] = analogRead(flex1);
    flex[1] = analogRead(flex2);
    fsr[0] = analogRead(fsr1);
    fsr[1] = analogRead(fsr2);
}

void handleHapticFeedback(int *fsr) {
    if (fsr[0] > 500 || fsr[1] > 500) {
        digitalWrite(haptic1, HIGH);
    } else {
        digitalWrite(haptic1, LOW);
    }
}

void handleIMUHapticFeedback(float ax, float ay, float az) {
    if (abs(ax) > 1.5 || abs(ay) > 1.5 || abs(az) > 1.5) {
        digitalWrite(haptic1, HIGH);
    } else {
        digitalWrite(haptic1, LOW);
    }
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

    // Format CSV Data
    Serial.print(flexData[0]); Serial.print(",");
    Serial.print(flexData[1]); Serial.print(",");
    Serial.print(fsrData[0]); Serial.print(",");
    Serial.print(fsrData[1]); Serial.print(",");
    Serial.print(ax, 2); Serial.print(",");
    Serial.print(ay, 2); Serial.print(",");
    Serial.print(az, 2); Serial.print(",");
    Serial.print(gx, 2); Serial.print(",");
    Serial.print(gy, 2); Serial.print(",");
    Serial.print(gz, 2); Serial.print(",");
    Serial.print(mx, 2); Serial.print(",");
    Serial.print(my, 2); Serial.print(",");
    Serial.println(mz, 2); // End line for CSV row

    // Handle Haptic Feedback
    handleHapticFeedback(fsrData);
    handleIMUHapticFeedback(ax, ay, az);

    delay(50);
}

