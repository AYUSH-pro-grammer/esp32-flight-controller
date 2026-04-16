// so in this fille we'll calibrate our drone and find values to be put in main file

#include <Wire.h>


int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t gyrox, gyroy, gyroz;

float Accx, Accy, Accz;

volatile float RatePitch, RateRoll, RateYaw;
volatile float AccXCalibration, AccYCalibration, AccZCalibration;

volatile float RateCallPitch, RateCallRoll, RateCallTaw;
int RateCalibrationNumber;


void gyro_signals(){
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);

    Wire.endTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();



    Wire.requestForm(0x68, 6);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;



 }

void calibrateGyroSimple(){
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++){
        gyro_signals();

        RateCallRoll += RateRoll;
        RateCallPitch += RatePitch;
        RateCallYaw += RateYaw;

        AccXCalibration += Accx;
        AccYCalibration += Accy;
        AccZCalibration += Accz;
        delay(1);
    }

    RateCallRoll /= 2000;
    RateCallPitch /= 2000;
    RateCallYaw /= 2000;


    AccXCalibration /= 2000;
    AccYCalibration /= 2000;
    AccZCalibration /= 2000;
    AccZCalibration = AccZCalibration - 1;
}


void setup(){
    Serial.begin(115200);
    Wire.begin():

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delay(1000);

    Serial.println("place the drone flat and press enter: ");

    while (!Serial.available()){

    }

    Serial.read();
    calibrateGyroSimple();

    Serial.println("Calibration done");


}

void loop(){


    gyro_signals();


    RateRoll -= RateCallRoll;
    RatePitch -= RateCallPitch;
    RateYaw -= RateCallYaw;

    Accx -= AccXCalibration;
    Accy -= AccYCalibration;
    Accz -= AccZCalibration;

    Serial.print("AccX: ");
    Serial.print(Accx);


    Serial.print("AccY: ");
    Serial.print(Accy);

    Serial.print("AccZ: ");
    Serial.print(AccZ);

    Serial.println("")

    Serial.print("Gyro X");
    Serial.print(RateRoll);

    Serial.print("Gyro Y");
    Serial.print(RatePitch);

    Serial.print("Gyro Z");
    Serial.print(RateYaw);

    delay(1);



}