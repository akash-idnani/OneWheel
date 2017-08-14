#include <Arduino.h>
#include <LiquidCrystal.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <SabertoothSimplified.h>
#include <Button.h>
#include <Rotary.h>
#include <EEPROMex.h>

LiquidCrystal lcd(5, 9, 8, 6, 7, 4);
int activeInput = 0;

Button pidButton(10, true, true, 20);

Rotary r(11, A0);
unsigned char result;
#define PID_DELTA 0.1

const int ADDR_KP = 0;
const int ADDR_KI = ADDR_KP + sizeof(double);
const int ADDR_KD = ADDR_KI + sizeof(double);
bool timeSet;

SabertoothSimplified ST;
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
// Balance PID controller Definitions
#define BALANCE_KP 17
#define BALANCE_KI 8
#define BALANCE_KD 0.5

#define BALANCE_SETPOINT 0

boolean live = false;

double input,out,setpoint;
double bal_kp,bal_ki,bal_kd;
PID pid(&input,&out,&setpoint,BALANCE_KP,BALANCE_KI,BALANCE_KD,DIRECT);

void init_imu() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(17);
    mpu.setYGyroOffset(-62);
    mpu.setZGyroOffset(32);
    mpu.setXAccelOffset(1417);
    mpu.setYAccelOffset(-250);
    mpu.setZAccelOffset(1525);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void set_pid_vals() {
    pid.SetTunings(bal_kp,bal_ki,bal_kd);
}

void write_EEPROM() {
    if (timeSet && EEPROM.isReady()) {
        EEPROM.updateDouble(ADDR_KP, bal_kp);
        EEPROM.updateDouble(ADDR_KI, bal_ki);
        EEPROM.updateDouble(ADDR_KD, bal_kd);
        timeSet = false;
    }
}

void init_pid() {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-127, 127);
    pid.SetSampleTime(10);

    setpoint = BALANCE_SETPOINT;

    bal_kp = EEPROM.readDouble(ADDR_KP);
    if (isnan(bal_kp)) bal_kp = BALANCE_KP;

    bal_ki = EEPROM.readDouble(ADDR_KI);
    if (isnan(bal_ki)) bal_ki = BALANCE_KI;

    bal_kd = EEPROM.readDouble(ADDR_KD);
    if (isnan(bal_kd)) bal_kd = BALANCE_KD;

    set_pid_vals();

}

void getvalues() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    }
    input = ypr[2] * 180/M_PI;
}

void motorspeed(int MotorAspeed) {
    ST.motor(1, MotorAspeed);
}

void new_pid() {
    pid.Compute();
    motorspeed(out);
}

void disp_pid() {

    lcd.setCursor(0, 0);
    lcd.print("   P    I    D");

    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.print(bal_kp, 1);
    lcd.print("  ");

    lcd.print(bal_ki, 1);
    lcd.print("  ");

    lcd.print(bal_kd, 1);

    uint8_t cursorPos = 0;
    if (activeInput == 0) {
       cursorPos = 2;
    } else if (activeInput == 1) {
        cursorPos = 7;
    } else if (activeInput == 2) {
        cursorPos = 12;
    }

    lcd.setCursor(cursorPos, 0);
    lcd.print('>');
}

void handle_pid_input() {
    pidButton.read();
    result = r.process();

    if (pidButton.wasReleased()) {
        activeInput++;
        if (activeInput > 2) activeInput = 0;
        timeSet = true;

    } else if (result) {
        double *toChange;

        switch (activeInput) {
            case 0:
                toChange = &bal_kp;
                break;
            case 1:
                toChange = &bal_ki;
                break;
            case 2:
                toChange = &bal_kd;
                break;
            default: return;
        }

        *toChange += result == DIR_CW ? PID_DELTA : - PID_DELTA;
        if (*toChange < 0) *toChange = 0;
        set_pid_vals();

    } else {
        return;
    }

    disp_pid();
}

void setup() {

    init_imu();
    SabertoothTXPinSerial.begin(9600);
    init_pid();

    lcd.begin(16, 2);
    disp_pid();
}

void loop() {
    handle_pid_input();
    write_EEPROM();
    getvalues();

    if (live) {
        new_pid();
    }

    else {
        if (abs(input - BALANCE_SETPOINT) < 4) {
            live = true;
        }
    }
}

