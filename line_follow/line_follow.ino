#include <Wire.h>
#include <Adafruit_MotorShield.h>

typedef struct {
    float control;
    float integral;
    float error;
} ControllerReturn;

typedef struct {
    int left;
    int right;
} SensorRead;

typedef struct {
    int cmd_left;
    int cmd_right;
} MotorCommands;

enum TuneState {NONE, LINEAR, ANGULAR, LEFT_BIAS, KP, KI, KD, CONTROL_MODE};

int MOTOR_LEFT_PIN = 1;
int MOTOR_RIGHT_PIN = 2;

int READING_LEFT_PIN = A0;
int READING_RIGHT_PIN = A1; 

// Distance between wheel centers, in meters
// float TRACK_WIDTH = 0.0129;
float TRACK_WIDTH = 1.0;
float ANGULAR_STRENGTH = 1.0;

// Maximum motor speed
int MAX_SPEED = 100;


// Difference between max sensor reading and min sensor reading
int SENSOR_OUT_RANGE = 400;

// State of waiting for tuning
int state = NONE;

// Current time
unsigned long current_time;

// Current base variables
float linear_base = 0.5;
float angular_base = 0.5;
float left_bias = 0;
float kp = 0.3;
float ki = 0;
float kd = 0;

float error = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor_left = AFMS.getMotor(MOTOR_LEFT_PIN);
Adafruit_DCMotor *motor_right = AFMS.getMotor(MOTOR_RIGHT_PIN);

void setup() {
    // Initialize serial connection
    long baudRate = 9600; 
    Serial.begin(baudRate);

    pinMode(READING_LEFT_PIN, INPUT);
    pinMode(READING_RIGHT_PIN, INPUT);

    AFMS.begin();
}

void loop() {
    // Motor, please spin
    // motor_left->setSpeed(40);
    // motor_left->run(FORWARD);
    // delay(1000);

    // Check for serial input for tuning
    /*
    if (Serial.available() > 0) {

        if (state != NONE) {
            cmds = drive(0.0,0.0,0.0)
            // state not NONE means we're waiting for tuning value
            tuningSetValue();
        } else {
            int serial_read = Serial.parseInt();
            if (serial_read != NONE) {
                state = serial_read;
            }
        }
    }
    */

    // If driving
    // Serial.println(state);
    //state == NONE;
    if (state == NONE) {
        SensorRead readings = readSensors();
        Serial.print("Readng left: ");
        Serial.print(readings.left);
        Serial.print("Readng right: ");
        Serial.println(readings.right);
        //SensorRead readings;
        //readings.left = 100;
        //readings.right = 300;

        // PID CONTROL
        unsigned long new_time = millis();
        float dt = (float)(new_time - current_time);
        current_time = new_time;
        float previous_error = 0.1;
        float integral = 0.0;
        ControllerReturn pid_out = pid_control(readings,kp,ki,kd,integral,previous_error,dt);

        // placeholder
        /*
        if (readings.left > readings.right) {
            drive(linear_base,angular_base,left_bias);
        } else {
            drive(linear_base,-1 * angular_base,left_bias);
        }
        */
    }
}

SensorRead readSensors() {
    SensorRead readings;

    readings.left = analogRead(READING_LEFT_PIN);
    readings.right = analogRead(READING_RIGHT_PIN);

    return readings;
}

void tuningSetValue() {
    // TODO have variables as a struct and pass the address
    float serial_read = Serial.parseFloat();
    // Update variable values
    switch (state) {
        case LINEAR:
            linear_base = serial_read;
            break;
        case ANGULAR:
            angular_base = serial_read;
            break;
        case LEFT_BIAS:
            left_bias = serial_read;
            break;
        case KP:
            kp = serial_read;
            break;
        case KI:
            ki = serial_read;
            break;
        case KD:
            kd = serial_read;
            break;
    }
    state = NONE;
}

MotorCommands drive(float linear, float angular, float left_motor_bias) {
    // Specified in meters/sec and rad/sec
    float vl = linear; // Between -1 and 1, 1 indicates max speed
    float vr = linear; // Between -1 and 1, 1 indicates max speed

    // Scaling here kept in case we determine numerical values for speed, rather than unitless
    // Currently equivalent to the same scaling of the PID coefficients
    vl -= angular * ANGULAR_STRENGTH;
    vr += angular * ANGULAR_STRENGTH;

    // In case asymmetric weight / driving, added to adjust heading at zero angular
    vl += left_motor_bias;

    // Motor speed takes an int from 0 to 255

    int speed_l;
    int speed_r;

    if (vl < 0) {
        speed_l = (int)(-1 * MAX_SPEED * vl);
        if (speed_l > MAX_SPEED) {speed_l = MAX_SPEED;}
        motor_left->setSpeed(speed_l);
        Serial.print("Speed L: +");
        Serial.println(speed_l);
        motor_left->run(FORWARD);
        //delay(100);
    } else {
        speed_l = (int)(MAX_SPEED * vl);
        if (speed_l > MAX_SPEED) {speed_l = MAX_SPEED;}
        motor_left->setSpeed(speed_l);
        Serial.print("Speed L: -");
        Serial.println(speed_l);
        motor_left->run(BACKWARD);
        //delay(100);
    }

    if (vr < 0) {
        speed_r = (int)(-1 * MAX_SPEED * vr);
        if (speed_r > MAX_SPEED) {speed_r = MAX_SPEED;}
        motor_right->setSpeed(speed_r);
        Serial.print("Speed R: +");
        Serial.println(speed_r);
        motor_right->run(BACKWARD);
        //delay(100);
    } else {
        speed_r = (int)(MAX_SPEED * vr);
        if (speed_r > MAX_SPEED) {speed_r = MAX_SPEED;}
        motor_right->setSpeed(speed_r);
        Serial.print("Speed R: -");
        Serial.println(speed_r);
        motor_right->run(FORWARD);
        //delay(100);
    }

    MotorCommands cmds = {speed_l,speed_r};
    return cmds;
}

float find_error(SensorRead readings) {

    // Take difference and constrain to +/- 400 range
    int diff = readings.left - readings.right;
    if (diff > SENSOR_OUT_RANGE) {
        diff = SENSOR_OUT_RANGE;
    } else if (diff < -1 * SENSOR_OUT_RANGE) {
        diff = -1 * SENSOR_OUT_RANGE;
    }
    //float rl = (float)readings.left / 1023.0;
    //float rr = (float)readings.right / 1023.0;
    float err = -1 * diff / (float)(SENSOR_OUT_RANGE);
    Serial.println("\n\n\nerr: ");
    Serial.println(err);
    return;
}

void bang_bang_control(SensorRead readings) {
    int tolerance = 100;
    MotorCommands cmds;
    if (readings.left - tolerance > readings.right) {
        // Right sensor is giving lower voltage; it sees tape
        cmds = drive(linear_base,angular_base,left_bias);
    }
    if (readings.right - tolerance > readings.left) {
        cmds = drive(linear_base,-1*angular_base,left_bias);
    }
    log_over_serial(readings,cmds.cmd_left,cmds.cmd_right);
}

ControllerReturn pid_control(SensorRead readings,float kp,float ki,float kd,float integral,float previous_error,float dt) {
    
    float error = find_error(readings); // -1 to 1

    float P = kp * error;
    float D = kd * (error - previous_error) / dt;
    integral = integral + error;
    float I = integral * ki * dt;

    ControllerReturn out;
    out.control = P + I + D;
    out.integral = integral;
    out.error = error;

    MotorCommands cmds = drive(linear_base,out.control,left_bias);
    log_over_serial(readings,cmds.cmd_left,cmds.cmd_right);
    return out;
}

void log_over_serial(SensorRead readings, int speed_left, int speed_right, float kp, float ki, float kd) {
    Serial.print(readings.left);
    Serial.print(",");
    Serial.print(readings.right);
    Serial.print(",");
    Serial.print(speed_left);
    Serial.print(",");
    Serial.print(speed_right);
    Serial.print(",");
    Serial.print(kp);
    Serial.print(",");
    Serial.print(ki);
    Serial.print(",");
    Serial.print(kd);
    Serial.print(",");
    Serial.println(millis())
}