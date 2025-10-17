#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Good parameters
// 0.6 kp, 0.0 ki, 0.9 kd, 40 max speed

// Return type of PID controller
typedef struct {
    float control;
    float integral;
    float error;
} ControllerReturn;

// Return type of sensor read function
typedef struct {
    int left;
    int right;
} SensorRead;

// Return type of motor command function
typedef struct {
    int cmd_left;
    int cmd_right;
} MotorCommands;

enum TuneState {NONE, LINEAR, ANGULAR, LEFT_BIAS, KP, KI, KD, CONTROL_MODE};

int MOTOR_LEFT_PIN = 1;
int MOTOR_RIGHT_PIN = 2;

int READING_LEFT_PIN = A0;
int READING_RIGHT_PIN = A1; 

// Scales angular command passed to drive
float ANGULAR_STRENGTH = 0.7;

// Maximum motor speed
int MAX_SPEED = 80;

// Minimum motor speed
// Nonzero values help prevent low speed commands locking up motor
int MIN_SPEED = 0;

// Difference between max sensor reading and min sensor reading
int SENSOR_OUT_RANGE = 400;

// State of waiting for tuning
int state = NONE;

// Current time
unsigned long current_time;

// Current base variables
float linear_base = 0.3;
float angular_base = 0.5;
float left_bias = 0;
float kp = 0.6;
float ki = 0.0;
float kd = 1.5;

float previous_error = 0;
float error_hist[5] = {0.0,0.0,0.0,0.0,0.0};

// Initialize motor shield and motors
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
    current_time = millis();
}

void loop() {
    // Check for serial input for tuning
    
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) {
            return; // Ignore empty tuning commands
        }

        if (state != NONE) { // If currently driving
            MotorCommands cmds = drive(0.0,0.0,0.0); // Pause while waiting for tuning
            // state not NONE means we're waiting for tuning value
            tuningSetValue(line.toFloat());
        } else {
            // next variable to tune is stored in our state as an int
            int serial_read = line.toInt();
            if (serial_read != NONE) {
                state = serial_read;
            }
        }
    }
    

    // If driving
    if (state == NONE) {
        SensorRead readings = readSensors();

        // Print values for manual logging
        Serial.print("Reading left: ");
        Serial.print(readings.left);
        Serial.print("Reading right: ");
        Serial.println(readings.right);

        // PID CONTROL
        unsigned long new_time = millis();
        float dt = (float)(new_time - current_time);
        current_time = new_time;

        // calculate integral error to pass in
        // more correct calculation would account for different timestep durations
        // however, we can assume constant iteration time for simplicity's sake
        float sum_integral = 0.0;
        for (int i = 0; i < 5; ++i) {
            sum_integral += error_hist[i];
        }
        sum_integral /= 5;

        // Pass values to controller
        ControllerReturn pid_out = pid_control(readings,kp,ki,kd,sum_integral,previous_error,dt);
        previous_error = pid_out.error;

        // Remake integral to reflect previous terms
        for (int i = 0; i < 4; ++i) {
            error_hist[i] = error_hist[i+1];
        }
        error_hist[4] = pid_out.error;
    }
}

SensorRead readSensors() {
    // Read sensor values and return a struct containing both
    SensorRead readings;
    readings.left = analogRead(READING_LEFT_PIN);
    readings.right = analogRead(READING_RIGHT_PIN);
    return readings;
}

void tuningSetValue(float val) {
    // Update variable values
    // Variable to adjust is specified by state variable
    switch (state) {
        case LINEAR:
            linear_base = val;
            break;
        case ANGULAR:
            angular_base = val;
            break;
        case LEFT_BIAS:
            left_bias = val;
            break;
        case KP:
            kp = val;
            break;
        case KI:
            ki = val;
            break;
        case KD:
            kd = val;
            break;
    }
    // Reset to NONE state indicating driving, not waiting for tuning
    state = NONE;
}

MotorCommands drive(float linear, float angular, float left_motor_bias) {
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

    // Scale motor commands to +/- MAX_SPEED
    // If backwards, negate the speed to be positive and call the correct motor direction
    if (vl < 0) {
        speed_l = (int)(-1 * MAX_SPEED * vl);
        if (speed_l > MAX_SPEED) {speed_l = MAX_SPEED;}
        if (speed_l < MIN_SPEED) {speed_l = MIN_SPEED;}
        motor_left->setSpeed(speed_l);
        Serial.print("Speed L: +");
        Serial.println(speed_l);
        motor_left->run(FORWARD);
    } else {
        speed_l = (int)(MAX_SPEED * vl);
        if (speed_l > MAX_SPEED) {speed_l = MAX_SPEED;}
        if (speed_l < MIN_SPEED) {speed_l = MIN_SPEED;}
        motor_left->setSpeed(speed_l);
        Serial.print("Speed L: -");
        Serial.println(speed_l);
        motor_left->run(BACKWARD);
    }

    if (vr < 0) {
        speed_r = (int)(-1 * MAX_SPEED * vr);
        if (speed_r > MAX_SPEED) {speed_r = MAX_SPEED;}
        if (speed_r < MIN_SPEED) {speed_r = MIN_SPEED;}
        motor_right->setSpeed(speed_r);
        Serial.print("Speed R: +");
        Serial.println(speed_r);
        motor_right->run(BACKWARD);
    } else {
        speed_r = (int)(MAX_SPEED * vr);
        if (speed_r > MAX_SPEED) {speed_r = MAX_SPEED;}
        if (speed_r < MIN_SPEED) {speed_r = MIN_SPEED;}
        motor_right->setSpeed(speed_r);
        Serial.print("Speed R: -");
        Serial.println(speed_r);
        motor_right->run(FORWARD);
    }

    MotorCommands cmds = {speed_l,speed_r};
    return cmds; // Return commands for logging purposes
}

float find_error(SensorRead readings) {
    // Take difference and constrain to +/- output range
    int diff = readings.left - readings.right;
    if (diff > SENSOR_OUT_RANGE) {
        diff = SENSOR_OUT_RANGE;
    } else if (diff < -1 * SENSOR_OUT_RANGE) {
        diff = -1 * SENSOR_OUT_RANGE;
    }
    // Divide by output range to normalize between +/- 1
    float err = -1 * diff / (float)(SENSOR_OUT_RANGE);
    return err;
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
    log_over_serial(readings,cmds.cmd_left,cmds.cmd_right,kp,ki,kd);
}

ControllerReturn pid_control(SensorRead readings,float kp,float ki,float kd,float integral,float previous_error,float dt) {
    float error = find_error(readings); // -1 to 1

    // Calculate control terms from error
    float P = kp * error;
    float D = kd * (error - previous_error) / dt;
    float I = integral * ki * dt;

    ControllerReturn out;
    // Calculate control to pass as angular argument to drive
    out.control = P + I + D;
    out.integral = integral;
    out.error = error;

    MotorCommands cmds = drive(linear_base,out.control,left_bias); // pass to motor drive command
    log_over_serial(readings,cmds.cmd_left,cmds.cmd_right,kp,ki,kd); // log for visualization
    return out;
}

void log_over_serial(SensorRead readings, int speed_left, int speed_right, float kp, float ki, float kd) {
    // Log all expected values for visualization over the serial connection
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
    Serial.println(millis());
}