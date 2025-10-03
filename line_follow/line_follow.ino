

struct {
    float control;
    float integral;
    float error;
} ControllerReturn;

struct {
    int left;
    int right;
} SensorRead;

enum TuneState {NONE, LINEAR, ANGULAR, LEFT_BIAS, KP, KI, KD, CONTROL_MODE};

int MOTOR_LEFT_PIN = 1;
int MOTOR_RIGHT_PIN = 2;

// Distance between wheel centers, in meters
float TRACK_WIDTH = 0.1;
flaot MAX_SPEED = 0.3;

// State of waiting for tuning
int state = NONE;

// Current base variables
float linear_base = 0.15;
float angular_base = 0.5;
float left_bias = 0;
float kp = 1;
float ki = 0;
float kd = 0;

void setup() {
    // Initialize serial connection
    long baudRate = 9600; 
    Serial.begin(baudRate);

    Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
    Adafruit_DCMotor *motor_left = AFMS.getMotor(1);
    Adafruit_DCMotor *motor_left = AFMS.getMotor(1);
}

void loop() {
    // Check for serial input for tuning
    if (Serial.available() > 0) {

        if (state != NONE) {
            // state not NONE means we're waiting for tuning value
            tuningSetValue();
        } else {
            serial_read = Serial.parseInt();
            if (serial_read != NONE) {
                state = serial_read;
            }
        }
    }

    // If driving
    if (state == NONE) {
        SensorRead readings = readSensors();

        // placeholder
        if (readings.left > readings.right) {
            drive(linear_base,angular_base,left_bias);
        } else {
            drive(linear_base,-1 * angular_base,left_bias);
        }
        
    }
}

SensorRead readSensors() {
    SensorRead readings;
    return readings;
}

void tuningSetValue() {
    // TODO have variables as a struct and pass the address
    serial_read = Serial.parseFloat();
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

void drive(float linear, float angular, float left_motor_bias) {
    // Specified in meters/sec and rad/sec
    float vl = linear;
    float vr = linear;

    float vl -= angular * TRACK_WIDTH * 0.25;
    float vr += angular * TRACK_WIDTH * 0.25;

    // In case asymmetric weight / driving, added to adjust heading at zero angular
    float vl += left_motor_bias;

    if (vl < 0) {
        speed_l = (int)(-1 * vl / MAX_SPEED);
        motor_left->setSpeed(speed_l);
        motor_left->run(BACKWARD);
    } else {
        speed_l = (int)(vl / MAX_SPEED);
        motor_left->setSpeed(speed_l);
        motor_left->run(FORWARD);
    }

    if (vr < 0) {
        speed_r = (int)(-1 * vl / MAX_SPEED);
        motor_left->setSpeed(speed_l);
        motor_left->run(BACKWARD);
    } else {
        speed_l = (int)(vl / MAX_SPEED);
        motor_left->setSpeed(speed_l);
        motor_left->run(FORWARD);
    }
}

float bang_bang_control(SensorRead readings) {
    if (readings.left > 0) {
        
    }
    if (readings.right > 0) {

    }
}

float pid_control(float error,float kp,float ki,float kd,float integral,float previous_error,float dt) {
    float P = kp * error;

    float D = kd * (error - previous_error) / dt;

    integral = integral + error;

    float I = integral * ki * dt;

    ControllerReturn out;
    out.control = P + I + D;
    out.integral = integral;
    out.error = error;

    return out;
}