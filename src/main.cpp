/*
speed 50 KP 0.0065 KI 0.00000015 KD 0.1375 Output 50
speed 85 KP 0.00562 KI 0.0000022 KD 0.1625 Output 75

speed 80 KP 0.01892 KI 0.00000022 KD 0.1825 Output 70


*/
#include <Arduino.h>
#include <IRremote.h>
// #include <WiFi.h>
// #include <WiFiServer.h>

#include "LF_SData.h"
#include "MotorDriver.h"
// #include "arduino_secrets.h"
// #include "WebServerHandler.h"

#define DEBUG_FLAG false
#define SETUP_WIFI false
#define CALIBRATION_FLAG false

// WiFiServer server(80);

LF_SData sensorData;

/*
    -- sensor pins --
*/
#define S0 D0  // Multiplexer select pin S0
#define S1 D1   // Multiplexer select pin S1
#define S2 D2   // Multiplexer select pin S2
#define S3 D3   // Multiplexer select pin S3
#define SIG A6   // Analog signal pin from multiplexer

#define M1DIR D7  // GPIO
#define M1PWM D6  // PWM 
#define M2DIR D5  // GPIO
#define M2PWM D4  // PWM 
MotorDriver motors(M1DIR, M1PWM, M2DIR, M2PWM);

#define IR_PIN D8

unsigned long startTime = 0;
unsigned long accelerationTime = 3000; // 5 seconds

bool robotActive = false;

double MAX_OUTPUT = 70;
double setBaseSpeed = 80;


double KP = 0.00605; // todo maybe a bit lower 0.00432
double KI = 0.00000022; // todo play 0.00000023
double KD = 0.1625; //todo increase 0.326

const float alpha = 0.88; // Lower = smoother but slower response
double smoothed_error = 0;

double last_derivative = 0;
const float dAlpha = 0.90;

double integral = 0;
double previousError = 0;

double BASE_SPEED = 0;

enum TurnType { TurnNone, TurnLeft, TurnRight };
bool inSharpTurn = false;
unsigned long sharpTurnStart = 0;
TurnType currentTurn = TurnNone;
const unsigned long sharpReverseMs = 6; // ~6 ms pivot phase
TurnType lastTurnDirection = TurnNone; // remembered direction to rotate when line is lost

void start_stop() {
    // IR handling
    if (IrReceiver.decode()) {
        uint8_t cmd = IrReceiver.decodedIRData.command;

        if (cmd == 0x45) {
            robotActive = true;
        } else if (cmd == 0x46) {
            robotActive = false;
        } else if (cmd == 0x47) {
            robotActive = true;
        } 

        IrReceiver.resume(); // ready to receive the next command
    }
}

double PID(double error) {
    integral += error;

    double raw_derivative = error - previousError;
    last_derivative = dAlpha * raw_derivative + (1 - dAlpha) * last_derivative;
    // last_derivative = raw_derivative;

    previousError = error;

    double correction = (KP * error) + (KI * integral) + (KD * last_derivative);
    return correction;
}

void setup() {
    // Serial.begin(115200);
    motors.begin();

    sensorData.setupLineSensors(S0, S1, S2, S3, SIG);

    if (CALIBRATION_FLAG) {
        // Calibration mode
        unsigned long startTime = millis();
        unsigned long calibrationTime = 15000;  // 15 seconds

        Serial.begin(115200);
        while(!Serial) {
            ; // wait for serial port to connect. Needed for native USB port only
        }

        delay(5000);
    
        Serial.println("Starting calibration...");
    
        while (millis() - startTime < calibrationTime) {
          sensorData.calibrateSensors(true);
        }

        Serial.println("Calibration complete.");
    } else {
        sensorData.calibrateSensors(false);
    }

    IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK); // Init IR receiver
    
    // if (WiFi.beginAP(SECRET_SSID, SECRET_PASS) != WL_AP_LISTENING) {
    //     if (DEBUG_FLAG) Serial.println("Failed to start AP");
    //     while (true); // halt
    // }

    // server.begin();

    if (DEBUG_FLAG) {
        Serial.println("Access Point started");
        Serial.print("IP Address: ");
        // Serial.println(WiFi.localIP()); // This will usually be 192.168.3.1
    }
}

void loop() {
    // handleWeb();
    start_stop();

    if (startTime == 0) {
        startTime = millis();
    }

    unsigned long elapsedTime = millis() - startTime;
    if (elapsedTime <= accelerationTime) {
        BASE_SPEED = map(elapsedTime, 0, accelerationTime, 0, setBaseSpeed);
    }

    if (true) {
        sensorData.getLiveSerialPrint(true);
    }

    int line_value = sensorData.getLinePosition();
    int raw_error = 7250 - line_value;
    smoothed_error = alpha * raw_error + (1 - alpha) * smoothed_error;
    // smoothed_error = raw_error;
    double pid_output = constrain(PID(smoothed_error), -MAX_OUTPUT, MAX_OUTPUT);


    // if (true) {
    //     Serial.print("Sensor Error: ");
    //     Serial.println(smoothed_error);
    //     Serial.print("PID: ");
    //     Serial.println(pid_output);
    //     Serial.print("active?: ");
    //     Serial.println(robotActive);
    // }

    double left = BASE_SPEED + pid_output;
    double right = BASE_SPEED - pid_output;

    if ((line_value > 2000 && smoothed_error <= 4000) || (smoothed_error >= 11000 && smoothed_error < 13000)) {  // Medium turn
        BASE_SPEED = setBaseSpeed * 0.80; // maybe 0.85
    } else {                     // Straight line
        BASE_SPEED = setBaseSpeed;  // Restore full speed
    }

    if (robotActive) {
        // Determine if line is lost (all white/black). Adjust thresholds as needed.
        bool lineLost = (line_value <= 200) || (line_value >= 14800);

        // Update last turn direction based on current signals when not in sharp-turn
        if (!inSharpTurn) {
            if (smoothed_error > 0) {
                lastTurnDirection = TurnLeft;  // need to steer left
            } else if (smoothed_error < 0) {
                lastTurnDirection = TurnRight; // need to steer right
            }
        }

        // If line is lost, rotate in place in the last known direction until line is found
        if (lineLost) {
            if (lastTurnDirection == TurnLeft) {
                motors.setMotor1Speed(80);
                motors.setMotor2Speed(-80);
            } else if (lastTurnDirection == TurnRight) {
                motors.setMotor1Speed(-80);
                motors.setMotor2Speed(80);
            } else {
                // Default fallback if no history: rotate left
                motors.setMotor1Speed(80);
                motors.setMotor2Speed(-80);
            }
            // Skip the rest of control for this cycle while searching
            delay(5);
            return;
        }

        // Detect near-90° sharp turn entry; use extreme line_value thresholds
        if (!inSharpTurn) {
            if (line_value <= 1500) {
                inSharpTurn = true;
                sharpTurnStart = millis();
                currentTurn = TurnLeft;   // pivot left: left fwd, right rev
                lastTurnDirection = TurnLeft;
            } else if (line_value >= 14500) {
                inSharpTurn = true;
                sharpTurnStart = millis();
                currentTurn = TurnRight;  // pivot right: left rev, right fwd
                lastTurnDirection = TurnRight;
            }
        }

        if (inSharpTurn) {
            unsigned long since = millis() - sharpTurnStart;
            if (since <= sharpReverseMs) {
                // Phase 1 (~6 ms): one motor forward, the other reverse (reference values)
                if (currentTurn == TurnLeft) {
                    motors.setMotor1Speed(80);   // left forward
                    motors.setMotor2Speed(-80);  // right reverse
                } else if (currentTurn == TurnRight) {
                    motors.setMotor1Speed(-80);  // left reverse
                    motors.setMotor2Speed(80);   // right forward
                }
            } else {
                // Phase 2: stop the motor that was reversing, keep the other forward
                if (currentTurn == TurnLeft) {
                    motors.setMotor1Speed(80);   // keep forward
                    motors.setMotor2Speed(0);    // stop reverse side
                } else if (currentTurn == TurnRight) {
                    motors.setMotor1Speed(0);    // stop reverse side
                    motors.setMotor2Speed(80);   // keep forward
                }
                // Exit sharp-turn after applying phase 2 for this cycle
                inSharpTurn = false;
                currentTurn = TurnNone;
            }
        } else {
            // FOR CLASSIC (PID driven)
            motors.setMotor1Speed(left);
            motors.setMotor2Speed(right);
        }

    } else {
        motors.setMotor1Speed(0);
        motors.setMotor2Speed(0);

        startTime = 0;
    }

    // if (DEBUG_FLAG) {
    //     Serial.print("left = ");
    //     Serial.println(left);

    //     Serial.print("right = ");
    //     Serial.println(right);
    // }

    delay(5);
    // delay(1000);
}