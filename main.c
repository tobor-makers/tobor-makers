#pragma config(Sensor, S1,     SENSOR_L,       sensorLightActive)
#pragma config(Sensor, S2,     SENSOR_S,       sensorSONAR)
#pragma config(Sensor, S3,     SENSOR_R,       sensorColorNxtRED)
#pragma config(Motor,  motorA,          MOTOR_EYES,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          MOTOR_R,       tmotorNXT, PIDControl, driveRight, encoder)
#pragma config(Motor,  motorC,          MOTOR_L,       tmotorNXT, PIDControl, driveLeft, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(NXT)


/**
 * @file
 * @mainpage Author Information
 *  Tobor Makers (<a href="https://github.com/tobor-makers/tobor-makers">Tobor Maker</a>) \n
 *  12/04/2017 \n
 *  Hogeschool Utrecht - Jaar 1 - Blok 3 - RobotC
 * @section program_name Program name
 *  Tobor the robot
*/


/*
* Enums.
*/

typedef enum {
    CROSSROAD = 0,
    INIT = 1,
    MOVING = 2,
    RIP = 3,
    STOPPED = 4
} StateEnum;


/*
* Globals.
*/

// This is is minimum distance an object needs to be in order to be detected.
const int VIEW_DIST = 20;
const int THRESHOLD_L = 50; // 50
const int THRESHOLD_R = 30; // 30
// STARTING STATE of the robot
StateEnum State = INIT;
// Mutes annoying sound
bool Muted = true;

// Bluetooth gobals.
const int MAX_SIZE_OF_MESSAGE = 30;
const int INBOX = 5;


// PID globals.
// See https://punpun.xyz/f747.pdf for more information about PID.

// black* and white* are the blackest and whitest values the sensors can see.
// Do not edit these, these values will be used (and set) by initState().
int BlackL = 100;
int BlackR = 100;
int WhiteL = 0;
int WhiteR = 0;

// offset is the avarage of the light-sensor measurements of "total white" and "total black".
// Do not edit these, these values will be used (and set) by initState().
int OffsetL = 0;
int OffsetR = 0;

// kp (the konstant for the proportional controller) is calculated using `0.60*kc`,
// kc (critical gain) being a value where the robot follows the line and gives noticeable
// oscillation but not really a wild one.
//
// his controls how fast the controller will try to get back to the line edge when it has drifted away from it,
// however a low KP value will increase oscillation.
const int KP = 80;

// The konstant integral, is calculated using (2*KP*DT/PC).
// TODO: Find a nice value for this maybe?
const int KI = 0;

// The konstant derivative, is calculated using (KP*PC/(8*DT)).
const int KD = 1500;

// tp (target power) is the power level of both motors when the robot is supposed to go
// straight ahead, which it does when the error has a value of 0.
const int TP = 35;

// integral is the running sum of the error.
// Do not edit this, this value will be used (and set) by moveState().
int Integral = 0;

// derivative is the current error minus the previous error.
// Do not edit this, this value will be used (and set) by moveState().
int Derivative = 0;

// These two should be fairly self-explanatory.
// Again, do not edit these, these values will bw used (and set) by moveState().
int LastError = 0;
int Turn = 0;


/*
* Functions.
*/

/**
* @brief Checks for bluetooth messages and sets command string
* @param command String to hold the bluetooth message
* @authors Nick Bout, Wouter Dijkstra
*/
void checkBluetoothMessage(string &command) {
    int nSizeOfMessage;
    TFileIOResult nBTCmdRdErrorStatus;
    ubyte nRcvBuffer[MAX_SIZE_OF_MESSAGE];
    nSizeOfMessage = cCmdMessageGetSize(INBOX);

    if (nSizeOfMessage > MAX_SIZE_OF_MESSAGE)
        nSizeOfMessage = MAX_SIZE_OF_MESSAGE;

    if (nSizeOfMessage > 0) {
        nBTCmdRdErrorStatus = cCmdMessageRead(nRcvBuffer, nSizeOfMessage, INBOX);
        nRcvBuffer[nSizeOfMessage] = '\0';
        stringFromChars(command, (char *)nRcvBuffer);
    }
}

/**
* @brief Checks for crossroads
* @return If Robot is on crossroad
* @authors Eelke Feitz
*/
bool checkCrossroad() {
    return (SensorValue[SENSOR_L] < (OffsetL + 5) && SensorValue[SENSOR_R] < (OffsetR+ 5));
}

/**
* @brief Plays sound
* @authors Wouter Dijkstra, Noortje Metsemakers
*/
void makeSound(){
    if (!Muted && !bSoundActive)
        playSound(soundUpwardTones);
    else if (Muted && bSoundActive)
        ClearSounds();
}

/**
* @brief Move state, moves robot etc
*
* Move state does the following:
* 	- Uses checkBluetoothMessage function to get command
*   - Sets robot state to STOPPED when "FIRE" command is given
*		- Sets robot state to STOPPED when there is a object
*		- Plays sound with the makeSound function
*		- Sets motor speed to MAX_SPEED
*
* @authors Nick Bout, Wouter Dijkstra, Eelke Feitz, Noortje Metsemakers, Camille Scholtz
*/
void moveState() {
    // Get command from app.
    string command = "";
    checkBluetoothMessage(command);

    // Mute robot
    if (command == "B") Muted = !Muted;

    // Kill robot
    if (command == "C") {
        State = RIP;
        return;
    }

    // Startup bot when the fire button is pressed on the Android app.
    if (command == "FIRE") {
        State = STOPPED;
        return;
    }

    // If we see a object.
    if (SensorValue[SENSOR_S] < VIEW_DIST) {
        State = STOPPED;
        return;
    }

    if (checkCrossroad()) {
        State = CROSSROAD;
        return;
    }

    // Play sound.
    makeSound();

    // Line follow using PID.
    int error = (SensorValue[SENSOR_L]-OffsetL) - (SensorValue[SENSOR_R]-OffsetR);
    //int error = (SensorValue[SENSOR_R]-OffsetR) - (SensorValue[SENSOR_L]-OffsetL);
    Integral += error;
    Derivative = error - LastError;

    Turn = (KP*error) + (KI*Integral) + (KD*Derivative);
    Turn = round(Turn / 100);

    LastError = error;

    // Actually alter motor speed.
    motor[MOTOR_L] = TP - Turn;
    motor[MOTOR_R] = TP + Turn;
}

/**
* @brief Reduce motor speed of robot slowly
* @authors Nick Bout
*/
void slowBreak() {
    int speed = ((motor[MOTOR_L] > motor[MOTOR_R]) ? motor[MOTOR_L] : motor[MOTOR_R]) / 2;

    for (int i = speed; i > 0; i--) {
        motor[MOTOR_L] = i;
        motor[MOTOR_R] = i;
        wait1Msec(20);
    }
    motor[MOTOR_L]  = 0;
    motor[MOTOR_R] = 0;
}

/**
*	@brief Turn face in degrees
* @param degrees The amount of degrees the face needs to turn.
* @authors Nick Bout
*/
void turnFaceDegrees(int degrees) {
    nMotorEncoder[MOTOR_EYES] = 0;
    nMotorEncoderTarget[MOTOR_EYES] = degrees;
    motor[MOTOR_EYES] = ((degrees > 0) ? 20 : -20);

    while(nMotorRunState[MOTOR_EYES] != runStateIdle) {}
    motor[MOTOR_EYES] = 0;
}

/**
* @brief Turn robot in degrees
* @param degrees The amount of degrees the robot needs to turn.
* @authors Nick Bout
*/
void turnRobotDegrees(int degrees) {
    degrees *= 2;
    nMotorEncoder[MOTOR_L] = 0;
    nMotorEncoderTarget[MOTOR_L] = degrees;
    motor[MOTOR_L] = ((degrees > 0) ? 20 : -20);
    motor[MOTOR_R] = ((degrees > 0) ? -20 : 20);

    while(nMotorRunState[MOTOR_L] != runStateIdle) {
      // Do not continue.
    }
    motor[MOTOR_L] = 0;
    motor[MOTOR_R] = 0;
}

/**
*	@brief Move around object, then drive back to line
*
* @authors Nick Bout
*/
void moveAroundObject() {
    const int robotLength = 360; // Robot length ~
    // Turn eyes 90 degrees
    turnFaceDegrees(-90); // Look right
    // Turn robot 90 degrees
    turnRobotDegrees(90);

    // Do this 2 times to drive around
    int num = 2;
    for (int i = 0; i < num; ++i) {
        // Move until not seeing anyting
        motor[MOTOR_L] = 20;
        motor[MOTOR_R] = 20;
        while (SensorValue[SENSOR_S] < 50) {
            // Follow object
            if (SensorValue[SENSOR_S] < 15) {
                motor[MOTOR_L] = 25;
                motor[MOTOR_R] = 20;
            } else if (SensorValue[SENSOR_S] >= 15 && SensorValue[SENSOR_S] <= 20) {
                motor[MOTOR_L] = 20;
                motor[MOTOR_R] = 20;
            } else if (SensorValue[SENSOR_S] > 24) {
                motor[MOTOR_L] = 20;
                motor[MOTOR_R] = 25;
            }
        }

        // Move full robot past object
        nMotorEncoder[MOTOR_L] = 0;
        nMotorEncoderTarget[MOTOR_L] = robotLength;
        motor[MOTOR_L] = 20;
        motor[MOTOR_R] = 20;
        while(nMotorRunState[MOTOR_L] != runStateIdle) {}
        slowBreak();
        // Turn -90 degrees.
        turnRobotDegrees(-90);

        if (i != num - 1) {
            // Move full robot to object.
            nMotorEncoder[MOTOR_L] = 0;
            nMotorEncoderTarget[MOTOR_L] = robotLength;
            motor[MOTOR_L] = 20;
            motor[MOTOR_R] = 20;
            while(nMotorRunState[MOTOR_L] != runStateIdle) {}
        }
    }
    slowBreak();
    // Move Eyes back.
    turnFaceDegrees(90); // Turn face forward again.

    // Go back to line.
    motor[MOTOR_L] = TP;
    motor[MOTOR_R] = TP;
    while (SensorValue[SENSOR_L] > THRESHOLD_L && SensorValue[SENSOR_R] > THRESHOLD_R) {}
    slowBreak();
}

/**
* @brief Turn robot left till line is found
*
* @authors Wouter Dijkstra
*/
void turnLeft () {
    motor[MOTOR_L] = -15;
    motor[MOTOR_R] = 15;
    while (SensorValue[SENSOR_R] >=THRESHOLD_R) {
        wait1Msec(1);
    }
    motor[MOTOR_L] = 0;
    motor[MOTOR_R] = 0;

    wait1Msec(50);
}

/**
* @brief Turn robot right till line is found
* @authors Wouter Dijkstra
*/
void turnRight() {
    motor[MOTOR_L] = 15;
    motor[MOTOR_R] = -15;
    while (SensorValue[SENSOR_L] >= THRESHOLD_L) {
        wait1Msec(1);
    }
    motor[MOTOR_L] = 0;
    motor[MOTOR_R] = 0;

    wait1Msec(50);
}

/**
* @brief Crossroad state, Waits at crossroad until command is given
*
* Crossroad does the following:
* 	- Uses checkBluetoothMessage function to get command
*		- Goes in direction command tells it to go
* 	- Sets state to MOVING when line is found
*
* @authors Nick Bout, Wouter Dijkstra
*/
void crossroadState() {
    string command = "";
    checkBluetoothMessage(command);

    if (command == "A") {
        State = MOVING;
        return;
    }

    if (command == "LEFT") {
        turnLeft();
        State = MOVING;
        return;
    } else if (command == "RIGHT") {
        turnRight();
        State = MOVING;
        return;
    } else if (command == "UP") {
   	    // Drive a litle bit forward
        nMotorEncoder[MOTOR_L] = 0;
        nMotorEncoderTarget[MOTOR_L] = 50;
        motor[MOTOR_L] = 20;
        motor[MOTOR_R] = 20;
        while(nMotorRunState[MOTOR_L] != runStateIdle) {} // drive until 50 degree
        motor[MOTOR_L] = 0;
        motor[MOTOR_R] = 0;

        State = MOVING;
        return;
  	} else if (command == "FIRE") {
        // Choose a random direction.
        int ran = random(2);
        if (ran == 0)
        	turnRight();
        else
        	turnLeft();

        State = MOVING;
        return;
    }

    // Stop motors if not stopped already.
    if (motor[MOTOR_L] > 0 || motor[MOTOR_R] > 0) {
        slowBreak();
    }
}

/**
* @brief Initialises robot
*
* initState does the following:
* 	- Calibrates L and R offset for PID line follow
* 	- Sets robot in STOPPED state
*
* @authors Noortje Metsemakers, Camille Scholtz
*/
void initState() {
    // tp (target power) is the power level of the motors.
    const int tp = 10;

    motor[MOTOR_L] = tp;
    motor[MOTOR_R] = -tp;

    while (SensorValue[SENSOR_R] == 0) {}
    for (int i=0; i<7500; i++) {
            if (SensorValue[SENSOR_L] > WhiteL) {
                WhiteL = SensorValue[SENSOR_L];
            } else if (SensorValue[SENSOR_L] <= BlackL) {
                BlackL = SensorValue[SENSOR_L];
            }

            if (SensorValue[SENSOR_R] > BlackR) {
                WhiteR = SensorValue[SENSOR_R];
            } else if (SensorValue[SENSOR_R] <= BlackR) {
                BlackR = SensorValue[SENSOR_R];
            }

          wait1Msec(1);
    }

    motor[MOTOR_L] = 0;
    motor[MOTOR_R] = 0;

    OffsetL = (BlackL+WhiteL)/2;
    OffsetR = (BlackR+WhiteR)/2;

    State = STOPPED;
}

/**
* @brief Stop state, stops the robot etc
*
* Move state does the following:
* 	- Uses checkBluetoothMessage function to get command
* 	- Sets robot state to MOVING when "FIRE" command is given
*		- uses slowBreak function when motor speed > 0
*
* @authors Nick Bout
*/
void stopState() {
    // Get command from app
    string command = "";
    checkBluetoothMessage(command);

    // Startup bot when fire button is pressed on the app
    if (command == "FIRE") {
        State = MOVING;
        return;
    } else if (command == "A") {
        moveAroundObject();
        turnRight();
        State = MOVING;
        return;
    }

    // Stop motors if not stopped already
    if (motor[MOTOR_L] > 0 || motor[MOTOR_R] > 0) {
        slowBreak();
    }
}


/*
* Main loop.
*/

/**
* @brief Main, Starts main loop and calls functions based on STATE
* @authors Nick Bout
*/
task main() {
    bool running = true;

    while(running) {
        switch(State) {
            case CROSSROAD:
                displayTextLine(0, "Crossroad");
                crossroadState();
                break;
            case INIT:
                displayTextLine(0, "Init");
                initState();
                break;
            case MOVING:
                displayTextLine(0, "Moving");
                moveState();
                break;
            case RIP:
                displayBigTextLine(0, "Rip");
                running = false;
                break;
            case STOPPED:
                displayBigTextLine(0, "Stopped");
                stopState();
                break;
            default:
                displayBigTextLine(0, "No state");
        }

        wait1Msec(1);
    }
}
