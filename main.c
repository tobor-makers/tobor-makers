#pragma config(Sensor, S1,     SENSOR_L,     sensorLightActive)
#pragma config(Sensor, S2,     SENSOR_S, sensorSONAR)
#pragma config(Sensor, S3,     SENSOR_R,     sensorColorNxtRED)
#pragma config(Motor,  motorB, MOTOR_R,      tmotorNXT, PIDControl, driveRight, encoder)
#pragma config(Motor,  motorC, MOTOR_L,      tmotorNXT, PIDControl, driveLeft,  encoder)
#pragma platform(NXT)


/*
* Enums.
*/

typedef enum {
	CROSSROAD = 0,
	INIT = 1,
	MOVING = 2,
	RIP = 3,
	STOPPED = 4
} State;


/*
* Globals.
*/

const int VIEW_DIST = 25;
const int TRESHHOLD_L = 40; // 50
const int TRESHHOLD_R = 20; // 30
State STATE = INIT;

// Bluetooth gobals.
const int MAX_SIZE_OF_MESSAGE = 30;
const int INBOX = 5;


// PID globals.
// See https://punpun.xyz/f747.pdf for more information about PID.

// black* and white* are the blackest and whitest values the sensors can see.
// Do not edit these, these values will be editied (and are used by) initState.
int BLACK_L = 100;
int BLACK_R = 100;
int WHITE_L = 0;
int WHITE_R = 0;

// offset is the avarage of the light-sensor measurements of "total white" and "total black".
// Do not edit these, these values will be editied (and are used by) initState.
int OFFSET_L = 0;
int OFFSET_R = 0;

// kp (the konstant for the proportional controller) is calculated using `0.60*kc`,
// kc (critical gain) being a value where the robot follows the line and gives noticeable
// oscillation but not really a wild one.
const int KP = 1;
const int KI = 0.001;
const int KD = 15;

// tp (target power) is the power level of both motors when the robot is supposed to go
// straight ahead, which it does when the error has a value of 0.
const int TP = 35;

// integral is the running sum of the error.
int INTEGRAL = 0;

// derivative is the current error minus the previous error.
int DERIVATIVE = 0;

// These two should be fairly self-explanatory.
// Again
int LAST_ERROR = 0;
int TURN = 0;


/*
* Functions.
*/

/**
* @brief Checks for bluetooth messages and sets command string
* @param command String to hold the bluetooth message
*/
void checkBluetoothMessage(string &command) {
	int nSizeOfMessage;
	TFileIOResult nBTCmdRdErrorStatus;
	ubyte nRcvBuffer[MAX_SIZE_OF_MESSAGE];
	nSizeOfMessage = cCmdMessageGetSize(INBOX);

	if (nSizeOfMessage > MAX_SIZE_OF_MESSAGE)
		nSizeOfMessage = MAX_SIZE_OF_MESSAGE;

	if (nSizeOfMessage > 0){
		nBTCmdRdErrorStatus = cCmdMessageRead(nRcvBuffer, nSizeOfMessage, INBOX);
		nRcvBuffer[nSizeOfMessage] = '\0';
		stringFromChars(command, (char *)nRcvBuffer);
	}
}

/**
* @brief Checks for crossroads
* @return If Robot is on crossroad
*/
bool checkCrossroad() {
	return (SensorValue[SENSOR_L] < TRESHHOLD_L || SensorValue[SENSOR_L] < TRESHHOLD_R);
}

/**
* @brief Crossroad state, Waits at crossroad until command is given
* Crossroad does the following:
* 	- Uses checkBluetoothMessage function to get command
*		- Sets robot
*/
void crossroadState() {
	string command = "";
	checkBluetoothMessage(command);

	// TODO: Remove.
	if (command == "A") {
		STATE = MOVING;
		return;
	}

	// if commanded left and there is a left
	if (command == "LEFT") {
		turnLeft();
		STATE = MOVING;
		displayBigTextLine(4, "LEFT");
	// If commanded right and there is a right
	} else if (command == "RIGHT") {
		turnRight();
		STATE = MOVING;
		displayBigTextLine(4, "LEFT");
	} else if (command == "FIRE") {
		// MAKE ROBOT CHOOSE
		turnRight();
		displayBigTextLine(4, "LEFT OF RIGHT");
	}

	// Stop motors if not stopped already
	if (motor[MOTOR_L] > 0 || motor[MOTOR_R] > 0) {
		slowBreak();
	}
}

void initState() {
	// tp (target power) is the power level of the motors.
	const int tp = 10;

	motor[MOTOR_L] = tp;
	motor[MOTOR_R] = -tp;

	while (SensorValue[SENSOR_R] == 0) {}
	for (int i=0; i<9000; i++) {
			if (SensorValue[SENSOR_L] > WHITE_L) {
				WHITE_L = SensorValue[SENSOR_L];
			} else if (SensorValue[SENSOR_L] <= BLACK_L) {
				BLACK_L = SensorValue[SENSOR_L];
			}

			if (SensorValue[SENSOR_R] > BLACK_R) {
				WHITE_R = SensorValue[SENSOR_R];
			} else if (SensorValue[SENSOR_R] <= BLACK_R) {
				BLACK_R = SensorValue[SENSOR_R];
			}

		  wait1Msec(1);
	}

	motor[MOTOR_L] = 0;
	motor[MOTOR_R] = 0;

	OFFSET_L = (BLACK_L+WHITE_L)/2;
	OFFSET_R = (BLACK_R+WHITE_R)/2;
}

/**
* @brief Plays sound
*/
void makeSound(){
	// TODO: enabble
	// playSound(soundUpwardTones);
}

/**
* @brief Move state, moves robot etc
* Move state does the following:
* 	- Uses checkBluetoothMessage function to get command
*   - Sets robot state to STOPPED when "FIRE" command is given
*		- Sets robot state to STOPPED when there is a objec
*		- Plays sound with the makeSound function
*		- Sets motor speed to MAX_SPEED
*/
// TODO: Remove counter and replace with time.
int COUNTER = 0;
void moveState() {
	// Get command from app.
	string command = "";
	checkBluetoothMessage(command);

	// Startup bot when the fire button is pressed on the Android app.
	if (command == "FIRE") {
		STATE = STOPPED;
		return;
	}

	// If we see a object.
	if (SensorValue[SENSOR_S] < VIEW_DIST) {
		STATE = STOPPED;
		return;
	}

	if (checkCrossroad()) {
		STATE = CROSSROAD;
		return;
	}

	// Play sound.
	COUNTER++;
	if (COUNTER > 75) {
		makeSound();
		COUNTER = 0;
	}

	// Line follow using PID.
	int error = (SensorValue[SENSOR_L]-OFFSET_L) - (SensorValue[SENSOR_R]-OFFSET_R);
	INTEGRAL += error;
	DERIVATIVE = error - LAST_ERROR;

	TURN = (KP*error) + (KI*INTEGRAL) + (KD*DERIVATIVE);

	LAST_ERROR = error;

	// Actually alter motor speed.
	motor[MOTOR_L] = TP - TURN;
	motor[MOTOR_R] = TP + TURN;
}

/**
* @brief Reduce motor speed of robot slowly
*/
void slowBreak() {
	int speed = ((motor[MOTOR_L] > motor[MOTOR_R]) ? motor[MOTOR_L] : motor[MOTOR_R]);

	for (int i = speed; i > 0; i--) {
		motor[MOTOR_L] = ((motor[MOTOR_L] > 0) ? motor[MOTOR_L] - 1 : 0);
		motor[MOTOR_R] = ((motor[MOTOR_L] > 0) ? motor[MOTOR_L] - 1 : 0);

		wait1Msec(10);
	}

	motor[MOTOR_L]  = 0;
	motor[MOTOR_R] = 0;
}

/**
* @brief Stop state, stops the robot etc
* Move state does the following:
* 	- Uses checkBluetoothMessage function to get command
*   - Sets robot state to MOVING when "FIRE" command is given
*		- uses slowBreak function when motor speed > 0
*/
void stopState() {
	// Get command from app
	string command = "";
	checkBluetoothMessage(command);

	// Startup bot when fire button is pressed on the app
	if (command == "FIRE") {
		STATE = MOVING;
		return;
	}

	// Stop motors if not stopped already
	if (motor[MOTOR_L] > 0 || motor[MOTOR_R] > 0) {
		slowBreak();
	}
}

void turnLeft () {
	motor[MOTOR_L] = -15;
	motor[MOTOR_R] = 15;
	while (SensorValue[SENSOR_R] >= TRESHHOLD_R) {
		wait1Msec(1);
	}
	motor[MOTOR_L] = 0;
	motor[MOTOR_R] = 0;

	wait1Msec(50);
}

void turnRight() {
	motor[MOTOR_L] = 15;
	motor[MOTOR_R] = -15;
	while (SensorValue[SESNOR_L] >= TRESHHOLD_L) {
		wait1Msec(1);
	}
	motor[MOTOR_L] = 0;
	motor[MOTOR_R] = 0;

	wait1Msec(50);
}


/*
* Main loop.
*/

/**
* @brief Main, Starts main loop and calls functions based on STATE
*/
task main() {
	bool running = true;

	while(running) {
		switch(STATE) {
			case CROSSROAD:
				displayBigTextLine(0, "Crossroad");
				crossroadState();
				break;
			case INIT:
				initState();
				STATE = STOPPED;
			case MOVING:
				displayBigTextLine(0, "Moving");
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
