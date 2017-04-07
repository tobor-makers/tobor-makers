#pragma config(Sensor, S1,     sensorL,        sensorLightActive)
#pragma config(Sensor, S2,     sensorSonar,    sensorSONAR)
#pragma config(Sensor, S3,     sensorR,        sensorColorNxtRED)
#pragma config(Motor,  motorB,          motorR,    tmotorNXT, PIDControl, driveRight, encoder)
#pragma config(Motor,  motorC,          motorL,     tmotorNXT, PIDControl, driveLeft, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(NXT)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
* Define enum's
*/
// State
typedef enum {
	CROSSROAD = 0,
	MOVING = 1,
	STOPPED = 2,
	RIP = 3,
	INIT = 4
} State;

/*
* GLOBALS
*/
const int VIEW_DIST = 25;
const int L_TRESHHOLD = 40; // 50
const int R_TRESHHOLD = 20; // 30
State STATE = INIT;
/*
* Bluetooth gobals
*/
const int kMaxSizeOfMessage = 30;
const int INBOX = 5;
/*
* PID GLOBALS
* See https://punpun.xyz/f747.pdf for more information about PID.
*/

int blackL = 100;
int blackR = 100;
int whiteL = 0;
int whiteR = 0;

// offset is the avarage of the light-sensor measurements of "total white" and "total black".
int offsetL = 0;
int offsetR = 0;

// kp (the konstant for the proportional controller) is calculated using `0.60*kc`,
// kc (critical gain) being a value where the robot follows the line and gives noticeable
// oscillation but not really a wild one.
const int kp = 1;
const int ki = 0.001;
const int kd = 15;
// tp (target power) is the power level of both motors when the robot is supposed to go
// straight ahead, which it does when the error has a value of 0.
const int tp = 35;
// integral is the running sum of the error.
int integral = 0;
// derivative is the current error minus the previous error.
int derivative = 0;
// These two should be fairly self-explanatory.
int lastError = 0;
int turn = 0;


// ---------
// FUNCTIONS
// ---------

void initState() {
	// tp (target power) is the power level of the motors.
	const int tp = 10;

	motor[motorL] = tp;
	motor[motorR] = -tp;
	while (SensorValue[sensorR] == 0) {}
	for (int i=0; i<9000; i++) {
			if (SensorValue[sensorL] > whiteL) {
				whiteL = SensorValue[sensorL];
			} else if (SensorValue[sensorL] <= blackL) {
				blackL = SensorValue[sensorL];
			}

			if (SensorValue[sensorR] > whiteR) {
				whiteR = SensorValue[sensorR];
			} else if (SensorValue[sensorR] <= blackR) {
				blackR = SensorValue[sensorR];
			}
		  wait1Msec(1);
	}

	motor[motorL] = 0;
	motor[motorR] = 0;
	offsetL = (blackL+whiteL)/2;
	offsetR = (blackR+whiteR)/2;
}

void turnLeft () {
	motor[motorL] = -15;
	motor[motorR] = 15;
	while (SensorValue[sensorR] >= R_TRESHHOLD) {
		wait1Msec(1);
  }
  motor[motorL] = 0;
  motor[motorR] = 0;
  wait1Msec(50);
}

void turnRight() {
	motor[motorL] = 15;
	motor[motorR] = -15;
	while (SensorValue[sensorL] >= L_TRESHHOLD) {
		wait1Msec(1);
  }
  motor[motorL] = 0;
  motor[motorR] = 0;
  wait1Msec(50);
}

/**
* @brief Checks for bluetooth messages and sets command string
* @param command String to hold the bluetooth message
*/
void checkBluetoothMessage(string &command) {
	int nSizeOfMessage;
	TFileIOResult nBTCmdRdErrorStatus;
	ubyte nRcvBuffer[kMaxSizeOfMessage];
  nSizeOfMessage = cCmdMessageGetSize(INBOX);

  if (nSizeOfMessage > kMaxSizeOfMessage)
    nSizeOfMessage = kMaxSizeOfMessage;

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

	return (SensorValue[sensorL] < L_TRESHHOLD || SensorValue[sensorR] < R_TRESHHOLD);
}

/**
* @brief Reduce motor speed of robot slowly
*/
void slowBreak() {
	int speed = ((motor[motorL] > motor[motorR]) ? motor[motorL] : motor[motorR]);
  for (int i = speed; i > 0; i--) {
    motor[motorL] = ((motor[motorL] > 0) ? motor[motorL] - 1 : 0);
    motor[motorR] = ((motor[motorL] > 0) ? motor[motorL] - 1 : 0);
    wait1Msec(10);
  }
  motor[motorL]  = 0;
  motor[motorR] = 0;
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
// TODO: REMOVE counter and replace with time
int counter = 0;
void moveState() {
	// Get command from app
	string command = "";
	checkBluetoothMessage(command);

	// Startup bot when fire button is pressed on the app
	if (command == "FIRE") {
		STATE = STOPPED;
		return;
	}

	// If we see a object
	if (SensorValue[sensorSonar] < VIEW_DIST) {
		STATE = STOPPED;
		return;
	}

	if (checkCrossroad()) {
		STATE = CROSSROAD;
		return;
	}

	// Play sound
	counter++;
	if (counter > 75) {
		makeSound();
		counter = 0;
	}

	/*
	* Line follow using PID
	*/
	int error = (SensorValue[sensorL]-offsetL) - (SensorValue[sensorR]-offsetR);
	integral += error;
	derivative = error - lastError;

	turn = (kp*error) + (ki*integral) + (kd*derivative);

	lastError = error;

	// Actually alter motor speed.
	motor[motorL] = tp - turn;
	motor[motorR] = tp + turn;
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
	if (motor[motorL] > 0 || motor[motorR] > 0) {
		slowBreak();
	}
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

	// TODO: remove
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

	if (motor[motorL] > 0 || motor[motorR] > 0) {
		slowBreak();
	}
}

/**
* @brief Main, Starts main loop and calls functions based on STATE
*/
task main()
{
	bool running = true;
	while(running) {
		switch(STATE) {
			case INIT:
				initState();
				STATE = STOPPED;
				break;
			case CROSSROAD:
				displayBigTextLine(0, "Crossroad");
				crossroadState();
				break;
			case MOVING:
				displayBigTextLine(0, "Moving");
				moveState();
				break;
			case STOPPED:
				displayBigTextLine(0, "Stopped");
				stopState();
				break;
			case RIP:
				displayBigTextLine(0, "Rip");
				running = false;
				break;
			default:
				displayBigTextLine(0, "No state");
		}

		wait1Msec(1);
	}
}
