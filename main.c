#pragma config(Sensor, S1,     sensorLight,    sensorLightActive)
#pragma config(Sensor, S2,     sensorSonar,    sensorSONAR)
#pragma config(Sensor, S3,     sensorRGB,      sensorColorNxtRED)
#pragma config(Motor,  motorB,          motorRight,    tmotorNXT, PIDControl, driveRight, encoder)
#pragma config(Motor,  motorC,          motorLeft,     tmotorNXT, PIDControl, driveLeft, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(NXT)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

typedef enum {
	CROSSROAD = 0,
	MOVING = 1,
	STOPPED = 2,
	RIP = 3
} State;

// GLOBALS
const int MAX_SPEED = 50;
const int VIEW_DIST = 25;
State STATE = STOPPED;
const int LIGHT_TRESHHOLD = 50; // 60
const int RGB_TRESHHOLD = 30; // 40
// Bluetooth gobals
const int kMaxSizeOfMessage = 30;
const int INBOX = 5;

// ---------
// FUNCTIONS
// ---------

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
  	stringFromChars(command, (char *) nRcvBuffer);
  }
}

/**
* @brief Checks for crossroads
* @return If Robot is on crossroad
*/
bool checkCrossroad() {
	return (SensorValue[sensorLight] < LIGHT_TRESHHOLD || SensorValue[sensorRGB] < RGB_TRESHHOLD);
}

/**
* @brief Reduce motor speed of robot slowly
*/
void slowBreak() {
	int speed = ((motor[motorLeft] > motor[motorRight]) ? motor[motorLeft] : motor[motorRight]);
  for (int i = speed; i > 0; i--) {
    motor[motorLeft] = i;
    motor[motorRight] = i;
		displayBigTextLine(3, "s: %d", i);

    wait1Msec(10);
  }
  motor[motorLeft]  = 0;
  motor[motorRight] = 0;
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


	// TODO: LINE follow

	// Full speed forward
	motor[motorLeft] = MAX_SPEED;
	motor[motorRight] = MAX_SPEED;
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
	if (motor[motorLeft] > 0 || motor[motorRight] > 0) {
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
	if (command == "LEFT" && SensorValue[sensorLight] < LIGHT_TRESHHOLD) {
		// MOVE LEFT
		displayBigTextLine(4, "LEFT");
	// If commanded right and there is a right
	} else if (command == "RIGHT" && SensorValue[sensorRGB] < RGB_TRESHHOLD) {
		// MOVE RIGHT
		displayBigTextLine(4, "LEFT");
	} else if (command == "FIRE") {
		// MAKE ROBOT CHOOSE
		displayBigTextLine(4, "LEFT OF RIGHT");
	}

	// Stop motors if not stopped already

	if (motor[motorLeft] > 0 || motor[motorRight] > 0) {
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
		// displayBigTextLine(4, "%d, %d", SensorValue[sensorLight], SensorValue[sensorRGB]);
		switch(STATE) {
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
