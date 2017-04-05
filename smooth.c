#pragma config(Sensor, S1,     sensorL, sensorLightActive)
#pragma config(Sensor, S3,     sensorR, sensorColorNxtRED)
#pragma config(Motor,  motorC, motorL,  tmotorNXT, PIDControl, driveLeft,  encoder)
#pragma config(Motor,  motorB, motorR,  tmotorNXT, PIDControl, driveRight, encoder)
#pragma platform(NXT)

// See https://punpun.xyz/f747.pdf for more information about PID.

task main() {
	// offset is the avarage of the light-sensor measurements of "total white" and "total black".
	const int offsetL = 54;
	const int offsetR = 37;

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

	while (true) {
		// Print debug info.
		displayTextLine(0, "%d", SensorValue[sensorL]);
		displayTextLine(1, "%d", SensorValue[sensorR]);

		int error = (SensorValue[sensorL]-offsetL) - (SensorValue[sensorR]-offsetR);
		integral += error;
		derivative = error - lastError;

		turn = (kp*error) + (ki*integral) + (kd*derivative);

		lastError = error;

		// Actually alter motor speed.
		motor[motorL] = tp - turn;
		motor[motorR] = tp + turn;
		}
}
