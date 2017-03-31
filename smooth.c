#pragma config(Sensor, S4, sensorLight, sensorLightActive)
#pragma config(Motor, motorB, motorRight, tmotorNXT, PIDControl, driveRight, encoder)
#pragma config(Motor, motorC, motorLeft,  tmotorNXT, PIDControl, driveLeft,  encoder)

// See https://punpun.xyz/f747.pdf for more information.

task main() {
	// offset is the avarage of the light-sensor measurements of "total white" and "total black".
	// "total white" being a value 61, and "total black" being a value of 35.
	const int offset = 48;

	const int kp = 6;
	const int ki = 0;
	const int kd = 0;

	// tp (target power)  is the power level of both motors when the robot is supposed to go
	// straight ahead, which it does when the error has a value of 0.
	const int tp = 30;

	// integral is the running sum of the error.
	int integral = 0;
	int derivative = 0;
	int lastError = 0;
	int turn = 0;

	while (true) {
		int error = SensorValue[sensorLight] - offset;
		integral += error;
		derivative = error - lastError;
		turn = kp*error + ki*integral + kd*derivative;
		turn = turn/100;

		motor[motorLeft] = tp + turn;
		motor[motorRight] = tp - turn;

		lastError = error;
	}
}
