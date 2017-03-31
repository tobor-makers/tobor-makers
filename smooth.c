#pragma config(Sensor, S4, sensorLight, sensorLightActive)
#pragma config(Motor, motorB, motorRight, tmotorNXT, PIDControl, driveRight, encoder)
#pragma config(Motor, motorC, motorLeft,  tmotorNXT, PIDControl, driveLeft,  encoder)

task main() {
	const int offset = 50;
	const int kp = 230;
	const int ki = 0;
	const int kd = 0;
	const int tp = 30;

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
