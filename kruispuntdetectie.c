#pragma config(Sensor, S4,     sensorLight,    sensorLightActive)
#pragma config(Sensor, S3,     sensorRGB,      sensorColorNxtRED)
#pragma config(Motor,  motorB,          motorRight,    tmotorNXT, PIDControl, driveRight, encoder)
#pragma config(Motor,  motorC,          motorLeft,     tmotorNXT, PIDControl, driveLeft, encoder)
#pragma platform(NXT)


const int LIGHT_TRESHHOLD = 45;
const int RGB_TRESHHOLD = 25;

bool crossroadDetection() {
	displayTextLine(0, "light: %d", SensorValue[sensorLight]);
	displayTextLine(1, "RGB: %d", SensorValue[sensorRGB]);
	return (SensorValue[sensorLight] < LIGHT_TRESHHOLD && SensorValue[sensorRGB] < RGB_TRESHHOLD);
}

task main() {
	motor[motorLeft] = 20;
	motor[motorRight] = 20;

	bool crossRoadDetected = false;
	while(!crossRoadDetected) {
		crossRoadDetected = crossroadDetection();
		displayTextLine(2, "Crossroad: %d", crossRoadDetected);
		wait1Msec(1);
	}
	motor[motorLeft] = 0;
	motor[motorRight] = 0;
}
