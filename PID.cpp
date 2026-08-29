/*
PID.cpp
Author: Charles Lambelet
Created: June, 2019
*/

#include "Arduino.h"
#include "PID.h"

// Constructor. Initialize different variables
PID::PID() {
	sampleTime = 100;									// default controller sample time is 1000us

	PID::initialize();
	PID::setLimits(-0.3, 0.3, -20, 20);					// setLimits(double OutMin, double OutMax, double ErrMin, double ErrMax)
	PID::setControllerDirection(DIRECT);
}

// Destructor.
PID::~PID() {
	kp = ki = kd = 0.0;
	*myInput = *myOutput = *mySetpoint = 0.0;
	lastTime = sampleTime = 0;
	sumError = lastError = 0.0;
}

// Initialize some variables.  
void PID::initialize() {
	lastTime = 0;
	sumError = lastError = 0.0;
}

// Compute the controller calculations.
bool PID::compute() {

	unsigned long nowUs = micros();
	unsigned long timeChange = (nowUs - lastTime);
	if (timeChange >= sampleTime) {
		// Compute error variables
		currentError = *mySetpoint - *myInput;
		sumError += currentError;
		if (sumError > sumErrMax) { sumError = sumErrMax; }
		else if (sumError < sumErrMin) { sumError = sumErrMin; }
		float varError = (currentError - lastError);
  
		// Compute PID output
		float output = kp * currentError + ki * sumError + kd * varError;
      
		if (output > outPutMax) { output = outPutMax; }
		else if (output < outPutMin) { output = outPutMin; }
		*myOutput = output;

		// DEBUG
		//Serial.print("*myInput: ");
		//Serial.print("\t");
		//Serial.println(*myInput, 3);
		//Serial.print("*mySetpoint: ");
		//Serial.print("\t");
		//Serial.println(*mySetpoint, 3);
		//Serial.print("currentError: ");
		//Serial.print("\t");
		//Serial.println(currentError, 3);
		//Serial.print("sumError: ");
		//Serial.print("\t");
		//Serial.println(sumError, 3);
		//Serial.print("lastError: ");
		//Serial.print("\t");
		//Serial.println(lastError, 3);
		//Serial.print("varError: ");
		//Serial.print("\t");
		//Serial.println(varError, 3);
		//Serial.print("output: ");
		//Serial.print("\t");
		//Serial.println(output, 3);
    
		lastError = currentError;
		lastTime = nowUs;
    
		return true;
	}
	else return false;
}

// Set input, target and output.
void PID::setVariables(float* input, float* output, float* setPoint) {
	myInput = input;
	myOutput = output;
	mySetpoint = setPoint;
}

// Set tuning parameters.
void PID::setTunings(float Kp, float Ki, float Kd) {
	if (Kp < 0 || Ki < 0 || Kd < 0) { return; }

	kp = Kp;
	ki = Ki;
	kd = Kd;

	if (controllerDirection == REVERSE) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}

// Set output limits.
void PID::setLimits(float outMin, float outMax, float errMin, float errMax) {
	if (outMin >= outMax || errMin >= errMax) { return; }

	outPutMin = outMin;
	outPutMax = outMax;
	sumErrMin = errMin;
	sumErrMax = errMax;

	if (*myOutput > outPutMax) { *myOutput = outPutMax; }
	else if (*myOutput < outPutMin) { *myOutput = outPutMin; }

	if (sumError > sumErrMax) { sumError = sumErrMax; }
	else if (sumError < sumErrMin) { sumError = sumErrMin; }
}

// Set controller direction. => DIRECT: +input leads to +output. REVERSE: +input leads to -output.
void PID::setControllerDirection(int direction) {
	if (direction != controllerDirection) {
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controllerDirection = direction;
}
  
// Set the sampling in microseconds at which the calculation is performed.
void PID::setSampleTime(int newSampleTime) {
	if (newSampleTime > 0) {
		float ratio  = (float)newSampleTime / (float)sampleTime;
		ki *= ratio;
		kd /= ratio;
		sampleTime = (unsigned long)newSampleTime;
	}
}

// Getters
int PID::getDirection() {
	return controllerDirection;
}

float PID::getCurErr() {
	return currentError;
}

float PID::getKp() {
	return  kp;
}

float PID::getKi() {
	return  ki;
}

float PID::getKd() {
	return  kd;
}
