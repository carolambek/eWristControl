/*
PID.h
Author: Charles Lambelet
Created: June, 2019
*/

#ifndef PID_h
#define PID_h

#define DIRECT  0
#define REVERSE  1

class PID {
public:

	PID();										// Constructor
	~PID();										// Destructor

	void initialize();							// initialize some variables.
	bool compute();								// perform PID calculation.
	void setVariables(float*, float*, float*);	// set variables to be computed.
	void setTunings(float, float, float);		// set tuning parameters.
	void setLimits(float, float, float, float); // clamp the output to a specific range.
	void setControllerDirection(int);			// set the direction of controller:
												// DIRECT: means the output will increase when error is positive.
												// REVERSE: what do you think?!?. 
	void setSampleTime(int);					// set sampling in microseconds at which the PID calculation is performed.
	
	int getDirection();
	float getCurErr();
	float getKp();
	float getKi();
	float getKd();

private:

	int controllerDirection;
    
	float kp;                    // (P)roportional tuning parameter
	float ki;                    // (I)ntegral tuning parameter
	float kd;                    // (D)erivative tuning parameter

	float *myInput;
	float *myOutput;
	float *mySetpoint;
			  
	unsigned long lastTime;       // [us]
	unsigned long sampleTime;     // [us]
	float sumError, lastError;
	float outPutMin, outPutMax;
	float sumErrMin, sumErrMax;
	float currentError;
};
#endif
