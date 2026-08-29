/*
IMU.cpp
Author: Charles Lambelet
Created: October, 2019
*/

#include "IMU.h"

volatile bool IMU::mpuInterrupt = false;

// this function deals with interrupts and need to be declared outside the class!
void dmpDataReady() {
	IMU::mpuInterrupt = true;
}

// Constructor. Initialize different variables
IMU::IMU() {
	IMU::initialize();
}

// Destructor.
IMU::~IMU() {

}

// Initialize IMU.  
void IMU::initialize() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	// initialize DMP
	initMPU6050DMP();
	counter_init = 0;
}

// Compute IMU calculations.
void IMU::compute() {
	computeMPU6050DMP();
	//calibrateMPU6050DMP(A_PITCH_IMU_BRD1, B_PITCH_IMU_BRD1, C_PITCH_IMU_BRD1, D_PITCH_IMU_BRD1, A_ROLL_IMU_BRD1, B_ROLL_IMU_BRD1, C_ROLL_IMU_BRD1, D_ROLL_IMU_BRD1);
}

// Initialize MPU6050DMP
void IMU::initMPU6050DMP() {

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro/accel offsets here, scaled for min sensitivity
	#ifdef IMU_BRD1_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD1);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD1);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD1);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD1);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD1);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD1); // 1688 factory default for my test chip
	#endif
	#ifdef IMU_BRD2_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD2);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD2);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD2);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD2);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD2);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD2); // 1688 factory default for my test chip
	#endif
	#ifdef IMU_BRD3_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD3);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD3);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD3);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD3);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD3);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD3); // 1688 factory default for my test chip
	#endif
	#ifdef IMU_BRD4_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD4);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD4);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD4);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD4);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD4);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD4); // 1688 factory default for my test chip
	#endif
	#ifdef IMU_BRD5_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD5);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD5);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD5);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD5);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD5);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD5); // 1688 factory default for my test chip
	#endif
	#ifdef IMU_BRD6_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD6);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD6);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD6);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD6);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD6);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD6); // 1688 factory default for my test chip
	#endif
	#ifdef IMU_BRD7_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD7);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD7);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD7);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD7);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD7);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD7); // 1688 factory default for my test chip
	#endif
	#ifdef IMU_BRD8_IN_USE
		mpu.setXGyroOffset(GYROX_OFFSET_IMU_BRD8);
		mpu.setYGyroOffset(GYROY_OFFSET_IMU_BRD8);
		mpu.setZGyroOffset(GYROZ_OFFSET_IMU_BRD8);
		mpu.setXAccelOffset(ACCELX_OFFSET_IMU_BRD8);
		mpu.setYAccelOffset(ACCELY_OFFSET_IMU_BRD8);
		mpu.setZAccelOffset(ACCELZ_OFFSET_IMU_BRD8); // 1688 factory default for my test chip
	#endif

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable interrupt detection
		Serial.println(F("Enabling interrupt detection..."));
		attachInterrupt(digitalPinToInterrupt(mpuIntPin), dmpDataReady, RISING);
		//attachInterrupt(digitalPinToInterrupt(12), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

// Compute MPU6050DMP. Takes about 800us to be executed
void IMU::computeMPU6050DMP() {  // DMP data motion processor -> directly processes raw data

	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	//  if(mpuInterrupt == true)
	//  {
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient, i.e. too slow)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		//Serial.println(F("FIFO overflow!"));
	}
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) {
			fifoCount = mpu.getFIFOCount();
		}

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);	// takes about 4800us to be executed every 5ms (200Hz) in addition to the 800us
		mpu.resetFIFO();
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// compute Euler angles in degrees (very fast execution => about 240us for the 3 functions)
		mpu.dmpGetQuaternion(&q, fifoBuffer);  		// execution time = 6us
		mpu.dmpGetGravity(&gravity, &q);			// execution time = 14us
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);	// execution time = 220us
		/*
		Serial.print(gravity.x, 2);
		Serial.print('\t');
		Serial.print(gravity.y, 2);
		Serial.print('\t');
		Serial.println(gravity.z, 2);
		*/
		/*
		// wait 1200 cylces at the beginning for yaw angle to stabilize
		if (counter_init <= 1200) {
			yaw_init = ypr[0];
			ypr[1] = 0;
			ypr[2] = 0;
			counter_init++;
		}
		ypr[0] = ypr[0] - yaw_init;
		*/
		//Serial.print("ypr\t");
		//Serial.print(ypr[0] * 180/M_PI);
		//Serial.print("\t");
		//Serial.print(ypr[1] * 180/M_PI);
		//Serial.print("\t");
		//Serial.println(ypr[2] * 180/M_PI);
		//Serial.println(counter_yaw_init);
	}
}

// Reset FIFO buffer when things are getting slow.
void IMU::resetFIFO() {
	mpu.resetFIFO();
}

/*
Calibrate MPU6050DMP. It only calibrates pitch and roll axis since yaw does not need any calibration.
The returned value is correct for angles from -140deg to 140 deg. When the board is fully upside down,
i.e. from -140deg to 180deg and 140deg to 180deg, the spatial orientation estimates are wrong.
This function is not required for functioning MPU6050. However, some of the boards are malfunctioning from factory settings already.
They will typically exhibit a certain delay in responding and overshoot. => If it is the case, change the board!
Therefore, check that the board indicates adequate angles according to its spatial orientation and responds quickly without overshooting!
*/
void IMU::calibrateMPU6050DMP(float aPitch, float bPitch, float cPitch, float dPitch, float aRoll, float bRoll, float cRoll, float dRoll) {
	ypr_calib[0] = ypr[0];
	ypr_calib[1] = aPitch*ypr[1]*ypr[1]*ypr[1] + bPitch*ypr[1]*ypr[1] + cPitch*ypr[1] + dPitch;
	ypr_calib[2] = aRoll*ypr[2]*ypr[2]*ypr[2] + bRoll*ypr[2]*ypr[2] + cRoll*ypr[2] + dRoll;
}

// Getters
float IMU::getYaw() {
	return ypr[0];					// yaw in the referential of forearm module is yaw of IMU
	//return ypr_calib[0];			// yaw in the referential of forearm module is yaw of IMU
}

float IMU::getPitch() {
	return ypr[2];					// pitch in the referential of forearm module is roll of IMU
	//return ypr_calib[2];			// pitch in the referential of forearm module is roll of IMU
}

float IMU::getRoll() {
	return ypr[1];					// roll in the referential of forearm module is pitch of IMU
	//return ypr_calib[1];			// roll in the referential of forearm module is pitch of IMU
}

float IMU::getGravityZ() {
	return gravity.z;
}
