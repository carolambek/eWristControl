/*
eWristControl.ino
Author: Charles Lambelet
Created: August 15, 2017
Modified: Marc Siegenthaler - June 2018
Modified: Melvin Mathis - June 2018
Modified: Charles Lambelet - March 2019
*/

/*
GENERAL REMARKS:
- !!!BEFORE STARTING!!!:
	1. set exo in use, i.e. right-hand or left-hand with EXO_IN_USE in EWrist.h
	2. set load cell/main board according to exo in use in EWrist.h
	3. set control mode (speed or current) in EWrist.h according to ESCON module
	4. set parameters mode for the admittance controller, i.e. FIXED_ADM_PARAM or VARIABLE_ADM_PARAM, in EWrist.h
	5. set if IMU is in use with IMU_ENABLED in EWrist.h
	6. set IMU MPU6050 board in use, i.e. board #1, #2, #3, ... in IMU.h
	7. set if PID is in use with PID_ENABLED in EWrist.h
	8. set if exo is to be used with visuomotor task with VMT_ENABLED in EWrist.h
- This code is meant to be used with the following electronics: Maxon motor and ESCON drive, Hall sensor encoder, IMU MPU6050, Teensy 3.2 and Raspberry Pi Zero.
- In real-time application, the print() function which requires a certain amount of cylces to be executed...
  ...can affect a lot the controller when called in the control loop!
- The sin() function and averaging functions impact a lot on the readings from the RPi0 and slow down the control loop!
- Multiplication is faster than division!
- All digital pins are 5 volt tolerant on Teensy 3.2 & 3.1. However, the analog-only pins (A10-A14), AREF, Program and Reset are 3.3V only.
- The electronics on the boards needs to warm up at the beginning (during ~5min) to work appropriately.
	=> for instance amplification chip for loadcell (i.e. to have no offset in force measurement).
- If no data appear in the serial monitor, run the terminal of Arduino IDE and reset the Teensy (with switch button). c.f. raw mode of serial.
  For the 1kHz serial you might have to reset the teensy several time before data are streamed in terminal of arduino IDE (keep baud rate at 115200). Also enabling 12V can help!
- The PID class can sometimes cause problems. If loop() does not start because of myPID, try to declare it before or after myADC and upload the sketch several times!
  Also try to comment all PID related objects, upload the sketch and then uncomment and reupload! You can also try to upload another simple example sketch with arduino IDE in between.
  For the loop() function to start, myPID must be used in loop().
  If you don't need PID, do not declare it!
- If IMU is used, the control loop timing of 1000us cannot be held! => the loop will take about 1250us to be executed and about 5630us every 5,7ms.
- If IMU data are not updated, shortly enabling the 12V on the forearm board might solve the issue.
- Be sure to calibrate the IMU chip before use => see IMU.h
- Do not forget to check which variables you want to use and need to be filtered with averageADCVar() (i.e. angPosEncRad, forceMeas, angVelMotEncRPM, motCurMeas, batVltMeas, batCurMeas)
  Filtering variables slow down the loop!

- Naming rules for variables:
	- Uppercase names are reserved for constant variables (#define).
	- For normal variables naming starts with a lowercase and then uppercase for each word. Words are then concatenated together.
	- For functions naming uses the same convention as for normal variables.
	- For classes naming starts with an uppercase character.
*/

#include "EWrist.h"
#include "Subject.h"
#include "AANController.h"
#include "PID.h"
#include "IMU.h"

// pins on teensy
const int loadCellDownPin = A19;    // ADC_1    const => "read-only" => value cannot be changed
const int loadCellUpPin = A20;      // ADC_1
const int motSpdPin = A0;           // ADC_0
const int motCurPin = A1;           // ADC_0
const int posEncPin = A2;           // ADC_0
const int spdOrCurCtrPin = A14;     // DAC
const int batVltPin = A15;          // ADC_1
const int batCurPin = A16;          // ADC_1
const int enableCWPin = 32;
const int enableCCWPin = 17;
const int mpuIntPin = 12;
const int gpio13Pin = 8;            // =PROG3 button. GPIO13 on RPI is connected to pin D8 of teensy.
const int gpio16Pin = 9;			// =PROG4 button. GPIO16 on RPI is connected to pin D9 of teensy.
const int gpio19Pin = 25;			// =SD_RPI button (on single short press) and lights up LED11. =Debug mode status on RPi0.
const int gpio26Pin = 28;
const int led1DebugPin = 7;
const int led2DebugPin = 24;

// timing variables
unsigned long startLoopMs = 0;
unsigned long startLoopUs = 0;
unsigned int sampleTimeCtrlLoop = 1000;         // [us], default=1000
//unsigned int sampleTimeSendLoop = 20000;        // [us], sending rate of Myo => 50Hz
unsigned int sampleTimeSendLoop = 16667;        // [us], sending rate of visuomotor task => 60Hz
unsigned int sampleTimePrintLoop = 50;          // [ms], default=50. must not be faster than 5ms otherwise serial monitor of Visual Studio Code freezes but not the one of Arduino IDE
unsigned int deltaTimeCtrlLoop = 0;
unsigned int deltaTimePrintLoop = 0;
unsigned int deltaTimeSendLoop = 0;
unsigned int lastTimeCtrlLoop = 0;
unsigned int lastTimePrintLoop = 0;
unsigned int lastTimeSendLoop = 0;
unsigned int tic = 0;
unsigned int toc = 0;
unsigned int ticTocDiff = 0;
unsigned long mainLoopCounter = 0;
unsigned long ctrlLoopCounter = 0;
unsigned long recordCounter = 0;
unsigned long tempCounter = 0;
double deltaTimeCtrlLoopSec = 0;

// ADC measured variables
float forceMeas;					// [N]
float forceOffset;					// [N]
float angPosEncDeg;					// [deg], starts from 0 deg to 154 deg and more
float angPosEncRad;					// [rad], starts from r rad to 2.7 rad and more
float lastAngPosEncRad;				// [rad]
float angVelPosEncRadSec;			// [rad/s]
float angVelMotEncRPM;				// [rpm]
float angVelMotEncRadSec;			// [rad/s]
float angVelWristRadSec;			// [rad/s]
float angVelWristDegSec;			// [deg/s]
float motCurrentMeas;				// [A]
float battVoltageMeas;				// [V]
float battCurrentMeas;				// [A]

// IMU
float yawAngDeg;					// [deg]
float pitchAngDeg;					// [deg]
float rollAngDeg;					// [deg]
float *pitchRollArray;
float gravityZ;

// EMG, Admittance
float EMGControlInputFlex;
float EMGControlInputExt;
float gravityForce;					// [Nm]
float angVelVirtWristRadSec;		// [rad/s]
float angVelVirtMotRadSec;			// [rad/s]
float lastAngVelVirtWristRadSec;	// [rad/s]
float controlCommand;
float angVelVirtMotRPM;				// [rpm]
float torqueWristDesired;			// [Nm]
float lastTorqueWristDesired;		// [Nm]
float *idxForceFreqMag;
float idxForceInstab;				// varies between 0 and 1
float lastIdxForceInstab;
float ampIdxInstab;					// usually varies between 0 and 6-8 N

// PID
float speedCommand;					// [rpm]
float targetAngleRad;				// [rad]

// subject data
float handWeight;					// [N]

// ohters
String incomingString;
bool directionFlag = true;
bool fctExecuted = false;

#ifdef PID_ENABLED
	PID *myPID = new PID();		// it seems myPID must be declared BEFORE myEWrist or even BEFORE myADC otherwise teensy does not start the loop() function?!?
#endif
ADC *myADC = new ADC();
#ifdef IMU_ENABLED
	IMU myIMU;
#endif
EWrist myEWrist;

EWrist average1;
EWrist average2;
EWrist average3;

// received data from serial
float *EMGArrayTemp;

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {

	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(enableCWPin, OUTPUT);
	pinMode(enableCCWPin, OUTPUT);
	//pinMode(led1DebugPin, OUTPUT);  // it seems it does not work with pin 7?!?
	pinMode(led2DebugPin, OUTPUT);
	pinMode(loadCellDownPin, INPUT);
	pinMode(loadCellUpPin, INPUT);
	pinMode(motSpdPin, INPUT);
	pinMode(motCurPin, INPUT);
	pinMode(posEncPin, INPUT);
	pinMode(batVltPin, INPUT);
	pinMode(batCurPin, INPUT);
	pinMode(mpuIntPin, INPUT);
	pinMode(gpio13Pin, INPUT);
	pinMode(gpio16Pin, INPUT);
	pinMode(gpio19Pin, INPUT);

	Serial.begin(115200);
	Serial1.begin(115200);
	Serial1.setTimeout(3);

	analogWriteResolution(12);
	digitalWrite(enableCWPin, LOW);
	digitalWrite(enableCCWPin, LOW);

	myEWrist.initADC(myADC);
	myEWrist.initEMG();
	#ifdef PID_ENABLED
		myEWrist.initPID(myPID);
		// initialize PD controller
		// myPID->setVariables(&angPosEncRad, &speedCommand, &targetAngleRad);	// set variables, i.e. input, output, target
		// myPID->setTunings(27500.0, 0.0, 2750.0);							// set tuning parameters, i.e. kp, ki, kd. default: 27500.0, 0.0, 2750.0
		// myPID->setLimits(-50000, 50000, -10000, 10000);						// set min and max output limits. default: -50000, 50000, -10000, 10000
		// myPID->setControllerDirection(DIRECT);								// default: DIRECT
	#endif
	#ifdef IMU_ENABLED
		// Initialize IMU
		myIMU.initialize();
	#endif
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void loop() {
	startLoopMs = millis();
	startLoopUs = micros();

	////////////////////
	// SERIAL READING //---------------------------------------------------------------
	////////////////////
	// read data from Laptop
	if (Serial.available() > 0) {
		// collect data
		incomingString = Serial.readStringUntil('\n');
		myEWrist.collectDataVMT(incomingString);
		//Serial.println(incomingString);
	}

	// read data from Raspberry Pi Zero
	if (Serial1.available() > 0) {
		// collect EMG data
		incomingString = Serial1.readStringUntil('\n');
		EMGArrayTemp = myEWrist.collectEMGAdptCtrl(incomingString);
		//EMGArrayTemp = myEWrist.collectEMGCMU(incomingString);
	}

	////////////////////////////
	// SENSOR DATA COLLECTION //-------------------------------------------------------
	////////////////////////////
	myEWrist.collectADC(myADC, loadCellDownPin, loadCellUpPin, motCurPin, motSpdPin, posEncPin, batVltPin, batCurPin);
	myEWrist.computeADC();
	myEWrist.averageADCVar(); // takes about 100us to execute when 4 variables are averaged with a window length of 20
	#ifdef IMU_ENABLED
		tic = micros();
		myIMU.compute(); // takes about 814us to execute and about 5630us every 5,6ms (i.e. every 7 loops)!!!
		toc = micros();
		ticTocDiff = toc-tic;
		// if myIMU.compute() has taken about 5,6ms to execute => adapt sampleTimeSendLoop
		if (ticTocDiff > 5000 && ticTocDiff < 10000) { // 10000 limit is important so that sampleTimeSendLoop does not go negative (i.e. very high number for unsigned)
			sampleTimeSendLoop = sampleTimeSendLoop - ticTocDiff;
		}
		// if myIMU.compute() takes abnormally long (i.e. more than 10ms) => reset FIFO buffer of IMU and wait 10ms
		else if (ticTocDiff > 10000) {
			myIMU.resetFIFO();
			delayMicroseconds(10000);
		}
		else {
			sampleTimeSendLoop = 16667; // => 60Hz
		}
	#endif
	// get the angular position [rad] from hall encoder (already averaged WIN_SIZE times by myEWrist.averageADCVar())
	angPosEncRad = myEWrist.getAngPosEncRad();
	// convert angPosEncRad into degree to get angPosEncDeg since angPosEncRad is already filtered.
	angPosEncDeg = myEWrist.rad2Deg(angPosEncRad);
	// reset the filtered angPosEncDeg into the class myEWrist
	myEWrist.setAngPosEncDeg(angPosEncDeg);

	// get the angular velocity at the wrist by deriving the angular position given by the position encoder at the wrist joint.
	// it is used to compare both angVelPosEncRadSec and angVelWristRadSec since they should be the same.
	angVelPosEncRadSec = (lastAngPosEncRad - angPosEncRad) / deltaTimeCtrlLoopSec;
	myEWrist.setAngVelPosEncRadSec(angVelPosEncRadSec);
	// average angVelPosEncRadSec since highly variable.
	myEWrist.averageDeriv();
	// get it back after averaging.
	angVelPosEncRadSec = myEWrist.getAngVelPosEncRadSec();

	// get the measured force [N] (already averaged WIN_SIZE times by myEWrist.averageADCVar())
	forceMeas = myEWrist.getForceMeas(); // [N]
	// get the force offset measured during VMT calibration
	forceOffset = myEWrist.getForceOffset(); // [N]
	// remove force offset
	forceMeas = forceMeas - forceOffset;
	// get subject's hand weight
	handWeight = myEWrist.getSubjectHandWeight();	

	// get the angular velocity from the motor encoder at the shaft (already averaged WIN_SIZE times by myEWrist.averageADCVar())
	angVelMotEncRPM = myEWrist.getAngVelMotEncRPM(); // [rpm] w.r.t. motor shaft
	angVelMotEncRadSec = myEWrist.getAngVelMotEncRadSec(); // [rad/s] w.r.t. motor shaft
	// get the angular velocity from the motor encoder at the wrist
	angVelWristRadSec = myEWrist.convertAngVelMot2Wrist(angVelMotEncRadSec); // [rad/s] w.r.t. wrist axis
	// convert angVelWristRadSec into degree to get angVelWristDegSec
	angVelWristDegSec = myEWrist.rad2Deg(angVelWristRadSec);

	// get motor current draw
	motCurrentMeas = myEWrist.getMotCurMeas();
	// get battery voltage and battery current draw
	battVoltageMeas = myEWrist.getBatVltMeas();
	battCurrentMeas = myEWrist.getBatCurMeas();

	#ifdef IMU_ENABLED
		// get yaw, pitch, roll and z orientation from IMU
		yawAngDeg = myEWrist.rad2Deg(myIMU.getYaw());
		pitchAngDeg = myEWrist.rad2Deg(myIMU.getPitch());
		rollAngDeg = myEWrist.rad2Deg(myIMU.getRoll());
		gravityZ = myIMU.getGravityZ();
		// inverse pitch and roll angle for left-hand exo since MPU6050 chip is orientated differently between right and left-hand exo
		if (EXO_IN_USE == 1) {
			pitchAngDeg = -pitchAngDeg;
			rollAngDeg = -rollAngDeg;
		}
		// filter pitch and roll
		//pitchRollArray = myEWrist.averageIMUVar(pitchAngDeg, rollAngDeg); // takes about 50us to execute when 2 variables are averaged with a window length of 20
	#endif

	/////////////////////
	// VISUOMOTOR TASK //--------------------------------------------------------------
	/////////////////////
	#ifndef VMT_ENABLED
		myEWrist.setForceOffset(0.62);
		myEWrist.setVmtStatus("n");
	#endif

	#ifdef VMT_ENABLED
		// compensate for hand weight if condition c2 enabled
		if (myEWrist.getVmtStatus() == "c2") {
			// if angular velocity is positive (i.e. extension movement direction) => use gravity extension gain
			if (angVelWristDegSec > 0) {
				forceMeas = forceMeas - (myEWrist.getSubjectGainGravExt() * myEWrist.computeGravCompForce(pitchAngDeg, rollAngDeg, gravityZ, handWeight)); // normal IMU data
				//forceMeas = forceMeas - (myEWrist.getSubjectGainGravExt() * myEWrist.computeGravCompForce(pitchRollArray[0], pitchRollArray[1], handWeight)); // filtered IMU data
			}
			// if angular velocity is negative (i.e. flexion movement direction) => use gravity flexion gain
			else if (angVelWristDegSec < 0) {
				forceMeas = forceMeas - (myEWrist.getSubjectGainGravFlex() * myEWrist.computeGravCompForce(pitchAngDeg, rollAngDeg, gravityZ, handWeight)); // normal IMU data
				//forceMeas = forceMeas - (myEWrist.getSubjectGainGravFlex() * myEWrist.computeGravCompForce(pitchRollArray[0], pitchRollArray[1], handWeight)); // filtered IMU data
			}
		}

		// input percentage sEMG difference if condition c31 enabled
		if (myEWrist.getVmtStatus() == "c31") {
			// if pMVC difference is positive (i.e. extension movement intention) => use sEMG extension gain
			if (myEWrist.getVmtInComingString() > 0) {
				forceMeas = forceMeas - (myEWrist.getSubjectGainSEMGExt() * myEWrist.getVmtInComingString());
			}
			// if pMVC difference is negative (i.e. flexion movement intention) => use sEMG flexion gain
			else if (myEWrist.getVmtInComingString() < 0) {
				forceMeas = forceMeas - (myEWrist.getSubjectGainSEMGFlex() * myEWrist.getVmtInComingString());
			}
		}

		// substract extensor sEMG to force if condition c32e is enabled
		if (myEWrist.getVmtStatus() == "c32e") {
			forceMeas = forceMeas - (myEWrist.getSubjectGainSEMGExt() * myEWrist.getVmtInComingString());
		}
		// add flexor sEMG to force if condition c32f is enabled
		else if (myEWrist.getVmtStatus() == "c32f") {
			forceMeas = forceMeas + (myEWrist.getSubjectGainSEMGFlex() * myEWrist.getVmtInComingString());
		}
	#endif

	// get rid of small force variations around zero
	forceMeas = myEWrist.fixForceAroundZero(forceMeas);

	//////////////////////////////////////
	// ADMITTANCE PARAMETERS ADAPTATION //---------------------------------------------
	//////////////////////////////////////
	// if variable admittance parameters is enabled
	#ifdef VARIABLE_ADM_PARAM
		// skip LOOP2SKIP main loops to perform the frequency and magnitude analysis of the force oscillations
		if (mainLoopCounter == LOOP2SKIP) {
			// analyze frequency and magnitude of force oscillations
			idxForceFreqMag = myEWrist.analyzeForceFreqMag(forceMeas);  // takes about 310us when WIN_SIZE_FFM=250
			// compute index of instability of the force
			idxForceInstab = myEWrist.computeIndexInstab(idxForceFreqMag, lastIdxForceInstab);
			// compute amplification of index of instability
			ampIdxInstab = myEWrist.computeStiffFct(angPosEncDeg);
			// adjust admittance controller parameters according to the index of instability
			myEWrist.adjustAdmCtrlParameters(idxForceInstab, ampIdxInstab);

			lastIdxForceInstab = idxForceInstab;
			mainLoopCounter = 0;
			
			/*Serial.print(idxForceFreqMag[0]);
			Serial.print("\t");
			Serial.print(idxForceFreqMag[1]);
			Serial.print("\t");
			Serial.print(idxForceInstab);
			Serial.print("\t");
			Serial.println(ampIdxInstab);*/
		}
	#endif

	// deltaTimeCtrlLoopSec is fixed at 1kHz at the end of the void loop with the function "delayMicroseconds(sampleTimeCtrlLoop-deltaTimeCtrlLoop)"
	deltaTimeCtrlLoopSec = ((double)deltaTimeCtrlLoop * 0.000001);

	////////////////////////////////////
	// ADMITTANCE CONTROLLER - TUSTIN //-----------------------------------------------
	////////////////////////////////////
	// convert measured force to torque
	torqueWristDesired = myEWrist.convertForce2Torque(forceMeas);
	// apply Tustin transform and use deltaTimeCtrlLoopSec of previous control loop and not actual loop (does not make a big difference)
	angVelVirtWristRadSec = myEWrist.applyTustinTransform(torqueWristDesired, lastTorqueWristDesired, lastAngVelVirtWristRadSec, deltaTimeCtrlLoopSec);
	// convert angular virtual velocity of wrist to motor angular virtual velocity
	angVelVirtMotRadSec = myEWrist.convertAngVelWrist2Mot(angVelVirtWristRadSec);
	// convert rad/s to rpm
	angVelVirtMotRPM = myEWrist.convertRadSec2RPM(angVelVirtMotRadSec);
	// convert angular virtual velocity to control command for the motor
	controlCommand = angVelVirtMotRPM * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3; // #bits sent: [0-4095]

	//controlCommand = myEWrist.average(controlCommand);
	//controlCommand = 2100;

	// add EMG contribution to the control command
	//controlCommand = 0.4*controlCommand + 0.6 * 3*(EMGArrayTemp[1] - EMGArrayTemp[0]);
	//controlCommand = 3 * (EMGArrayTemp[1] - EMGArrayTemp[2]);

	// check control command range
	controlCommand = myEWrist.checkCtrlCmdLimits(controlCommand);

	///////////////////
	// ACTUATE MOTOR //----------------------------------------------------------------
	///////////////////
	// check status of debug mode on RPi0 and visuomotor task (VMT).
	// If debug mode disabled AND vmtStatus in normal mode ("n") OR condition c2 mode ("c2") OR condition c31 mode ("c31") OR condition c32e mode ("c32e") OR condition c32f mode ("c32f")
	if (digitalRead(gpio19Pin) == LOW && (myEWrist.getVmtStatus() == "n" || myEWrist.getVmtStatus() == "c2" || myEWrist.getVmtStatus() == "c31" || myEWrist.getVmtStatus() == "c32e" || myEWrist.getVmtStatus() == "c32f")) {
		// actuate motor according to controlCommand and in the predefined ROM. Do not execute actuateMotorInROM() if sensors are unconnected from forearm electronic module => otherwise it will crash!
		myEWrist.actuateMotorInROM(myADC, -controlCommand, enableCWPin, enableCCWPin, spdOrCurCtrPin, loadCellDownPin, loadCellUpPin, motCurPin, motSpdPin, posEncPin, batVltPin, batCurPin);
	}
	// If debug mode disabled AND vmtStatus in calibration mode ("c")
	else if (digitalRead(gpio19Pin) == LOW && myEWrist.getVmtStatus() == "c") {
		// actuate motor to target angle
		myEWrist.actuateMotor2Pos(1500, myEWrist.getVmtInComingString(), enableCWPin, enableCCWPin, spdOrCurCtrPin);
	}
	// If debug mode disabled AND vmtStatus in stop mode ("x")
	else if (digitalRead(gpio19Pin) == LOW && myEWrist.getVmtStatus() == "x") {
		// stop motor 
		myEWrist.stopMotor(enableCWPin, enableCCWPin, spdOrCurCtrPin);
	}
	// if debug mode enabled
	else if (digitalRead(gpio19Pin) == HIGH) {
		// actuate motor with buttons PROG1 and PROG2
		myEWrist.actuateMotorWithButton(enableCWPin, enableCCWPin, spdOrCurCtrPin, gpio13Pin, gpio16Pin);
		// move exo indefinitely in extension and flexion (for autonomy assessment)
		//directionFlag = myEWrist.actuateMotorExtFlex(enableCWPin, enableCCWPin, spdOrCurCtrPin, directionFlag);
		// move motor to defined position with buttons PROG1 and PROG2 (e.g. for step response)
		//myEWrist.actuateMotor2PosWithButton(6.0, angPosEncDeg, MAX_ANG_FLEX_A2P, MAX_ANG_EXT_A2P, enableCWPin, enableCCWPin, spdOrCurCtrPin, gpio13Pin, gpio16Pin);
		// assess static friction with buttons PROG1 and PROG2
		//myEWrist.assessStaticFriction(myADC, myPID, enableCWPin, enableCCWPin, spdOrCurCtrPin, loadCellDownPin, loadCellUpPin, motCurPin, motSpdPin, posEncPin, batVltPin, batCurPin, gpio13Pin, gpio16Pin);
		//myEWrist.assessStaticFrictionManual(myADC, enableCWPin, enableCCWPin, spdOrCurCtrPin, loadCellDownPin, loadCellUpPin, motCurPin, motSpdPin, posEncPin, batVltPin, batCurPin, gpio13Pin, gpio16Pin);
		//myEWrist.assessDynamicFriction(myADC, enableCWPin, enableCCWPin, spdOrCurCtrPin, loadCellDownPin, loadCellUpPin, motCurPin, motSpdPin, posEncPin, batVltPin, batCurPin, gpio13Pin, gpio16Pin);
		// if (fctExecuted == false) {
		// 	fctExecuted = myEWrist.assessPositionBandwidth(myADC, myPID, enableCWPin, enableCCWPin, spdOrCurCtrPin, loadCellDownPin, loadCellUpPin, motCurPin, motSpdPin, posEncPin, batVltPin, batCurPin, gpio13Pin, gpio16Pin);
		// }
		//targetAngleRad = myEWrist.followCMUEMG(myPID, EMGArrayTemp, targetAngleRad, speedCommand, enableCWPin, enableCCWPin, spdOrCurCtrPin);
		//myEWrist.tunePID(myADC, myPID, enableCWPin, enableCCWPin, spdOrCurCtrPin, loadCellDownPin, loadCellUpPin, motCurPin, motSpdPin, posEncPin, batVltPin, batCurPin, gpio13Pin, gpio16Pin);
		//Serial.println("In debug mode");
	}

	// check battery voltage and signal low voltage by blinking led2Debug to prevent battery damages. Also stop motor.
	myEWrist.checkBatteryVoltage(battVoltageMeas, enableCWPin, enableCCWPin, spdOrCurCtrPin, led2DebugPin);

	///////////////
	// SEND DATA //--------------------------------------------------------------------
	///////////////
	// send every sampleTimeSendLoop [us] over serial.
	deltaTimeSendLoop = (micros() - lastTimeSendLoop);
	if (deltaTimeSendLoop >= sampleTimeSendLoop) {
		lastTimeSendLoop = micros();  // put this initialisation before sending data since sendDataOverSerial50Hz()/60Hz() need from 500us to 600us to execute
		//myEWrist.sendDataOverSerial50Hz(angPosEncRad, forceMeas, angVelWristRadSec, motCurrentMeas, battVoltageMeas, battCurrentMeas);
		//myEWrist.sendDataOverSerial60Hz(deltaTimeSendLoop, angPosEncRad, forceMeas, angVelWristRadSec, pitchAngDeg, rollAngDeg, gravityZ); // in radian
		myEWrist.sendDataOverSerial60Hz(deltaTimeSendLoop, angPosEncDeg, forceMeas, angVelWristDegSec, pitchAngDeg, rollAngDeg, gravityZ); // in degree
		//myEWrist.sendDataOverSerial60Hz(deltaTimeSendLoop, angPosEncDeg, forceMeas, angVelWristDegSec, pitchRollArray[0], pitchRollArray[1], gravityZ); // in degree and IMU filtered
		//myEWrist.sendDataOverSerial60Hz(deltaTimeSendLoop, angPosEncRad, forceMeas, angVelWristRadSec, 1.0, 1.0, 1.0);
	}
	// !!!ATTENTION!!! For fast sending rate (i.e. 1kHz or close) => use control loop timing (sampleTimeCtrlLoop) with function sendDataOverSerial1kHz() placed just before the imposed delay at the end of the loop().

	///////////
	// DEBUG //------------------------------------------------------------------------
	///////////
	// print averaged data
	//average1.average(angVelWristRadSec*180/PI);
	//average2.average(motCurrentMeas);
	//average3.average(battCurrentMeas);

	// print every sampleTimePrintLoop [ms] on serial monitor. Must not be faster than 5ms otherwise serial monitor freezes!
	deltaTimePrintLoop = (millis() - lastTimePrintLoop);
	if (deltaTimePrintLoop >= sampleTimePrintLoop) {

		//Serial.println(forceMeas/9.81, 3);
		//Serial.print("\t");
		//Serial.print(EMGArrayTemp[0]);
		//Serial.print("\t");
		//Serial.print(EMGArrayTemp[1]);
		//Serial.print("\t");
		//Serial.println(EMGArrayTemp[1] - EMGArrayTemp[0]);
		//Serial.print("\t");
		//Serial.print(angPosEncRad, 3);
		//Serial.print('\t');
		//Serial.println(angPosEncDeg, 2);
		//Serial.print('\t');
		//Serial.println(angVelPosEncRadSec, 3);
		//Serial.print('\t');
		//Serial.println(angVelMotEncRadSec, 6);
		//Serial.print('\t');
		//Serial.println(angVelMotEncRPM, 3);
		//Serial.print('\t');
		//Serial.println(angVelWristRadSec*180/PI, 3);
		//Serial.print('\t');
		//Serial.print(angVelVirtWristRadSec, 6);
		//Serial.print('\t');
		//Serial.print(angVelVirtMotRPM, 6);
		//Serial.print('\t');
		//Serial.print(targetAngleRad, 3);
		//Serial.print('\t');
		//Serial.println(speedCommand, 3);
		//Serial.print('\t');
		//Serial.println(controlCommand);
		//Serial.print('\t');
		//Serial.print(motCurrentMeas, 3);
		//Serial.print('\t');
		//Serial.println(battVoltageMeas, 2);
		//Serial.print('\t');
		//Serial.println(battCurrentMeas, 2);
		//Serial.print('\t');
		//Serial.print(yawAngDeg, 2);
		//Serial.print('\t');
		//Serial.print(pitchAngDeg, 2);
		//Serial.print('\t');
		//Serial.println(rollAngDeg, 2);
		//Serial.print('\t');
		//Serial.print(pitchRollArray[0], 2);
		//Serial.print('\t');
		//Serial.println(pitchRollArray[1], 2);
		//Serial.println(gravityZ, 2);
		//Serial.print(myEWrist.getGravityGain());
		//Serial.print('\t');
		//Serial.println(myEWrist.getEMGForceGain());
		//Serial.print('\t');
		//Serial.println(myEWrist.computeGravCompForce(pitchAngDeg, rollAngDeg, 1), 3);
		//Serial.print('\t');
		//Serial.print(myEWrist.getVmtStatus());
		//Serial.print('\t');
		//Serial.println(myEWrist.getVmtInComingString());
		//Serial.print('\t');
		//Serial.println(forceOffset, 2);
		//Serial.print('\t');
		//Serial.println(handWeight, 2);
		//Serial.print('\t');
		//Serial.println(myEWrist.getSubjectGainGravExt());
		//Serial.print('\t');
		//Serial.println(myEWrist.getSubjectGainGravFlex());
		//Serial.print('\t');
		//Serial.println(myEWrist.getSubjectGainSEMGExt());
		//Serial.print('\t');
		//Serial.println(myEWrist.getSubjectGainSEMGFlex());
		//Serial.print('\t');
		//Serial.println(deltaTimeCtrlLoop);
		//Serial.print('\t');
		//Serial.println(deltaTimeCtrlLoopSec, 6);
		//Serial.print('\t');
		//Serial.println(ticTocDiff);
		//Serial.print('\t');
		//Serial.print(sampleTimeSendLoop);
		//Serial.print('\t');
		//Serial.print(deltaTimeSendLoop);
		//Serial.print('\t');
		//Serial.println(lastTimeSendLoop);
		//Serial.print('\t');
		//Serial.print(digitalRead(gpio13Pin));
		//Serial.print('\t');
		//Serial.print(digitalRead(gpio16Pin));
		//Serial.print('\t');
		//Serial.println(digitalRead(gpio19Pin));
		//Serial.print('\t');
		//Serial.println(directionFlag);
		//Serial.print(millis());
		//Serial.print('\t');
		//Serial.println(micros());

		lastTimePrintLoop = millis();
	}

	// send data at 1kHz (sampleTimeCtrlLoop) for transparency planes and step responses.
	//myEWrist.sendDataOverSerial1kHz(angPosEncRad, forceMeas, angVelWristRadSec, motCurrentMeas, battCurrentMeas);
	// send data at 1kHz (sampleTimeCtrlLoop) for stiffness function assessment.
	//myEWrist.sendDataOverSerial1kHz(angPosEncDeg-81.4, forceMeas, idxForceFreqMag[0], idxForceFreqMag[1], battCurrentMeas);

	// update counters and variables being integrated/derivated
	mainLoopCounter++;
	lastAngPosEncRad = angPosEncRad;
	myEWrist.setLastAngPosEncRad(lastAngPosEncRad);
	lastAngVelVirtWristRadSec = angVelVirtWristRadSec;
	lastTorqueWristDesired = torqueWristDesired;

	lastTimeCtrlLoop = startLoopUs;
	deltaTimeCtrlLoop = (micros() - lastTimeCtrlLoop);
	// fix the control loop timing at 1kHz by delaying it if needed
	if (deltaTimeCtrlLoop <= sampleTimeCtrlLoop) {
		delayMicroseconds(sampleTimeCtrlLoop - deltaTimeCtrlLoop);
	}
	// check how long does actually take one control loop without delay
	//Serial.println(deltaTimeCtrlLoop);
	// check how long does take one control loop with delay (should be 1000us => 1ms)
	//Serial.println(micros() - startLoopUs);

	// blink teensy LED to show normal execution (i.e. in control loop)
	digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
}
