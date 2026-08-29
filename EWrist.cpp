/*
EWrist.cpp
Author: Charles Lambelet
Created: February 2018
Modified: Marc Siegenthaler - June 2018
Modified: Charles Lambelet - March 2019
*/

/*
Hardware setup:
	Maxon motor:
		EN_CW						=>  pin D32
		EN_CCW					=>  pin D17
		SPD_CTR					=>  pin A14/DAC
		CUR_AVG					=>  pin A1
		SPD_AVG					=>  pin A0

	IMU:
		SDA							=>  pin D18/A4
		SCL							=>  pin D19/A5
		INT							=>  pin D33
		ADD							=>  GND through 6k7 resistor

	Load cell:
		LC_DOWN					=>  pin A19
		LC_UP						=>  pin A20

	Position encoder:
		POS_ENC					=>  pin A2

	Battery current sensor:
		CUR_SENS				=>  pin A16
		FAULT_EN				=>  pin D2

	Battery voltage:
		BATT_V_SENS			=>  pin A15

	RPi Zero:
		RPI_UART_TX			=>  pin D0/RX
		RPI_UART_RX			=>  pin D1/TX
		RPI_SPI_MOSI		=>  pin D11
		RPI_SPI_MISO		=>  pin D12
		RPI_SPI_SCLK		=>  pin D13
		RPI_SPI_CS			=>  pin D10
		GPIO13					=>  pin D8
		GPIO16					=>  pin D9
		GPIO19					=>  pin D25
		GPIO26					=>  pin D28

	Debug LEDs:
		LED_DEBUG1_TSY	=>	pin D7
		LED_DEBUG2_TSY	=>	pin D24
 */

#include "EWrist.h"

// Constructor. Initialize different variables
EWrist::EWrist() {
	angVelMeas = 0.0;						// [rad/s]
	angPosEncDeg = 0.0;       	// [deg]
	angPosEncRad = 0.0;       	// [rad]
	forceMeas = 0.0;          	// [N]
	forceOffset = 0.0;					// [N]
	batVltMeas = 0.0;						// [V]
	batCurMeas = 0.0;						// [A]
	mpuAccX = 0.0;            	// [m/s^2]
	mpuAccY = 0.0;            	// [m/s^2]
	mpuAccZ = 0.0;            	// [m/s^2]
	timerBlinkVoltage = 0;			// [ms]
	recordCounter = 0;
	vmtStatus = "x";						// "x" = stop motor
	//vmtInComingString = "1.34"; // [rad]
	vmtInComingString = "77";		// [deg]
	virtualMass = MIN_VIRT_MASS;
	virtualDamping = MIN_VIRT_DAMPING;
	fix_var_adm_param = true;
}

// Destructor.
EWrist::~EWrist() {

}

void EWrist::initADC(ADC *myADC) {
	//  adc->setReference(ADC_REF_1V2, ADC_0);      // set voltage reference for ADC_0
	//  adc->setReference(ADC_REF_3V3, ADC_1);      // set voltage reference for ADC_1

	myADC->setAveraging(4, ADC_0);                          // set number of averages
	myADC->setResolution(12, ADC_0);                        // set bits of resolution
	myADC->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_0);       // set the conversion speed
	myADC->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_0);         // set the sampling speed

	myADC->setAveraging(4, ADC_1);                          // set number of averages
	myADC->setResolution(12, ADC_1);                        // set bits of resolution
	myADC->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_1);       // set the conversion speed
	myADC->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_1);         // set the sampling speed

	// always call the compare functions after changing the resolution!
	//adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
	//adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V
}

void EWrist::collectADC(ADC *myADC, const int lCDP, const int lCUP, const int mCP, const int mSP, const int pEP, const int bVP, const int bCP) {
	loadCellDownADC = myADC->analogRead(lCDP, ADC_1);        // will return ADC_ERROR_VALUE if the comparison is false.
	loadCellUpADC = myADC->analogRead(lCUP, ADC_1);
	motCurADC = myADC->analogRead(mCP, ADC_0);
	motSpdADC = myADC->analogRead(mSP, ADC_0);
	posEncADC = myADC->analogRead(pEP, ADC_0);
	batVltADC = myADC->analogRead(bVP, ADC_1);
	batCurADC = myADC->analogRead(bCP, ADC_1);
/*
	// check and print error => errors are defined in ADC_Module.h
	if (myADC->adc0->fail_flag) {
		Serial.print("ADC0 error flags: 0x");
		Serial.println(myADC->adc0->fail_flag, HEX);
		if (myADC->adc0->fail_flag == ADC_ERROR::COMPARISON) {
			myADC->adc0->fail_flag &= ~ADC_ERROR::COMPARISON; // clear that error
			Serial.println("Comparison error in ADC0");
		}
	}
	if (myADC->adc1->fail_flag) {
		Serial.print("ADC1 error flags: 0x");
		Serial.println(myADC->adc1->fail_flag, HEX);
		if (myADC->adc1->fail_flag == ADC_ERROR::COMPARISON) {
			myADC->adc1->fail_flag &= ~ADC_ERROR::COMPARISON; // clear that error
			Serial.println("Comparison error in ADC1");
		}
	}
*/
}

void EWrist::initEMG() {
	EMGExtVal = 0.0;
	EMGFlexVal = 0.0;
}

// Initializing PID in EWrist class seems to solve some pointer problems! Otherwise teensy does not run through the whole loop() function!
void EWrist::initPID(PID *myPID) {
	myPID->initialize();
}

// Collect sEMG data from RPi for the adaptive controller implemented by Marc Siegenthaler
float *EWrist::collectEMGAdptCtrl(String incomingString) {
	// extract substrings from string
	stringLen = incomingString.length();
	commaIndex1 = incomingString.indexOf(',');
	commaIndex2 = incomingString.indexOf(',', commaIndex1 + 1);
	commaIndex3 = incomingString.indexOf(',', commaIndex2 + 1);
	subStrEMG3 = incomingString.substring(0, commaIndex1);
	subStrEMG4 = incomingString.substring(commaIndex1 + 1, commaIndex2);
	subStrEMG7 = incomingString.substring(commaIndex2 + 1, commaIndex3);
	subStrEMG8 = incomingString.substring(commaIndex3 + 1, stringLen);

	EMG3 = subStrEMG3.toFloat();
	EMG4 = subStrEMG4.toFloat();
	EMG7 = subStrEMG7.toFloat();
	EMG8 = subStrEMG8.toFloat();

	// to make sure EMG values are adapted only if incomming strings are correct:
	if (EMG3 < 2048 && EMG3 > 0 && EMG4 < 2048 && EMG4 > 0) {
		EMGExtVal = (EMG3 + EMG4) / 2;
	}
	if (EMG7 < 2048 && EMG7 > 0 && EMG8 < 2048 && EMG8 > 0) {
		EMGFlexVal = (EMG7 + EMG8) / 2;
	}

	EMGVal[0] = EMGExtVal;
	EMGVal[1] = EMGFlexVal;

	return EMGVal;
}

/* 
Collect sEMG data from CMU patients. These data are read and sent by the jupyter script read_send_cmu_data.ipynb.
The script sends two strings which correspond to the average of the two most activated channels in extension and the two most activated channels in flexion.
*/
float *EWrist::collectEMGCMU(String incomingString) {
	// extract substrings from string
	String emgValue;
	int commaIndex;
    int i;
    for (i=0; i<2; i++) {
      commaIndex = incomingString.indexOf(',');
      emgValue = incomingString.substring(0, commaIndex);
      incomingString = incomingString.substring(commaIndex + 1);
      EMGVal[i] = emgValue.toFloat();
    }

	return EMGVal;
}

/*
Collect data send by the Visuomotor task (VMT) => visuomotor_task.ipynb.
Data sent can be the force offset (f), the hand weight (h) or the sEMG as a percentage of MVC (e).
*/ 
void EWrist::collectDataVMT(String inComStrg) {
	int commaIndex;
	commaIndex = inComStrg.indexOf(',');
	vmtStatus = inComStrg.substring(0, commaIndex);
	vmtInComingString = inComStrg.substring(commaIndex + 1);

	// set force offset
	if (vmtStatus == "fo") {
		forceOffset = vmtInComingString.toFloat();
	}
	// set hand weight
	else if (vmtStatus == "hw") {
		mySubject.setHandWeight(vmtInComingString.toFloat());
	}
	// set mvc extensor
	else if (vmtStatus == "mvce") {
		mySubject.setMVCExtensor(vmtInComingString.toFloat());
	}
	// set mvc flexor
	else if (vmtStatus == "mvcf") {
		mySubject.setMVCFlexor(vmtInComingString.toFloat());
	}
	// set max active flexion angle
	else if (vmtStatus == "mafa") {
		mySubject.setMaxActFlexAngle(vmtInComingString.toFloat());
	}
	// set max active extension angle
	else if (vmtStatus == "maea") {
		mySubject.setMaxActExtAngle(vmtInComingString.toFloat());
	}
	// set max passive flexion angle
	else if (vmtStatus == "mpfa") {
		mySubject.setMaxPasFlexAngle(vmtInComingString.toFloat());
	}
	// set max passive extension angle
	else if (vmtStatus == "mpea") {
		mySubject.setMaxPasExtAngle(vmtInComingString.toFloat());
	}
	// set gravity extension gain
	else if (vmtStatus == "gge") {
		mySubject.setGainGravExt(vmtInComingString.toFloat());
	}
	// set gravity flexion gain
	else if (vmtStatus == "ggf") {
		mySubject.setGainGravFlex(vmtInComingString.toFloat());
	}
	// set sEMG extension gain
	else if (vmtStatus == "gse") {
		mySubject.setGainSEMGExt(vmtInComingString.toFloat());
	}
	// set sEMG flexion gain
	else if (vmtStatus == "gsf") {
		mySubject.setGainSEMGFlex(vmtInComingString.toFloat());
	}
	// set force stiffness function gain
	else if (vmtStatus == "gfsf") {
		mySubject.setGainStiffFct(vmtInComingString.toFloat());
	}
	// enable/disable force stiffness function
	else if (vmtStatus == "edfsf") {
		fix_var_adm_param = vmtInComingString.toFloat();
	}
	// set stiffness function parameters
	mySubject.setStiffFctPara(vmtStatus, vmtInComingString);

	// set percentage MVC difference
	//else if (vmtStatus == "c3") {
	//	mySubject.setPMVCDiff(vmtInComingString.toFloat());
	//}
}

/* 
Compute ADC and transform them to actual measurements unit.
*/
void EWrist::computeADC() {
	// load cell #5 with upperarm board #2. => right-handed exo
	#ifdef LC5B2_IN_USE
		// convert load cell ADC into newton
		forceMeas = convertADC2NewtonLC5B2(loadCellDownADC, loadCellUpADC, RECI_A_DOWN_LC5_BRD2, B_DOWN_LC5_BRD2, RECI_A_UP_LC5_BRD2, B_UP_LC5_BRD2, A_ALPHA_LC5_BRD2, B_ALPHA_LC5_BRD2, A_BETA_LC5_BRD2, B_BETA_LC5_BRD2, ADC_LIM_ONLY_DOWN_LC5_BRD2, ADC_LIM_ONLY_UP_LC5_BRD2);
		// convert position encoder ADC into degree
		angPosEncDeg = convertADC2Degree(posEncADC, ADC_POS_ENC_R1_MIN, ADC_POS_ENC_R1_MAX); // position encoder R1 (right-handed exo)
		// convert battery voltage ADC into volt
		batVltMeas = convertADC2VoltBat(batVltADC, RECI_A_VLT_BRD2, B_VLT_BRD2); // upperarm board #2
		// convert battery current ADC into ampere
		batCurMeas = convertADC2AmpereBat(batCurADC, RECI_A_CUR_BRD2, B_CUR_BRD2); // upperarm board #2
	#endif

	// load cell #13 with upperarm board #1. => left-handed exo
	#ifdef LC13B1_IN_USE
		// convert load cell ADC into newton. Since load cell #13 is for a left-hand exo, loadCellDownADC and loadCellUpADC should be inverted compared to right-handed exo.
		forceMeas = convertADC2NewtonLC13B1(loadCellUpADC, loadCellDownADC, RECI_A_DOWN_HEAVY_LC13_BRD1, B_DOWN_HEAVY_LC13_BRD1, RECI_A_UP_HEAVY_LC13_BRD1, B_UP_HEAVY_LC13_BRD1, RECI_A_UP_LIGHT_LC13_BRD1, B_UP_LIGHT_LC13_BRD1, ADC_LIM_ONLY_DOWN_HEAVY_LC13_BRD1, ADC_LIM_ONLY_UP_HEAVY_LC13_BRD1, ADC_LIM_ONLY_UP_LIGHT_LC13_BRD1);
		// convert position encoder ADC into degree. MAX and MIN inverted compared to right since left-handed exo.
		angPosEncDeg = convertADC2Degree(posEncADC, ADC_POS_ENC_L1_MAX, ADC_POS_ENC_L1_MIN); // position encoder L1 (left-handed exo).
		// convert battery voltage ADC into volt
		batVltMeas = convertADC2VoltBat(batVltADC, RECI_A_VLT_BRD1, B_VLT_BRD1); // upperarm board #1
		// convert battery current ADC into ampere
		batCurMeas = convertADC2AmpereBat(batCurADC, RECI_A_CUR_BRD1, B_CUR_BRD1); // upperarm board #1
	#endif
	
	angPosEncRad = deg2Rad(angPosEncDeg);

	// convert motor velocity encoder ADC into rad/sec
	angVelMotEncRPM = convertADC2RPMMot(motSpdADC);
	angVelMotEncRadSec = convertRPM2RadSec(angVelMotEncRPM);

	// convert motor current ADC into ampere
	motCurMeas = convertADC2AmpereMot(motCurADC);

	// average collected data for calibration
	//average(loadCellDownADC);
	//average(loadCellUpADC);
	//average(posEncADC);
	//average(batVltADC);
	//average(batCurADC);

	//Serial.print(loadCellDownADC);
	//Serial.print("\t");
	//Serial.print(loadCellUpADC);
	//Serial.print("\t");
	//Serial.println(posEncADC);
	//Serial.print("\t");
	//Serial.println(batVltADC);
}

// !!!ATTENTION!!! This function takes about 400us to be executed (15 concatenated strings with eol)!
void EWrist::sendDataOverSerial50Hz(float aPosEnc, float frcMeas, float aVelWrist, float mCurMeas, float bVltMeas, float bCurMeas) {
	const String separator = ",";
	const String eol = "\n";

	// get time when data are collected and sent. Precise timing between data measurements is important for derivation!!!
	String strTimeStamp = "";
	strTimeStamp = String(micros());

	String dataString;
	dataString.concat(strTimeStamp);
	dataString.concat(separator);
	dataString.concat(aPosEnc);
	dataString.concat(separator);
	dataString.concat(frcMeas);
	dataString.concat(separator);
	dataString.concat(aVelWrist);
  dataString.concat(separator);
	dataString.concat(mCurMeas);
	dataString.concat(separator);
	dataString.concat(bVltMeas);
	dataString.concat(separator);
	dataString.concat(bCurMeas);
	dataString.concat(separator);

	dataString.concat(eol);
	Serial.print(dataString);
}

// !!!ATTENTION!!! This function takes about 520us to be executed (19 concatenated strings with eol)!
void EWrist::sendDataOverSerial60Hz(unsigned int dTimeSendLoop, float aPosEnc, float frcMeas, float aVelWrist, float pitchAng, float rollAng, float gravZ) {
	const String separator = ",";
	const String eol = "\n";

	// get time when data are collected and sent. Precise timing between data measurements is important for derivation!!!
	String strTimeStamp = "";
	strTimeStamp = String(micros());

	String dataString;
	dataString.concat(recordCounter);
  dataString.concat(separator);
	dataString.concat(strTimeStamp);
	dataString.concat(separator);
	dataString.concat(dTimeSendLoop);
	dataString.concat(separator);
	dataString.concat(aPosEnc);
	dataString.concat(separator);
	dataString.concat(frcMeas);
	dataString.concat(separator);
	dataString.concat(aVelWrist);
  dataString.concat(separator);
	dataString.concat(pitchAng);
  dataString.concat(separator);
	dataString.concat(rollAng);
  dataString.concat(separator);
	dataString.concat(gravZ);
  dataString.concat(separator);

	dataString.concat(eol); // needed for newline in terminal visualization
	Serial.print(dataString);
	recordCounter += 1;
}

// !!!ATTENTION!!! This function takes about 400us to be executed (13 concatenated strings with eol)!
void EWrist::sendDataOverSerial1kHz(float aPosEnc, float frcMeas, float aVelWrist, float mCurMeas, float bCurMeas) {
	const String separator = ",";
	const String eol = "\n";

	// get time when data are collected and sent. Precise timing between data measurements is important for derivation!!!
	String strTimeStamp = "";
	strTimeStamp = String(micros());

	String dataString;
	dataString.concat(recordCounter);
  dataString.concat(separator);
	dataString.concat(strTimeStamp);
	dataString.concat(separator);
	dataString.concat(aPosEnc);
	dataString.concat(separator);
	dataString.concat(frcMeas);
	dataString.concat(separator);
	dataString.concat(aVelWrist);
	dataString.concat(separator);
	dataString.concat(mCurMeas);
	dataString.concat(separator);
	// dataString.concat(bCurMeas);
	// dataString.concat(separator);

	dataString.concat(eol);
	Serial.print(dataString);
	recordCounter += 1;
}

/*
Two different amplifications (DOWN direction and UP direction) are used to convert the ADC from the load cell into Newton.
Each amplification direction has its own linear relation => aDown, bDown and aUp, bUp are the parameters of these two linear relations.
According to the load cell and upperarm board (amplification) used, the transition rule between DOWN and UP differs.
Here, for load cell #5 and upper board #2, in the transition between DOWN and UP, the force is calculated as a combination of both amplifications => force = alpha*forceDown + beta*forceUp.
*/
float EWrist::convertADC2NewtonLC5B2(int lCDownADC, int lCUpADC, float reciADown, float bDown, float reciAUp, float bUp, float aAlpha, float bAlpha, float aBeta, float bBeta, int limOnlyDown, int limOnlyUp) {
	// linear relations between ADC and force (in Newton) for both DOWN and UP directions
	// forceDown = (lCDownADC - bDown) / aDown;
	float forceDown = (lCDownADC - bDown) * reciADown;
	// forceUp = (lCUpADC - bUp) / aUp;
	float forceUp = (lCUpADC - bUp) * reciAUp;
	float force = 0.0;
	float alpha = aAlpha * lCDownADC + bAlpha;
	float beta = aBeta * lCDownADC + bBeta;

	// according to load cell and upperarm board (amplification) used, decide which direction to use to compute the force.
	if (lCDownADC > limOnlyDown) {
		force = forceDown;
	}
	// in the transition between DOWN and UP. The "<= and ">=" are important otherwise if loadCellDownADC = 550 then force = -forceUp!
	else if (lCDownADC <= limOnlyDown && lCDownADC >= limOnlyUp) {
		forceUp = -forceUp;	// in this part (where forceUp is negative) we want forceUp to be positive
		force = alpha * forceDown + beta * forceUp;
	}
	// else use only UP direction
	else {
		force = -forceUp;
	}
	return force;
}

/*
Here, for load cell #13 and upper board #1, in the transition between DOWN and UP, the force is calculated as a mixture of 3 different linear relations:
1) A linear relation for heavy weigths (from 0.5 to 3kg) in the DOWN direction.
2) A linear relation for light weigths (from 0 to 0.5kg, i.e. around transition between DOWN and UP) in UP direction.
3) A linear relation for heavy weights (from 0.5 to 3kg) in UP direction.
The linear relation for light weights in DOWN direction is not needed here since the linear relation for heavy weights is very similar.
*/
float EWrist::convertADC2NewtonLC13B1(int lCDownADC, int lCUpADC, float reciADownH, float bDownH, float reciAUpH, float bUpH, float reciAUpL, float bUpL, int limOnlyDownH, int limOnlyUpH, int limOnlyUpL) {
	// linear relations between ADC and force (in Newton) for both DOWN & UP directions and HEAVY & LIGHT weights
	// forceDownHeavy = (lCDownADC - bDownH) / aDownH;
	float forceDownHeavy = (lCDownADC - bDownH) * reciADownH;
	// forceUpHeavy = (lCUpADC - bUpH) / aUpH;
	float forceUpHeavy = (lCUpADC - bUpH) * reciAUpH;
	// forceDownLight = (lCDownADC - bDownL) / aDownL;
	//float forceDownLight = (lCDownADC - bDownL) * reciADownL;
	// forceUpLight = (lCUpADC - bUpL) / aUpL;
	float forceUpLight = (lCUpADC - bUpL) * reciAUpL;
	float force = 0.0;

	// according to load cell and upperarm board (amplification) used, decide which direction to use to compute the force.
	// if lCDownADC is greater than limOnlyDownH (288) use DOWN-HEAVY amplification
	if (lCDownADC > limOnlyDownH) {
		force = forceDownHeavy;
	}
	//
	// else if lCUpADC is between limOnlyUpL (160) and limOnlyUpH (652) use UP-LIGHT amplification
	else if (lCUpADC > limOnlyUpL && lCUpADC < limOnlyUpH) {
		force = -forceUpLight;
	}
	// else if lCUpADC is greater than limOnlyUpH (652) use UP-HEAVY amplification
	else if (lCUpADC >= limOnlyUpH) {
		force = -forceUpHeavy;
	}
	else {
		force = 0.0;
	}
	return force;
}

float EWrist::convertADC2Degree(int pEncADC, int pEncMin, int pEncMax) {
	// (pEncMax = -77° and pEncMin = +77° => impose the 0° at pEncMin (+77°) and the 154° at pEncMax (-77°))
	return (float)(pEncADC - pEncMin) * (154 / (float)(pEncMax - pEncMin));
}

float EWrist::convertADC2RPMMot(int mSpdADC) {
	// ((((float) mSpdADC) * 3.3 / 4095) - 1.65 - NOISE_OFFSET)*(30000 / 3.3);
	return ((((float)mSpdADC) * LOGIC_3V3_LEV * RECI_12_BITS) - 1.65 - NOISE_OFFSET) * ((ESCON_GET_SPD_AT_3V3 - ESCON_GET_SPD_AT_0V) * RECI_LOGIC_3V3_LEV);
}

float EWrist::convertADC2AmpereMot(int mCurADC) {
	// ((((float) mCurADC) * 3.3 / 4095) - 1.65 - NOISE_OFFSET)*(6 / 3.3);
	return ((((float)mCurADC) * LOGIC_3V3_LEV * RECI_12_BITS) - 1.65 - NOISE_OFFSET) * ((ESCON_GET_CUR_AT_3V3 - ESCON_GET_CUR_AT_0V) * RECI_LOGIC_3V3_LEV);
}

float EWrist::convertADC2VoltBat(int bVltADC, float reciAVlt, float bVlt) {
	// (bVltADC - bVlt) / aVlt;
	return (bVltADC - bVlt) * reciAVlt;
}

float EWrist::convertADC2AmpereBat(int bCurADC, float reciACur, float bCur) {
	// (batteryCurrentADC - bCur) / aCur;
	return (bCurADC - bCur) * reciACur;
}

float EWrist::convertRPM2RadSec(float angVelRPM) {
	// angVelRPM * 2 * PI / 60;
	return angVelRPM * 0.0166666 * TWO_PI;
}

float EWrist::convertRadSec2RPM(float angVelRadSec) {
	// angVelRadSec * 60 / 2 * PI;
	return angVelRadSec * 60 * RECI_TWO_PI;
}

float EWrist::convertAngVelMot2Wrist(float angVelMot) {
	return angVelMot * RECI_GEARDRIVE_RATIO;
}

float EWrist::convertAngVelWrist2Mot(float angVelWrist) {
	return angVelWrist * GEARDRIVE_RATIO;
}

float EWrist::convertForce2Torque(float force) {
	return force * LENGTH;
}

float EWrist::deg2Rad(float angDeg) {
	return angDeg * DEG_TO_RAD; // [rad]
}

float EWrist::rad2Deg(float angRad) {
	return angRad * RAD_TO_DEG; // [deg]
}

float EWrist::fixForceAroundZero(float force) {
	if (force < DEAD_ZONE_FORCE_MEAS  && -force < DEAD_ZONE_FORCE_MEAS) {
		force = 0.0;
	}
	else {
		force = force;
	}
	return force;
}

/*
Analyze the frequency of changes of the force sign and the magnitude of the force during instable oscillations of the handle.
Takes about 310us to be executed if WIN_SIZE_FFM=250
*/
float *EWrist::analyzeForceFreqMag(float forceMeas) {
	int forceSignChange = 1;
	float sumForceSignChange = 0;
	float sumForceMagSquared = 0.0;
	float forceMagRMS = 0.0;

	if (forceMeas > 0) {
		forceMeasSign = true;
	}
	else if (forceMeas < 0) {
		forceMeasSign = false;
	}
	// if force sign did NOT change after a loop => assign it to 0, else it stays 1 as initialized
	if (forceMeasSign == lastForceMeasSign) {
		forceSignChange = 0;
	}

	// copy blocks of memory. takes about 37us for both memcpy functions when WIN_SIZE_FFM=400.
	// void * memcpy (void * destination, const void * source, size_t num)
	memcpy(forceSignChangeArrayTemp, forceSignChangeArray, WIN_SIZE_FFM * sizeof(int));
	memcpy(forceMagSquaredArrayTemp, forceMagSquaredArray, WIN_SIZE_FFM * sizeof(int));
	// loop over both arrays. takes about 400us when WIN_SIZE_FFM=400 and 260us when WIN_SIZE_FFM=250
	for (int i=0; i<WIN_SIZE_FFM; i++)
	{
		// sum number of force sign changes found in array. The array corresponds to an epoch of approx. 2s if WIN_SIZE_FFM=500 and if 4 mainloop out of 5 are skipped
		sumForceSignChange += forceSignChangeArrayTemp[i];
		// sum the squared magnitudes of the force
		sumForceMagSquared += forceMagSquaredArrayTemp[i];
		// put a condition to not exceed the array size
		if (i<(WIN_SIZE_FFM-1))
		{
			// shift all values and drop the last value of the array
			forceSignChangeArray[i+1] = forceSignChangeArrayTemp[i];
			forceMagSquaredArray[i+1] = forceMagSquaredArrayTemp[i];
		}
	}
	// put the new sign change at the first place of the array
	forceSignChangeArray[0] = forceSignChange;
	// put the new squared force at the first place of the array
	forceMagSquaredArray[0] = forceMeas*forceMeas;
	// normalize the number of force sign changes to the maximal number of force sign changes
	sumForceSignChange = sumForceSignChange/MAX_FORCE_FREQ;
	// compute root mean square (RMS) of force magnitude and normalize it to the maximal force magnitude
	forceMagRMS = (sqrt(sumForceMagSquared/WIN_SIZE_FFM)) / MAX_FORCE_MAG;  // takes about 20us
	// update last force sign
	lastForceMeasSign = forceMeasSign;

	idxForceFreqMag[0] = sumForceSignChange;
	idxForceFreqMag[1] = forceMagRMS;

	// return the frequency of sign changes and magnitude of force
	return idxForceFreqMag;
}

/*
Compute the index of instability of the force during unwanted oscillations of the handle.
The index is based on the frequency and magnitude of the force oscillations but also on the past index.
*/
float EWrist::computeIndexInstab(float *idxForceFreqMag, float lastIdxForceInstab) {
	return idxForceFreqMag[0]*idxForceFreqMag[1] + LAMBDA*lastIdxForceInstab;
}

/*
Compute the amplification which multiplies the index of instability, and which is based on the wrist angular position.
The more flexed or extended is the wrist, the larger the influence of the index.
The amplification follows a non linear relationship generated during wirst impedance calibration.
*/
float EWrist::computeStiffFct(float aPosEnc) {
	// get parameters of the stiffness function
	float *stiffFctPara = mySubject.getStiffFctPara();
	// assign variables to locals
	float a1 = stiffFctPara[0];
	float b1 = stiffFctPara[1];
	float c1 = stiffFctPara[2];
	float x2 = stiffFctPara[3];
	float a2 = stiffFctPara[4];
	float b2 = stiffFctPara[5];
	float x3 = stiffFctPara[6];
	float a3 = stiffFctPara[7];
	float b3 = stiffFctPara[8];
	float c3 = stiffFctPara[9];
	float x = aPosEnc;
	float y = 0;

	// if in 1st parabola regime
	if (x < x2) {
		y = a1*x*x + b1*x + c1;
	}
	// if in line regime
	else if (x > x2 && x < x3) {
		y = a2*x + b2;
	}
	// if in 2nd parabola regime
	else if (x > x3) {
		y = a3*x*x + b3*x + c3;
	}

	// return the absolute value of the amplification and ensure it is never null (+1)
	return abs(y) + 1;
}

/*
Adjust the admittance controller parameters (virtual mass and damping) based on the index of instability of the force.
*/
void EWrist::adjustAdmCtrlParameters(float idxForceInstab, float ampIdxInstab) {
	// only increase virtual mass and keep virtual damping constant
	if (MASS_INCREASE) {
		virtualMass = MIN_VIRT_MASS + mySubject.getGainStiffFct() * ampIdxInstab * idxForceInstab;
		virtualDamping = MIN_VIRT_DAMPING;
	}
	// only increase virtual damping and keep virtual mass constant
	else if (DAMPING_INCREASE) {
		virtualMass = MIN_VIRT_MASS;
		virtualDamping = MIN_VIRT_DAMPING + mySubject.getGainStiffFct() * ampIdxInstab * idxForceInstab;
	}
	// increase virtual mass and damping with constant ratio
	else if (MASS_DAMPING_INCREASE) {
		virtualMass = MIN_VIRT_MASS + mySubject.getGainStiffFct() * ampIdxInstab * idxForceInstab;
		virtualDamping = (MIN_VIRT_DAMPING/MIN_VIRT_MASS) * virtualMass;
	}
	//Serial.print(idxForceInstab, 3);
	//Serial.print("\t");
	//Serial.print(virtualMass, 3);
	//Serial.print("\t");
	//Serial.println(virtualDamping, 3);
}

/*
Discretize the command with Tustin method.
*/
float EWrist::applyTustinTransform(float trqDsrd, float lastTrqDsrd, float lastAngVelVrt, double deltaTimeSec) {
	/*
	// with fixed virtual mass and damping
	#ifdef FIXED_ADM_PARAM
		float tustinNum = (trqDsrd + lastTrqDsrd) * deltaTimeSec + (2 * MIN_VIRT_MASS - MIN_VIRT_DAMPING * deltaTimeSec) * lastAngVelVrt;
		float tustinDenom = 2 * MIN_VIRT_MASS + MIN_VIRT_DAMPING * deltaTimeSec;
	#endif
	// with variable virtual mass and damping
	#ifdef VARIABLE_ADM_PARAM
		float tustinNum = (trqDsrd + lastTrqDsrd) * deltaTimeSec + (2 * virtualMass - virtualDamping * deltaTimeSec) * lastAngVelVrt;
		float tustinDenom = 2 * virtualMass + virtualDamping * deltaTimeSec;
	#endif
	*/
	float tustinNum = 0.0;
	float tustinDenom = 0.0;
	// with fixed virtual mass and damping
	if (!fix_var_adm_param) {
		tustinNum = (trqDsrd + lastTrqDsrd) * deltaTimeSec + (2 * MIN_VIRT_MASS - MIN_VIRT_DAMPING * deltaTimeSec) * lastAngVelVrt;
		tustinDenom = 2 * MIN_VIRT_MASS + MIN_VIRT_DAMPING * deltaTimeSec;
	}
	// with variable virtual mass and damping
	else if (fix_var_adm_param) {
		tustinNum = (trqDsrd + lastTrqDsrd) * deltaTimeSec + (2 * virtualMass - virtualDamping * deltaTimeSec) * lastAngVelVrt;
		tustinDenom = 2 * virtualMass + virtualDamping * deltaTimeSec;
	}

	return tustinNum / tustinDenom; // [rad/s] angular velocity at the wrist since torque is measured at the wrist
}

/*
Check the control command to not exceed exessive speed.
*/
float EWrist::checkCtrlCmdLimits(float ctrlCmd) {
	if (ctrlCmd > MAX_SPD_CMD * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3) {
		ctrlCmd = MAX_SPD_CMD * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3;
	}
	else if (ctrlCmd < MIN_SPD_CMD * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3) {
		ctrlCmd = MIN_SPD_CMD * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3;
	}
	else {
		ctrlCmd = ctrlCmd;
	}
	return ctrlCmd;
}

/*
This function computes the amount of hand weight that needs to be compensated according to IMU readouts and the angular position of the wrist.
*/
float EWrist::computeGravCompForce(float pitchAngDeg, float rollAngDeg, float gravityZ, float handWeight) {
	float compForce = 0;
	// if eWrist is NOT upside down
	if (gravityZ > 0) {
		compForce = handWeight * cos(pitchAngDeg*DEG_TO_RAD) * cos(rollAngDeg*DEG_TO_RAD) * cos(angPosEncRad - 1.34);
	}
	// if eWrist is upside down
	else {
		compForce = -handWeight * cos(pitchAngDeg*DEG_TO_RAD) * cos(rollAngDeg*DEG_TO_RAD) * cos(angPosEncRad - 1.34);
	}
	return compForce;
}

/*
This function checks the battery voltage.
*/
void EWrist::checkBatteryVoltage(float voltage, const int enCWP, const int enCCWP, const int sCCP, const int l2DP) {
	unsigned int timerTemp = millis() - timerBlinkVoltage;
	// if battery voltage lower than limit and timer elapsed => blink LED
	if (voltage < LOW_VOLTAGE_LIPO_3S && timerTemp > 200) {
		digitalWrite(l2DP, !digitalRead(l2DP));
		timerBlinkVoltage = millis();
	}
	// if battery voltage lower than limit => stop motor
	else if (voltage < LOW_VOLTAGE_LIPO_3S) {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
	}
	// if battery voltage higher than limit => switch off LED
	else if (voltage >= LOW_VOLTAGE_LIPO_3S) {
		digitalWrite(l2DP, LOW);
	}
}

/*
This function stops the motor.
*/
void EWrist::stopMotor(const int enCWP, const int enCCWP, const int sCCP) {
	digitalWrite(enCWP, LOW);
	digitalWrite(enCCWP, LOW);
	analogWrite(sCCP, 0);
}

/*
This function actuates the motor in predefined ROM.
*/
void EWrist::actuateMotorInROM(ADC *myADC, float ctrlCmd, const int enCWP, const int enCCWP, const int sCCP, const int lCDP, const int lCUP, const int mCP, const int mSP, const int pEP, const int bVP, const int bCP) {
	// if extension angle limit reached, run motor in flexion direction
	if (angPosEncRad > MAX_ANG_EXT) {
		while (angPosEncRad > (MAX_ANG_EXT-0.02)) {
			digitalWrite(enCWP, HIGH);
			digitalWrite(enCCWP, LOW);
			analogWrite(sCCP, 1000 * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
			// update ADC variables
			collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
			computeADC();
			averageADCVar();
		}
	}
	// if flexion angle limit reached, run motor in extension direction
	else if (angPosEncRad < MAX_ANG_FLEX) {
		while (angPosEncRad < (MAX_ANG_FLEX+0.02)) {
			digitalWrite(enCWP, LOW);
			digitalWrite(enCCWP, HIGH);
			analogWrite(sCCP, 1000 * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
			// update ADC variables
			collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
			computeADC();
			averageADCVar();
		}
	}
	// else run motor according to controller command when out of dead zone
	else {
		// run motor in clockwise direction
		if (ctrlCmd < -DEAD_ZONE_CMD * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3) {
			digitalWrite(enCWP, HIGH);
			digitalWrite(enCCWP, LOW);
			analogWrite(sCCP, -ctrlCmd); // always send positive values
		}
		// run motor in counterclockwise direction
		else if (ctrlCmd > DEAD_ZONE_CMD * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3) {
			digitalWrite(enCWP, LOW);
			digitalWrite(enCCWP, HIGH);
			analogWrite(sCCP, ctrlCmd);
		}
		// if you are in the DEAD_ZONE_CMD => do nothing
		else {
			digitalWrite(enCWP, LOW);
			digitalWrite(enCCWP, LOW);
			analogWrite(sCCP, 0);
		}
	}
}

/*
This function actuates the motor until a given angular position is reached.
*/
void EWrist::actuateMotor2Pos(float cmd, float ang2ReachDeg, const int enCWP, const int enCCWP, const int sCCP) {
	// run motor in flexion direction
	if (angPosEncDeg > ang2ReachDeg + 0.8) {
		digitalWrite(enCWP, HIGH);
		digitalWrite(enCCWP, LOW);
		#ifdef SPEED_CONTROL
		analogWrite(sCCP, cmd * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
		#endif
		#ifdef CURRENT_CONTROL
			analogWrite(sCCP, cmd * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
	}
	// run motor in extension direction
	else if (angPosEncDeg < ang2ReachDeg - 0.8) {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, HIGH);
		#ifdef SPEED_CONTROL
		analogWrite(sCCP, cmd * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
		#endif
		#ifdef CURRENT_CONTROL
			analogWrite(sCCP, cmd * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
	}
	// if angle reached => stop motor
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
	}
}

/*
This function actuates the motor indefinitely when buttons are pressed. Repress the button to stop actuation.
Be careful this function can actuate the motor out of predefined ROM!
*/
void EWrist::actuateMotorWithButton(const int enCWP, const int enCCWP, const int sCCP, const int gpio13P, const int gpio16P) {
	// spin motor in CW direction (i.e. flexion direction) when PROG1 button is pressed
	if (digitalRead(gpio13P) == HIGH && digitalRead(gpio16P) == LOW) {
		digitalWrite(enCWP, HIGH);
		digitalWrite(enCCWP, LOW);
		#ifdef SPEED_CONTROL
			analogWrite(sCCP, 2500 * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
		#endif
		#ifdef CURRENT_CONTROL
			analogWrite(sCCP, 0.3 * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
	}
	// spin motor in CCW direction (i.e. extension direction) when PROG2 button is pressed
	else if (digitalRead(gpio13P) == LOW && digitalRead(gpio16P) == HIGH) {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, HIGH);
		#ifdef SPEED_CONTROL
			analogWrite(sCCP, 2500 * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
		#endif
		#ifdef CURRENT_CONTROL
			analogWrite(sCCP, 0.3 * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
	}
	// do not spin motor
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
	}
}

/*
This function actuates the motor to predefined angular positions when buttons are pressed (e.g. used for step response).
*/
void EWrist::actuateMotor2PosWithButton(float command, float aPosEncDeg, float angle2ReachFlex, float angle2ReachExt, const int enCWP, const int enCCWP, const int sCCP, const int gpio13P, const int gpio16P) {
	// spin motor in CW direction until angle is reached
	if (digitalRead(gpio13P) == HIGH && digitalRead(gpio16P) == LOW && aPosEncDeg > angle2ReachFlex) {
		digitalWrite(enCWP, HIGH);
		digitalWrite(enCCWP, LOW);
		#ifdef SPEED_CONTROL
			analogWrite(sCCP, command * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
		#endif
		#ifdef CURRENT_CONTROL
			analogWrite(sCCP, command * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
	}
	// spin motor in CCW direction until angle is reached
	else if (digitalRead(gpio13P) == LOW && digitalRead(gpio16P) == HIGH && aPosEncDeg < angle2ReachExt) {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, HIGH);
		#ifdef SPEED_CONTROL
			analogWrite(sCCP, command * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
		#endif
		#ifdef CURRENT_CONTROL
			analogWrite(sCCP, command * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
	}
	// do not spin motor
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
	}
}

/*
This function actuates the motor in current control to assess the static friction in the transmission.
Press button 1 to move motor to initial position.
Press button 2 to start the friction assessment where the current is gently increased until the handles moves.
Once it has moves it records the current and move the handle 5deg further for a next static friction assessment.
It can assess the static friction in both directions (i.e. from extension to flexion or from flexion to extension).
*/
void EWrist::assessStaticFriction(ADC *myADC, PID *myPID, const int enCWP, const int enCCWP, const int sCCP, const int lCDP, const int lCUP, const int mCP, const int mSP, const int pEP, const int bVP, const int bCP, const int gpio13P, const int gpio16P) {
	// spin motor in a defined direction until starting angle is reached
	if (digitalRead(gpio13P) == HIGH && digitalRead(gpio16P) == LOW && angPosEncRad > (MAX_ANG_FLEX-0.05)) {
		// initialize PID controller to reach desired position
		myPID->setVariables(&angPosEncRad, &currentCommand, &targetAngleRad);	// set variables
		myPID->setTunings(10.0, 0.01, 0.0);									// set tuning parameters, i.e. kp, ki, kd
		myPID->setLimits(-0.4, 0.4, -5, 5);									// set min and max limits. Choose the minimum current (here 0.3A) which actuates the exo on the whole range and then tune the parameters. 
		myPID->compute();
		while (abs(myPID->getCurErr()) > 0.01) {
			myPID->compute();
			// run motor in flexion direction
			if (currentCommand < 0) {
				digitalWrite(enCWP, HIGH);
				digitalWrite(enCCWP, LOW);
			}
			// run motor in extension direction
			else if (currentCommand > 0) {
				digitalWrite(enCWP, LOW);
				digitalWrite(enCCWP, HIGH);
			}
			#ifdef CURRENT_CONTROL
				analogWrite(sCCP, abs(currentCommand) * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
			#endif
			// update ADC variables (i.e. angPosEncRad)
			collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
			computeADC();
			averageADCVar();
		}
		// initialize targetAngleRad to start angle
		targetAngleRad = MAX_ANG_FLEX;
		Serial.println("Ready to start static friction assessment!");
	}
	// increase motor current until handle moves and then move to next angle step
	else if (digitalRead(gpio13P) == LOW && digitalRead(gpio16P) == HIGH && angPosEncRad < MAX_ANG_EXT) {
		// run motor in extension direction
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, HIGH);
		// reinitialize current command and lastAngPosEncRad so that the difference between angPosEncRad is null
		currentCommand = 0.0;
		lastAngPosEncRad = angPosEncRad;
		//lastAngPosEncRad = angPosEncRad;
		Serial.println("print_1:");
		Serial.print("currentCommand: ");
		Serial.print(currentCommand, 2);
		Serial.print('\t');
		Serial.print("angPosEncRad: ");
		Serial.print(angPosEncRad, 3);
		Serial.print('\t');
		Serial.print("lastAngPosEncRad: ");
		Serial.print(lastAngPosEncRad, 3);
		Serial.print('\t');
		Serial.print("targetAngleRad: ");
		Serial.println(targetAngleRad, 3);
		delayMicroseconds(2000000);
		// increase current command of 0.01A until handle moves
		while (abs(lastAngPosEncRad-angPosEncRad) < 0.003) {
			currentCommand = currentCommand + 0.01;
			#ifdef CURRENT_CONTROL
				analogWrite(sCCP, currentCommand * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
			#endif
			lastAngPosEncRad = angPosEncRad;
			// wait max 50ms so that once the motor spins it does not spin for a too long time and overshoot the next step position (i.e. +5deg => +0.087rad)
			delayMicroseconds(20000);
			// update ADC variables (i.e. angPosEncRad)
			collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
			computeADC();
			averageADCVar();
			Serial.print("angPosEncRad: ");
			Serial.print(angPosEncRad, 3);
			Serial.print('\t');
			Serial.print("currentCommand: ");
			Serial.println(currentCommand, 2);
		}
		// reinitialize currentCommand and stop motor
		currentCommand = 0.0;
		#ifdef CURRENT_CONTROL
				analogWrite(sCCP, currentCommand * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
		// wait 4 second
		delayMicroseconds(2000000);
		// set new target for PID (+5deg => +0.087rad) and use same tuning as before
		targetAngleRad = targetAngleRad + 0.087;
		//myPID->setVariables(&angPosEncRad, &currentCommand, &targetAngleRad);
		myPID->compute();
		Serial.println("print_2:");
		Serial.print("currentCommand: ");
		Serial.print(currentCommand, 2);
		Serial.print('\t');
		Serial.print("angPosEncRad: ");
		Serial.print(angPosEncRad, 3);
		Serial.print('\t');
		Serial.print("lastAngPosEncRad: ");
		Serial.print(lastAngPosEncRad, 3);
		Serial.print('\t');
		Serial.print("targetAngleRad: ");
		Serial.println(targetAngleRad, 3);
		while (abs(myPID->getCurErr()) > 0.01) {
			myPID->compute();
			// run motor in flexion direction
			if (currentCommand < 0) {
				digitalWrite(enCWP, HIGH);
				digitalWrite(enCCWP, LOW);
			}
			// run motor in extension direction
			else if (currentCommand > 0) {
				digitalWrite(enCWP, LOW);
				digitalWrite(enCCWP, HIGH);
			}
			#ifdef CURRENT_CONTROL
				analogWrite(sCCP, abs(currentCommand) * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
			#endif
			// update ADC variables (i.e. angPosEncRad)
			collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
			computeADC();
			averageADCVar();
		}
		lastAngPosEncRad = angPosEncRad;

		// if it moved, actuate motor to next step (+5deg => +0.087rad)
		/*else if (abs(lastAngPosEncRad - angPosEncRad) > 0.003) {
			Serial.println("in large diff cond");
			Serial.print("angle diff: ");
			Serial.print('\t');
			Serial.println(abs(lastAngPosEncRad - angPosEncRad), 5);
			delayMicroseconds(50000);
			Serial.print("angPosEncRad: ");
			Serial.print('\t');
			Serial.println(angPosEncRad, 3);
			analogWrite(sCCP, 0);
			delayMicroseconds(500000);
			Serial.print("angleStep: ");
			Serial.print('\t');
			Serial.println(targetAngleRad, 3);

			while (angPosEncRad < (targetAngleRad + 0.087)) {
				digitalWrite(enCWP, LOW);
				digitalWrite(enCCWP, HIGH);
				#ifdef CURRENT_CONTROL
					analogWrite(sCCP, 0.3 * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
				#endif
				// update ADC variables
				collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
				computeADC();
				averageADCVar();
			}
			analogWrite(sCCP, 0);
			delayMicroseconds(500000);
			targetAngleRad = targetAngleRad + 0.087;
			Serial.println("end");
			Serial.println(angPosEncRad, 3);
			Serial.println(targetAngleRad, 3);
		}*/
	}
	// do not spin motor and initialize current control
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
		currentCommand = 0.0;
	}
}

/*
This function actuates the motor in current control to assess the static friction in the transmission.
Press button PROG1 to start the friction assessment in extension direction.
Press button PROG2 to start the friction assessment in flexion direction.
Once a button (1 or 2) is pressed, the current will gently increase until the handles moves.
Once it has moved, it displays the current and the handle has to be moved 5deg further for the next static friction assessment.
The handle can easily be moved by directly turning the worm with a screwdriver from the front of the eWrist.
*/
void EWrist::assessStaticFrictionManual(ADC *myADC, const int enCWP, const int enCCWP, const int sCCP, const int lCDP, const int lCUP, const int mCP, const int mSP, const int pEP, const int bVP, const int bCP, const int gpio13P, const int gpio16P) {
	// if button PROG1 is activated, increase motor current until handle moves in extension direction
	if (digitalRead(gpio13P) == HIGH && digitalRead(gpio16P) == LOW) {
		// run motor in extension direction
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, HIGH);

		// reinitialize current command and lastAngPosEncRad so that the difference with angPosEncRad is null
		currentCommand = 0.0;
		lastAngPosEncRad = angPosEncRad;

		Serial.println("The static friction assessment in EXTENSION direction will start in 2s");
		delayMicroseconds(2000000);
		// increase current command of 0.01A until handle moves (0.003rad is slighty larger than the sensor noise)
		while (abs(lastAngPosEncRad-angPosEncRad) < 0.003) {
			currentCommand = currentCommand + 0.01;
			#ifdef CURRENT_CONTROL
				analogWrite(sCCP, currentCommand * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
			#endif
			lastAngPosEncRad = angPosEncRad;
			// wait 100ms so that the motor has time to move before next incrementation
			delayMicroseconds(100000);
			// update ADC variables (i.e. angPosEncRad)
			collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
			computeADC();
			averageADCVar();
			Serial.print("angPosEncRad: ");
			Serial.print(angPosEncRad, 3);
			Serial.print('\t');
			Serial.print("lastAngPosEncRad: ");
			Serial.print(lastAngPosEncRad, 3);
			Serial.print('\t');
			Serial.print("currentCommand: ");
			Serial.println(currentCommand, 2);
		}
		// print the current command which moved the handle
		Serial.print("currentCommand: ");
		Serial.println(currentCommand, 2);
		// reinitialize currentCommand and stop motor
		currentCommand = 0.0;
		#ifdef CURRENT_CONTROL
				analogWrite(sCCP, currentCommand * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
		// wait 5 seconds to have time to deactivate the press button
		Serial.println("Deactivate button PROG1 within 5s and move the handle of 5deg in extension for next assessment");
		delayMicroseconds(5000000);
	}

	// if button PROG2 is activated, increase motor current until handle moves in flexion direction
	else if (digitalRead(gpio13P) == LOW && digitalRead(gpio16P) == HIGH) {
		// run motor in flexion direction
		digitalWrite(enCWP, HIGH);
		digitalWrite(enCCWP, LOW);
		
		// reinitialize current command and lastAngPosEncRad so that the difference with angPosEncRad is null
		currentCommand = 0.0;
		lastAngPosEncRad = angPosEncRad;

		Serial.println("The static friction assessment in FLEXION direction will start in 2s");
		delayMicroseconds(2000000);
		// increase current command of 0.01A until handle moves (0.003rad is slighty larger than the sensor noise)
		while (abs(lastAngPosEncRad-angPosEncRad) < 0.003) {
			currentCommand = currentCommand + 0.01;
			#ifdef CURRENT_CONTROL
				analogWrite(sCCP, currentCommand * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
			#endif
			lastAngPosEncRad = angPosEncRad;
			// wait 100ms so that the motor has time to move before next incrementation
			delayMicroseconds(100000);
			// update ADC variables (i.e. angPosEncRad)
			collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
			computeADC();
			averageADCVar();
			Serial.print("angPosEncRad: ");
			Serial.print(angPosEncRad, 3);
			Serial.print('\t');
			Serial.print("lastAngPosEncRad: ");
			Serial.print(lastAngPosEncRad, 3);
			Serial.print('\t');
			Serial.print("currentCommand: ");
			Serial.println(currentCommand, 2);
		}
		// print the current command which moved the handle
		Serial.print("currentCommand: ");
		Serial.println(currentCommand, 2);
		// reinitialize currentCommand and stop motor
		currentCommand = 0.0;
		#ifdef CURRENT_CONTROL
				analogWrite(sCCP, currentCommand * NORM_12_BITS * RECI_ESCON_SET_CUR_AT_3V3);  // set current command in [A], max is 6A
		#endif
		// wait 5 seconds to have time to deactivate the press button
		Serial.println("Deactivate button PROG2 within 5s and move the handle of 5deg in flexion for next assessment");
		delayMicroseconds(5000000);
		
	}
	// do not spin motor and initialize current control
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
		currentCommand = 0.0;
	}
}

/*
This function assess the dynamic friction of the eWrist by spinning it at various angular velocity.
*/
void EWrist::assessDynamicFriction(ADC *myADC, const int enCWP, const int enCCWP, const int sCCP, const int lCDP, const int lCUP, const int mCP, const int mSP, const int pEP, const int bVP, const int bCP, const int gpio13P, const int gpio16P) {
	unsigned long timeUs = 0;		// [us]
	unsigned long initTimeUs = 0;		// [us]
	unsigned int lastTimeUs = 0;	// [us]
	unsigned int deltaTimeUs = 0;	// [us]
	int speedCommand [20] = {0, 2500, 5000, 7500, 10000, 12500, 15000,
							 17500, 20000, 22500, 25000, 27500, 30000,
							 32500, 35000, 37500, 40000, 42500, 45000, 47500}; // [rpm]
	double sumAngVelWristDegSec = 0.0;
	double sumMotCurMeas = 0.0;
	double sumBatCurMeas = 0.0;
	int counter = 0;

	timeUs = micros();
	// start assessment in extension direction when PROG1 button is high and PROG2 button is low
	if (digitalRead(gpio13P) == HIGH && digitalRead(gpio16P) == LOW) {
		Serial.println("Dynamic friction assessment in EXTENSION has started!");
		// run motor in extension direction
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, HIGH);
		// assess the dynamic friction from 0 to 47'500rpm every 2'500rpm
		for (int i=0; i<20; i++) {
			initTimeUs = micros();
			#ifdef SPEED_CONTROL
				analogWrite(sCCP, speedCommand[i] * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
			#endif
			// assess dynamic friction during 10s for each speed command
			while (abs((long)(timeUs-initTimeUs)) < 10000000) {
				timeUs = micros();
				// update ADC variables (i.e. angPosEncRad)
				collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
				computeADC();
				averageADCVar();

				// sum up variables to compute average at the end of the loop
				sumAngVelWristDegSec += convertAngVelMot2Wrist(angVelMotEncRadSec)*180/PI;
				sumMotCurMeas += motCurMeas;
				sumBatCurMeas += batCurMeas;

				counter++;
				lastTimeUs = timeUs;
				deltaTimeUs = (micros() - lastTimeUs);
				// fix the loop timing at 20Hz by delaying it
				if (deltaTimeUs <= 50000) {
					delayMicroseconds(50000 - deltaTimeUs);
				}
			}
			// print average of variables from finished speed command loop
			Serial.print(counter);
			Serial.print("\t");
			Serial.print(speedCommand[i]);
			Serial.print("\t");
			Serial.print(abs(sumAngVelWristDegSec/counter), 2);
			Serial.print("\t");
			Serial.print(abs(sumMotCurMeas/counter), 2);
			Serial.print("\t");
			Serial.println(sumBatCurMeas/counter, 2);
			// initialize sums and counter for next speed command loop
			sumAngVelWristDegSec = 0.0;
			sumMotCurMeas = 0.0;
			sumBatCurMeas = 0.0;
			counter = 0;
		}
	}
	// start assessment in flexion direction when PROG1 button is low and PROG2 button is high
	else if (digitalRead(gpio13P) == LOW && digitalRead(gpio16P) == HIGH) {
		Serial.println("Dynamic friction assessment in FLEXION has started!");
		// run motor in flexion direction
		digitalWrite(enCWP, HIGH);
		digitalWrite(enCCWP, LOW);
		// assess the dynamic friction from 0 to 47'500rpm every 2'500rpm
		for (int i=0; i<20; i++) {
			initTimeUs = micros();
			#ifdef SPEED_CONTROL
				analogWrite(sCCP, speedCommand[i] * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
			#endif
			// assess dynamic friction during 10s for each speed command
			while (abs((long)(timeUs-initTimeUs)) < 10000000) {
				timeUs = micros();
				// update ADC variables (i.e. angPosEncRad)
				collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
				computeADC();
				averageADCVar();

				// sum up variables to compute average at the end of the loop
				sumAngVelWristDegSec += convertAngVelMot2Wrist(angVelMotEncRadSec)*180/PI;
				sumMotCurMeas += motCurMeas;
				sumBatCurMeas += batCurMeas;

				counter++;
				lastTimeUs = timeUs;
				deltaTimeUs = (micros() - lastTimeUs);
				// fix the loop timing at 20Hz by delaying it
				if (deltaTimeUs <= 50000) {
					delayMicroseconds(50000 - deltaTimeUs);
				}
			}
			// print average of variables from finished speed command loop
			Serial.print(counter);
			Serial.print("\t");
			Serial.print(speedCommand[i]);
			Serial.print("\t");
			Serial.print(abs(sumAngVelWristDegSec/counter), 2);
			Serial.print("\t");
			Serial.print(abs(sumMotCurMeas/counter), 2);
			Serial.print("\t");
			Serial.println(sumBatCurMeas/counter, 2);
			// initialize sums and counter for next speed command loop
			sumAngVelWristDegSec = 0.0;
			sumMotCurMeas = 0.0;
			sumBatCurMeas = 0.0;
			counter = 0;
		}
	}
	// do not spin motor
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
	}
}

/*
This function continuously and repetitively actuates the motor in extension and flexion.
*/
bool EWrist::actuateMotorExtFlex(const int enCWP, const int enCCWP, const int sCCP, bool dirFlag) {
	// if angle is in ROM when it starts, run motor in extension direction
	if (angPosEncRad < MAX_ANG_EXT_EF && angPosEncRad > MAX_ANG_FLEX_EF) {
		digitalWrite(enCWP, dirFlag);
		digitalWrite(enCCWP, !dirFlag);
		analogWrite(sCCP, 2000 * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
	}
	// if extension angle limit reached, run motor in flexion direction
	else if (angPosEncRad > MAX_ANG_EXT_EF) {
		dirFlag = true;
		digitalWrite(enCWP, dirFlag);
		digitalWrite(enCCWP, !dirFlag);
		analogWrite(sCCP, 2000 * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
	}
	// if flexion angle limit reached, run motor in extension direction
	else if (angPosEncRad < MAX_ANG_FLEX_EF) {
		dirFlag = false;
		digitalWrite(enCWP, dirFlag);
		digitalWrite(enCCWP, !dirFlag);
		analogWrite(sCCP, 2000 * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
	}
	return dirFlag;
}

/*
This function assess the closed-loop position bandwidth of the eWrist. First the PD controller must be tuned with tunePID() for the desired RoM.
Returns true once all frequencies have been tested.
During assessment, tuning parameters were set at: kp=40'000, ki=0, kd=4'000
ESCON settings:
- speed at 0.0 V = 0.0 rpm
- speed at 3.3 V = 50'000.0 rpm
- current limit = 6.0 A
- acceleration/deceleration = 100'000.0 rpm/s
- offset = 0.0 rpm
- speed at 0.0 V = -50'000.0 rpm
- spedd at 3.3 V = 50'000.0 rpm
- current at 0.0 V = -6.0 A
- current at 3.3 V = 6.0 A
*/
bool EWrist::assessPositionBandwidth(ADC *myADC, PID *myPID, const int enCWP, const int enCCWP, const int sCCP, const int lCDP, const int lCUP, const int mCP, const int mSP, const int pEP, const int bVP, const int bCP, const int gpio13P, const int gpio16P) {
	// start assessment when PROG1 button is high and PROG2 button is low
	if (digitalRead(gpio13P) == HIGH && digitalRead(gpio16P) == LOW) {
		// initialize PD controller
		myPID->setVariables(&angPosEncRad, &speedCommand, &targetAngleRad);	// set variables, i.e. input, output, target
		//myPID->setTunings(27500.0, 0.0, 2750.0);							// set tuning parameters, i.e. kp, ki, kd
		myPID->setTunings(40000.0, 0.0, 4000.0);
		myPID->setLimits(-50000, 50000, -10000, 10000);						// set min and max output limits.
		myPID->setControllerDirection(DIRECT);

		unsigned long timeUs = 0;				// [us]
		unsigned int lastTimeUs = 0;		// [us]
		unsigned int deltaTimeUs = 0;		// [us]
		double timeSec = 0.0;						// [s]
		double initTimeSec = 0.0;				// [s]
		float amplitude = 0.087;				// [rad], default is 5 deg in extension and flexion => 0.087 rad. Tried 40 deg (0.7 rad) but it is meaningless for such an assessment. 
		float frequency [32] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
														1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 
														2.2, 2.4, 2.6, 2.8, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0}; // [Hz]
		float phase = 0.0;							// [rad]
		targetAngleRad = 0.0;						// [rad], straight wrist is 1.34 rad
		float targetAngleRadTemp = 0.0; // [rad]
		// assess the bandwidth from 0.1 to 10 Hz
		for (int i=0; i<32; i++) {
			// get initial time in second
			initTimeSec = ((double)micros() * 0.000001);
			// compute the phase so that the new frequency loop starts with the same targetAngleRad => no abrupt jump!
			phase = asin(targetAngleRadTemp) - (TWO_PI*frequency[i]*initTimeSec);
			// check the time needed at the respective frequency to accomplish 3 cycles (always assign full cycles)
			while (timeSec-initTimeSec < 3/frequency[i]) {
				timeUs = micros();
				timeSec = ((double)timeUs * 0.000001);
				// generate sine wave to be followed which oscillates around 1.34 rad (straight wrist position), or 1.1 rad where friction is lower
				targetAngleRadTemp = sin(TWO_PI*frequency[i]*timeSec + phase);
				targetAngleRad = amplitude * targetAngleRadTemp + 1.1;
				//targetAngleRad = (amplitude * sin(TWO_PI*frequency[i]*timeSec + phase)) + 1.34;  // takes about 50 us to be executed
				
				myPID->compute();
				// run motor in flexion direction
				if (speedCommand < 0) {
					digitalWrite(enCWP, HIGH);
					digitalWrite(enCCWP, LOW);
				}
				// run motor in extension direction
				else if (speedCommand > 0) {
					digitalWrite(enCWP, LOW);
					digitalWrite(enCCWP, HIGH);
				}
				#ifdef SPEED_CONTROL
					analogWrite(sCCP, abs(speedCommand) * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
				#endif
				// update ADC variables (i.e. angPosEncRad)
				collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
				computeADC();
				averageADCVar();
				// send variables for analysis. Multiply by 10'000 to keep accuracy when converting to string since only 2 decimals are kept!
				sendDataOverSerial1kHz(frequency[i], angPosEncRad*10000, targetAngleRad*10000, 0.0, 0.0);

				// DEBUG
				// Serial.print(frequency[i], 2);
				// Serial.print("\t");
				// Serial.print(timeSec, 6);
				// Serial.print("\t");
				// Serial.print(angPosEncRad, 4);
				// Serial.print("\t");
				// Serial.println(speedCommand, 4);
				// Serial.print("\t");
				// Serial.println(targetAngleRad, 4);

				lastTimeUs = timeUs;
				deltaTimeUs = (micros() - lastTimeUs);
				// fix the loop timing at 1kHz by delaying it
				if (deltaTimeUs <= 1000) {
					delayMicroseconds(1000 - deltaTimeUs);
				}
				// check how long does actually take one loop without the delay
				//Serial.println(deltaTimeUs); // => about 485 us with data sending
				// check how long lasts one loop (should be 1000us => 1ms)
				//Serial.println(micros() - timeUs); // => about 1004us
			}
		}
		return true;
	}
	// do not spin motor and initialize speed control
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
		speedCommand = 0.0;
	}
	return false;
}

/*
This function actuates the motor to follow the sEMG data from the CMU.
*/
float EWrist::followCMUEMG(PID *myPID, float *emg, float trgtAngRad, float spdCmd, const int enCWP, const int enCCWP, const int sCCP) {

	float gain = 1.5;
	// transform emg to radian
	trgtAngRad = (gain * emg[0] * PI/180) + 1.34;
	// compute speed command for next loop
	myPID->compute();

	// run motor in flexion direction
	if (spdCmd < 0) {
		digitalWrite(enCWP, HIGH);
		digitalWrite(enCCWP, LOW);
	}
	// run motor in extension direction
	else if (spdCmd > 0) {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, HIGH);
	}
	// do not spin motor and initialize speed control
	else {
		digitalWrite(enCWP, LOW);
		digitalWrite(enCCWP, LOW);
		analogWrite(sCCP, 0);
		spdCmd = 0.0;
	}
	#ifdef SPEED_CONTROL
		analogWrite(sCCP, abs(spdCmd) * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
	#endif

	// DEBUG
	// Serial.print(trgtAngRad, 3);
	// Serial.print("\t");
	// Serial.println(spdCmd, 3);

	return trgtAngRad;
}

/*
This function is used to tune the PID controller in position. Use a battery to allow enough current during high burst otherwise the exo will stop!
By pressing button PROG1 and PROG2, the exo will reach successively the target angles defined below => defined RoM.
Ziegler-Nichols rule to tune a PD: First set Ki and Kd to 0 and gently increase Kp to find Ku (ultimate gain) when system stabily oscillates.
Record the oscillation period Tu in second. Set Kp to 0.8*Ku and Kd to Ku*Tu/10.
Here Ku was evaluated at 200'000 and Tu at 0.96s. Kp = 0.8*200'000 = 160'000 and Kd = 200'000*0.96/10=19'200
For a PD controller: 
- Kp=32'500 and Ki=3'250 works fine without overshoot and reaches precisely the target angle for range 0.84rad to 1.84rad!
- Kp=27'500 and Ki=2'750 works fine for range 0.64rad to 2.04rad!
- Kp=22'500 and Ki=2'250 works fine for range 0.24rad to 2.44rad!
For a PID controller:

ESCON settings:
- speed at 0.0 V = 0.0 rpm
- speed at 3.3 V = 50'000.0 rpm
- current limit = 6.0 A
- acceleration/deceleration = 100'000.0 rpm/s
- offset = 0.0 rpm
- speed at 0.0 V = -50'000.0 rpm
- spedd at 3.3 V = 50'000.0 rpm
- current at 0.0 V = -6.0 A
- current at 3.3 V = 6.0 A
*/
void EWrist::tunePID(ADC *myADC, PID *myPID, const int enCWP, const int enCCWP, const int sCCP, const int lCDP, const int lCUP, const int mCP, const int mSP, const int pEP, const int bVP, const int bCP, const int gpio13P, const int gpio16P) {
	// initialize PD controller
	myPID->setVariables(&angPosEncRad, &speedCommand, &targetAngleRad);	// set variables, i.e. input, output, target
	// set tuning parameters, i.e. kp, ki, kd. => Ku (ultimate gain) is 200'000 => produce stable oscillations at Tu=2*0.48=0.96s!
	//myPID->setTunings(27500.0, 0.0, 2750.0);									// tuning parameters for steady-state error assessment
	myPID->setTunings(40000.0, 0.0, 4000.0);									// tuning parameters used for position bandwidth assessment and applied to steady-state error assessment for comparison
	myPID->setLimits(-50000, 50000, -10000, 10000);						// set min and max output limits. min/max speed is 50000 rpm. And set min/max sum error of integrator
	myPID->setControllerDirection(DIRECT);

	unsigned long timeUs = 0;		// [us]
	unsigned int lastTimeUs = 0;	// [us]
	unsigned int deltaTimeUs = 0;	// [us]
	int sample_cnt = 0;

	while (digitalRead(gpio13P) == HIGH || digitalRead(gpio16P) == HIGH) {
		timeUs = micros();
		// go to position 1 when PROG1 button is high and PROG2 button is low
		if (digitalRead(gpio13P) == HIGH && digitalRead(gpio16P) == LOW) {
			targetAngleRad = 2.04;  // default=1.84 or 2.04 or 2.44
		}
		else {
			targetAngleRad = 0.64;  // default=0.84 or 0.64 or 0.24
		}
		myPID->compute();
		// run motor in flexion direction
		if (speedCommand < 0) {
			digitalWrite(enCWP, HIGH);
			digitalWrite(enCCWP, LOW);
		}
		// run motor in extension direction
		else if (speedCommand > 0) {
			digitalWrite(enCWP, LOW);
			digitalWrite(enCCWP, HIGH);
		}
		#ifdef SPEED_CONTROL
			analogWrite(sCCP, abs(speedCommand) * NORM_12_BITS * RECI_ESCON_SET_SPD_AT_3V3);  // set speed command in [rpm] (@ motor shaft)
		#endif
		// update ADC variables (i.e. angPosEncRad)
		collectADC(myADC, lCDP, lCUP, mCP, mSP, pEP, bVP, bCP);
		computeADC();
		averageADCVar();

		// print only 4000 samples
		if (sample_cnt < 4000) {
			//DEBUG
			//Serial.print(angPosEncRad * 180/PI, 3);
			//Serial.print("\t");
			//Serial.print(targetAngleRad * 180/PI, 3);
			//Serial.print("\t");
			Serial.println((targetAngleRad - angPosEncRad) * 180/PI, 3);
			// Serial.print("\t");
			// Serial.println(speedCommand, 3);
		}

		lastTimeUs = timeUs;
		deltaTimeUs = (micros() - lastTimeUs);
		// fix the loop timing at 1kHz by delaying it
		if (deltaTimeUs <= 1000) {
			delayMicroseconds(1000 - deltaTimeUs);
		}
		// check how long does actually take one loop without the delay
		//Serial.println(deltaTimeUs);
		// check how long lasts one loop (should be 1000us => 1ms)
		//Serial.println(micros() - timeUs); // => about 1004us

		sample_cnt++;
	}
}

void EWrist::averageADCVar() {
	// only average data that are necessary for the controller or safety issues
	angPosEncRad = movingWindowAverage(angPosEncRad, angPosEncRadArray, angPosEncRadArrayTemp, WIN_SIZE_ADC);
	forceMeas = movingWindowAverage(forceMeas, forceMeasArray, forceMeasArrayTemp, WIN_SIZE_ADC);
	angVelMotEncRPM = movingWindowAverage(angVelMotEncRPM, angVelMotEncRPMArray, angVelMotEncRPMArrayTemp, WIN_SIZE_ADC);
	//motCurMeas = movingWindowAverage(motCurMeas, motCurMeasArray, motCurMeasArrayTemp, WIN_SIZE_ADC);
	batVltMeas = movingWindowAverage(batVltMeas, batVltMeasArray, batVltMeasArrayTemp, WIN_SIZE_ADC);
	//batCurMeas = movingWindowAverage(batCurMeas, batCurMeasArray, batCurMeasArrayTemp, WIN_SIZE_ADC);
}

void EWrist::averageDeriv() {
	angVelPosEncRadSec = movingWindowAverage(angVelPosEncRadSec, angVelPosEncRadSecArray, angVelPosEncRadSecArrayTemp, WIN_SIZE_DERIV);
}

float *EWrist::averageIMUVar(float pitchAngle, float rollAngle) {
	IMUVal[0] = movingWindowAverage(pitchAngle, pitchAngleArray, pitchAngleArrayTemp, WIN_SIZE_IMU);
	IMUVal[1] = movingWindowAverage(rollAngle, rollAngleArray, rollAngleArrayTemp, WIN_SIZE_IMU);
	return IMUVal;
}

void EWrist::average(float inputData) {
	float average;
	sumAverage += inputData;
	if (counterAverage >= NUM2AVERAGE)
	{
		average = sumAverage / counterAverage;
		Serial.println(average);
		sumAverage = 0;
		counterAverage = 0;
	}
	counterAverage++;
}

float EWrist::movingWindowAverage(float inputValue, float *inputArray, float *inputArrayTemp, int windowSize) {
	float sum = 0.0;
	float average = 0.0;

	// copy block of memory
	// void * memcpy (void * destination, const void * source, size_t num);
	memcpy(inputArrayTemp, inputArray, windowSize * sizeof(float));
	for (int i=0; i<windowSize; i++)
	{
		sum += inputArrayTemp[i];
		// put a condition to not exceed the array size of inputArray
		if (i<(windowSize-1))
		{
			// shift all values and drop the last value of the array
			inputArray[i+1] = inputArrayTemp[i];
		}
	}
	// put the new force measurement at the first place of the array
	inputArray[0] = inputValue;
	// compute the mean
	average = sum / windowSize;

	return average;
}

// Getters
float EWrist::getAngPosEncRad() {
	return angPosEncRad;
}

float EWrist::getAngPosEncDeg() {
	return angPosEncDeg;
}

float EWrist::getLastAngPosEncRad() {
	return lastAngPosEncRad;
}

float EWrist::getAngVelPosEncRadSec() {
	return angVelPosEncRadSec;
}

float EWrist::getAngVelMotEncRPM() {
	return angVelMotEncRPM;
}

float EWrist::getAngVelMotEncRadSec() {
	return angVelMotEncRadSec;
}

float EWrist::getForceMeas() {
	return forceMeas;
}

float EWrist::getForceOffset() {
	return forceOffset;
}

float EWrist::getMotCurMeas() {
	return motCurMeas;
}

float EWrist::getBatVltMeas() {
	return batVltMeas;
}

float EWrist::getBatCurMeas() {
	return batCurMeas;
}

float EWrist::getSubjectGainGravExt() {
	return mySubject.getGainGravExt();
}

float EWrist::getSubjectGainGravFlex() {
	return mySubject.getGainGravFlex();
}

float EWrist::getSubjectGainSEMGExt() {
	return mySubject.getGainSEMGExt();
}

float EWrist::getSubjectGainSEMGFlex() {
	return mySubject.getGainSEMGFlex();
}

String EWrist::getVmtStatus() {
	return vmtStatus;
}

float EWrist::getVmtInComingString() {
	return vmtInComingString.toFloat();
}

float EWrist::getSubjectHandWeight() {
	return mySubject.getHandWeight();
}

// Setters
void EWrist::setAngPosEncDeg(float aPEDeg) {
	angPosEncDeg = aPEDeg;
}

void EWrist::setLastAngPosEncRad(float lAPERad) {
	lastAngPosEncRad = lAPERad;
}

void EWrist::setAngVelPosEncRadSec(float aVPERadSec) {
	angVelPosEncRadSec = aVPERadSec;
}

void EWrist::setForceOffset(float fOff) {
	forceOffset = fOff;
}

void EWrist::setVmtStatus(String vmtStat) {
	vmtStatus = vmtStat;
}
