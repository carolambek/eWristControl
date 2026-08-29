/*
Subject.cpp
Author: Melvin Mathis
Created: April, 2018
Modified: Charles Lambelet - March 2019
*/

#include "Subject.h"

// Constructor. Initialize different variables
Subject::Subject() {
	handWeight = 0.0;
	maxActFlexAngle = 0.0;
	maxActExtAngle = 0.0;
	maxPasFlexAngle = 0.0;
	maxPasExtAngle = 0.0;
	gainGravExt = 1.0;			// default: 1.0
	gainGravFlex = 1.0;			// default: 1.0
	gainSEMGExt = 1.0;			// default: 1.0
	gainSEMGFlex = 1.0;			// default: 1.0
	x1 = a1 = b1 = c1 = x2 = a2 = b2 = x3 = a3 = b3 = c3 = x4 = 0;
	gainStiffFct = 0.5;
}

// Destructor.
Subject::~Subject() {

}

// Getters
float Subject::getHandWeight() {
	return handWeight;
}

float Subject::getMVCExtensor() {
	return MVCExtensor;
}

float Subject::getMVCFlexor() {
	return MVCFlexor;
}

float Subject::getMaxActFlexAngle() {
	return maxActFlexAngle;
}

float Subject::getMaxActExtAngle() {
	return maxActExtAngle;
}

float Subject::getMaxPasFlexAngle() {
	return maxPasFlexAngle;
}

float Subject::getMaxPasExtAngle() {
	return maxPasExtAngle;
}

float Subject::getPMVCDiff() {
	return pMVCDiff;
}

float Subject::getGainGravExt() {
	return gainGravExt;
}

float Subject::getGainGravFlex() {
	return gainGravFlex;
}

float Subject::getGainSEMGExt() {
	return gainSEMGExt;
}

float Subject::getGainSEMGFlex() {
	return gainSEMGFlex;
}

float Subject::getGainStiffFct() {
	return gainStiffFct;
}

float *Subject::getStiffFctPara(){
	stiffFctPara[0] = a1;
	stiffFctPara[1] = b1;
	stiffFctPara[2] = c1;
	stiffFctPara[3] = x2;
	stiffFctPara[4] = a2;
	stiffFctPara[5] = b2;
	stiffFctPara[6] = x3;
	stiffFctPara[7] = a3;
	stiffFctPara[8] = b3;
	stiffFctPara[9] = c3;

	return stiffFctPara;
}

// Setters
void Subject::setHandWeight(float hWht) {
	handWeight = hWht;
}

void Subject::setMVCExtensor(float MVCExt) {
	MVCExtensor = MVCExt;
}

void Subject::setMVCFlexor(float MVCFlex) {
	MVCFlexor = MVCFlex;
}

void Subject::setMaxActFlexAngle(float mActFlexAng) {
	maxActFlexAngle = mActFlexAng;
}

void Subject::setMaxActExtAngle(float mActExtAng) {
	maxActExtAngle = mActExtAng;
}

void Subject::setMaxPasFlexAngle(float mPasFlexAng) {
	maxPasFlexAngle = mPasFlexAng;
}

void Subject::setMaxPasExtAngle(float mPasExtAng) {
	maxPasExtAngle = mPasExtAng;
}

void Subject::setPMVCDiff(float pMVCD) {
	pMVCDiff = pMVCD;
}

void Subject::setGainGravExt(float gGravExt) {
	gainGravExt = gGravExt;
}

void Subject::setGainGravFlex(float gGravFlex) {
	gainGravFlex = gGravFlex;
}

void Subject::setGainSEMGExt(float gSEMGExt) {
	gainSEMGExt = gSEMGExt;
}

void Subject::setGainSEMGFlex(float gSEMGFlex) {
	gainSEMGFlex = gSEMGFlex;
}

void Subject::setGainStiffFct(float gStiffFct) {
	gainStiffFct = gStiffFct;
}

void Subject::setStiffFctPara(String status, String vmtInComingString) {
	// set x1 (1st point, i.e. maxFlexAngle)
	if (status == "sfx1") {
		maxPasFlexAngle = x1 = vmtInComingString.toFloat();
	}
	// set a1 (1st parabola)
	else if (status == "sfa1") {
		a1 = vmtInComingString.toFloat();
	}
	// set b1 (1st parabola)
	else if (status == "sfb1") {
		b1 = vmtInComingString.toFloat();
	}
	// set c1 (1st parabola)
	else if (status == "sfc1") {
		c1 = vmtInComingString.toFloat();
	}
	// set x2 (2nd point)
	else if (status == "sfx2") {
		x2 = vmtInComingString.toFloat();
	}
	// set a2 (line)
	else if (status == "sfa2") {
		a2 = vmtInComingString.toFloat();
	}
	// set b2 (line)
	else if (status == "sfb2") {
		b2 = vmtInComingString.toFloat();
	}
	// set x3 (3rd point)
	else if (status == "sfx3") {
		x3 = vmtInComingString.toFloat();
	}
	// set a3 (2nd parabola)
	else if (status == "sfa3") {
		a3 = vmtInComingString.toFloat();
	}
	// set b3 (2nd parabola)
	else if (status == "sfb3") {
		b3 = vmtInComingString.toFloat();
	}
	// set c3 (2nd parabola)
	else if (status == "sfc3") {
		c3 = vmtInComingString.toFloat();
	}
	// set x4 (last point, i.e. maxExtAngle)
	else if (status == "sfx4") {
		maxPasExtAngle = x4 = vmtInComingString.toFloat();
	}
/*
	Serial.print(x1, 3);
	Serial.print('\t');
	Serial.print(a1, 3);
	Serial.print('\t');
	Serial.print(b1, 3);
	Serial.print('\t');
	Serial.print(c1, 3);
	Serial.print('\t');
	Serial.print(x2, 3);
	Serial.print('\t');
	Serial.print(a2, 3);
	Serial.print('\t');
	Serial.print(b2, 3);
	Serial.print('\t');
	Serial.print(x3, 3);
	Serial.print('\t');
	Serial.print(a3, 3);
	Serial.print('\t');
	Serial.print(b3, 3);
	Serial.print('\t');
	Serial.print(c3, 3);
	Serial.print('\t');
	Serial.println(x4, 3);
*/
}