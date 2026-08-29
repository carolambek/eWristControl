/*
Subject.h
Author: Melvin Mathis
Created: April, 2018
Modified: Charles Lambelet - March 2019
*/

#ifndef SUBJECT_h
#define SUBJECT_h

#include <WProgram.h>

class Subject {
public:

	Subject();      // Constructor
	~Subject();     // Destructor

	float getHandWeight();
	float getMVCExtensor();
	float getMVCFlexor();
	float getMaxActFlexAngle();
	float getMaxActExtAngle();
	float getMaxPasFlexAngle();
	float getMaxPasExtAngle();
	float getPMVCDiff();
	float getGainGravExt();
	float getGainGravFlex();
	float getGainSEMGExt();
	float getGainSEMGFlex();
	float getGainStiffFct();
	float *getStiffFctPara();
	void setHandWeight(float);
	void setMVCExtensor(float);
	void setMVCFlexor(float);
	void setMaxActFlexAngle(float);
	void setMaxActExtAngle(float);
	void setMaxPasFlexAngle(float);
	void setMaxPasExtAngle(float);
	void setPMVCDiff(float);
	void setGainGravExt(float);
	void setGainGravFlex(float);
	void setGainSEMGExt(float);
	void setGainSEMGFlex(float);
	void setGainStiffFct(float);
	void setStiffFctPara(String, String);

private:

	float handWeight;
	float MVCExtensor;
	float MVCFlexor;
	float maxActFlexAngle;
	float maxActExtAngle;
	float maxPasFlexAngle;
	float maxPasExtAngle;
	float pMVCDiff;
	float gainGravExt;
	float gainGravFlex;
	float gainSEMGExt;
	float gainSEMGFlex;

	// stiffness function parameters
	float x1, a1, b1, c1, x2, a2, b2, x3, a3, b3, c3, x4;
	float stiffFctPara[10];  // without x1 and x4
	float gainStiffFct;
};
#endif
