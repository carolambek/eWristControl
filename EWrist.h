/*
EWrist.cpp
Author: Charles Lambelet
Created: August 17, 2017
*/

#ifndef EWRIST_h
#define EWRIST_h

#include <WProgram.h>
#include <ADC.h>
#include "PID.h"
#include "Subject.h"

// Load cells characteristics (when electronics has warmed up):
// ADC to Newton conversion parameters for load cell #5 with upperarm board #2
#define A_DOWN_LC5_BRD2 140.1959
#define RECI_A_DOWN_LC5_BRD2 0.0071329
#define B_DOWN_LC5_BRD2 -216.9172
#define A_UP_LC5_BRD2 115.5917
#define RECI_A_UP_LC5_BRD2 0.0086511
#define B_UP_LC5_BRD2 549.5951
#define A_ALPHA_LC5_BRD2 0.0022							// = 1/463
#define B_ALPHA_LC5_BRD2 -0.1879						// = -87/463
#define A_BETA_LC5_BRD2 -0.0022							// = -1/463
#define B_BETA_LC5_BRD2 1.1879							// = 550/463
#define ADC_LIM_ONLY_DOWN_LC5_BRD2 550					// limit above which only DOWN direction ADC is used to compute the force.
#define ADC_LIM_ONLY_UP_LC5_BRD2 150					// limit below which only UP direction ADC is used to compute the force.

// ADC to Newton conversion parameters for load cell #13 with upperarm board #1
#define A_DOWN_HEAVY_LC13_BRD1 119.2720
#define RECI_A_DOWN_HEAVY_LC13_BRD1 0.0083842
#define B_DOWN_HEAVY_LC13_BRD1 464.6556
#define A_UP_HEAVY_LC13_BRD1 138.1119
#define RECI_A_UP_HEAVY_LC13_BRD1 0.0072405
#define B_UP_HEAVY_LC13_BRD1 -104.1995
#define A_DOWN_LIGHT_LC13_BRD1 116.6487
#define RECI_A_DOWN_LIGHT_LC13_BRD1 0.0085727
#define B_DOWN_LIGHT_LC13_BRD1 460.1572
#define A_UP_LIGHT_LC13_BRD1 126.6970
#define RECI_A_UP_LIGHT_LC13_BRD1 0.0078928
#define B_UP_LIGHT_LC13_BRD1 -40.9598
#define ADC_LIM_ONLY_DOWN_HEAVY_LC13_BRD1 288			// limit above which only DOWN direction ADC is used to compute the force.
#define ADC_LIM_ONLY_UP_HEAVY_LC13_BRD1 652				// limit below which only UP direction ADC is used to compute the force.
#define ADC_LIM_ONLY_UP_LIGHT_LC13_BRD1 160				// limit below which only UP direction ADC is used to compute the force.

// Position encoder characteristics:
// For encoder R1 (right, #1) 
#define ADC_POS_ENC_R1_NOM 2365							// = hand aligned with forearm (0° from horizontal)
#define ADC_POS_ENC_R1_MAX 3397							// = wrist full extension (77° from horizontal)
#define ADC_POS_ENC_R1_MIN 1158							// = wrist full flexion (77° from horizontal)
// For encoder L1 (left, #1)
#define ADC_POS_ENC_L1_NOM 2789							// = hand aligned with forearm (0° from horizontal)
#define ADC_POS_ENC_L1_MIN 1675							// = wrist full extension (77° from horizontal)
#define ADC_POS_ENC_L1_MAX 3804							// = wrist full flexion (77° from horizontal)

// ROM of eWrist:
#define MAX_ANG_EXT 2.8								// [rad], maximum extension angle, default=2.32rad=132.93deg, 2.74rad=157deg
#define MAX_ANG_FLEX -0.1								// [rad], maximum flexion angle, default=0.28rad=16.04deg, -0.05rad=-3deg
// ROM for actuate motor to position
#define MAX_ANG_EXT_A2P 114								// [deg], default=114deg for step response (to leave margin for the handle to stop)
#define MAX_ANG_FLEX_A2P 40								// [deg], default=40deg for step response (to leave margin for the handle to stop)
// ROM for indefinite extensions and flexions
#define MAX_ANG_EXT_EF 1.94								// [rad]
#define MAX_ANG_FLEX_EF 0.64							// [rad]

// Motor, gears and ESCON module characteristics:
#define MOTOR_GEARHEAD_RATIO 19
#define WORMGEAR_RATIO 25
#define GEARDRIVE_RATIO 475								// = 19*25
#define RECI_GEARDRIVE_RATIO 0.0021053					// = 1/475 = 1/(19*25)
#define MAX_SPD_CMD 10000								// [rpm], default=7000, also set in ESCON controller at 70000 rpm
#define MIN_SPD_CMD -10000								// [rpm], default=-7000, also set in ESCON controller at -70000 rpm
#define DEAD_ZONE_CMD 100								// default=0.01
#define DEAD_ZONE_FORCE_MEAS 0.1						// [N]
#define ESCON_MAX_PERM_SPD 70000						// [rpm]
#define ESCON_SET_SPD_AT_3V3 50000						// [rpm], default=15000, physical_max=50000, permissible_max=70000
#define RECI_ESCON_SET_SPD_AT_3V3 0.00002				// [1/rpm]
#define ESCON_GET_SPD_AT_0V -ESCON_SET_SPD_AT_3V3
#define ESCON_GET_SPD_AT_3V3 ESCON_SET_SPD_AT_3V3
#define ESCON_SET_CUR_AT_3V3 6							// [A], default=6, max=6
#define RECI_ESCON_SET_CUR_AT_3V3 0.1666666				// [1/A]
#define ESCON_GET_CUR_AT_0V -ESCON_SET_CUR_AT_3V3				
#define ESCON_GET_CUR_AT_3V3 ESCON_SET_CUR_AT_3V3
#define ESCON_ACC_RAMP 100000							// [rpm/s], default=30000, max=1000000
#define ESCON_DEC_RAMP 100000							// [rpm/s], default=30000, max=1000000
#define ESCON_OFFSET 0									// [rpm]
#define NOISE_OFFSET 0.0114352							// averaged noise offset. When 1.65V corresponds to 0rpm, 1.65+0.0114352 are read because of noise

// The maximal physical angular velocity reachable by the current configuration is about 49'000rpm (@ motor shaft).
// This angular velocity was reached by a step response in current control were 6.0[A] (max) were given as input command.
// !!!IMPORTANT!!! A battery should be used to release large current in short time. Or a power supply with max current!
// When ESCON_ACC_RAMP is set too high (i.e. 500000 - 1000000 [rpm/s]) the motor might vibrate and not start spinning!

// Teensy ADC conversion characteristics:
#define NORM_12_BITS 4095								// 12 bits ADC conversion on Teensy 3.1/3.2
#define RECI_12_BITS 0.0002442							// reciproc(4095) = 1/4095 => 12 bits ADC conversion
#define LOGIC_3V3_LEV 3.3								// logic voltage level is 3.3V (for ESCON module as well)
#define RECI_LOGIC_3V3_LEV 0.3030303					// reciproc of logic voltage level => used to have multiplications instead of divisions

// Upperarm board voltage and current characteristics:
// For board #1
#define A_VLT_BRD1 224.81
#define RECI_A_VLT_BRD1 0.0044482
#define B_VLT_BRD1 8.83
#define A_CUR_BRD1 125.8
#define RECI_A_CUR_BRD1 0.0079491
#define B_CUR_BRD1 2052.6
// For board #2
#define A_VLT_BRD2 223.64
#define RECI_A_VLT_BRD2 0.0044715
#define B_VLT_BRD2 18.97
// #define A_CUR_BRD2 118.1
// #define RECI_A_CUR_BRD2 0.0084674
// #define B_CUR_BRD2 2057.6
#define A_CUR_BRD2 116.81
#define RECI_A_CUR_BRD2 0.0085609
#define B_CUR_BRD2 2063.90

// Battery characteristics:
#define LOW_VOLTAGE_LIPO_3S 10							// [V] minimum voltage per cell should be 3.3V => 9.9V for 3 cells

// Averaging parameters:
#define NUM2AVERAGE 500
#define WIN_SIZE_ADC 20									// default=20, large window size will slow down the real-time process!
#define WIN_SIZE_DERIV 200								// default=200
#define WIN_SIZE_CTR_CMD 40								// default=40
#define WIN_SIZE_EMG 100								// default=100
#define WIN_SIZE_IMU 20									// default=20

// Admittance controller fixed virtual parameters:						
#define FIXED_ADM_PARAM									// use fixed parameters (virtual mass and damping) for the admittance controller.
#define VARIABLE_ADM_PARAM								// use variable parameters (virtual mass and damping) for the admittance controller.
#define MIN_VIRT_MASS 0.004								// [Nm/rad/s^2] minimal virtual mass or inertia. Default=0.01. Transparency plane testing values: 0.005, 0.0025, 0.01, 0.05, 0.1, 0.5 
#define MIN_VIRT_DAMPING 0.04							// [Nm/rad/s] minimal virtual damping. Default=0.1. Should not be lower than 0.01 or become instable. Transparency plane testing values: 0.05, 0.1, 0.2, 0.5, 1
#define LENGTH 0.08										// [m] length from rotation axis to average center of force application on handle. Default=0.08 m or 0.07 m for stiffness function assessment
#define WIN_SIZE_FFM 200								// window size for the analysis of the Force Frequency and Magnitude (FFM). default=250, large window size will slow down the real-time process!
#define LOOP2SKIP 8										// number of main loop to skip to perform the force frequency and magnitude analysis.
#define MAX_FORCE_FREQ (WIN_SIZE_FFM/LOOP2SKIP)			// maximal number of force sign changes during unstable oscillations.
#define MAX_FORCE_MAG 8									// [N] maximal force applicable on the handle => used for normalization. Default=5. The higher the less the magnitude is taken into account
#define LAMBDA 0.7										// adjust influence of past index of instability.
#define MASS_INCREASE 0									// only virtual mass increases during instability and virtual damping remains constant.
#define DAMPING_INCREASE 1								// only virtual damping increases during instability and virtual mass remains constant.
#define MASS_DAMPING_INCREASE 0							// both virtual mass and damping increase during instability with a constant ratio.

// Math constants
//#define PI 3.1415926									// already defined in wiring.h
//#define TWO_PI 6.2831853								// already defined in wiring.h
#define RECI_PI 0.3183099
#define RECI_TWO_PI 0.1591549


// Exo in use (right-hand or left-hand)
#define EXO_IN_USE 1									// 0 = right-hand, 1 = left-hand

// Loadcell and board in use
//#define LC5B2_IN_USE									// i.e. right-hand exo
#define LC13B1_IN_USE									// i.e. left-hand exo

// Control in use
#define SPEED_CONTROL
//#define CURRENT_CONTROL

// if IMU is in use
#define IMU_ENABLED

// if PID is in use
//#define PID_ENABLED

// if exo is to be used with visuomotor task
#define VMT_ENABLED

class EWrist {
public:

	EWrist();      // Constructor
	~EWrist();     // Destructor

	void initADC(ADC*);
	void initEMG();
	void initPID(PID*);
	void collectADC(ADC*, const int, const int, const int, const int, const int, const int, const int);
	void computeADC();
	void collectDataVMT(String);
	void averageADCVar();
	void averageDeriv();
	void sendDataOverSerial50Hz(float, float, float, float, float, float);
	void sendDataOverSerial60Hz(unsigned int, float, float, float, float, float, float);
	void sendDataOverSerial1kHz(float, float, float, float, float);
	void checkBatteryVoltage(float, const int, const int, const int, const int);
	void stopMotor(const int, const int, const int);
	void actuateMotorInROM(ADC*, float, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int);
	void actuateMotor2Pos(float, float, const int, const int, const int);
	void actuateMotorWithButton(const int, const int, const int, const int, const int);
	void actuateMotor2PosWithButton(float, float, float, float, const int, const int, const int, const int, const int);
	void assessStaticFriction(ADC*, PID*, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int);
	void assessStaticFrictionManual(ADC*, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int);
	void assessDynamicFriction(ADC*, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int);
	bool actuateMotorExtFlex(const int, const int, const int, bool);
	bool assessPositionBandwidth(ADC*, PID*, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int);
	void tunePID(ADC*, PID*, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int, const int);
	void average(float);

	float *collectEMGAdptCtrl(String);
	float *collectEMGCMU(String);
	float *averageIMUVar(float, float);
	float followCMUEMG(PID*, float*, float, float, const int, const int, const int);
	float convertADC2NewtonLC5B2(int, int, float, float, float, float, float, float, float, float, int, int);
	float convertADC2NewtonLC13B1(int, int, float, float, float, float, float, float, int, int, int);
	float convertADC2Degree(int, int, int);
	float convertADC2RPMMot(int);
	float convertADC2AmpereMot(int);
	float convertADC2VoltBat(int, float, float);
	float convertADC2AmpereBat(int, float, float);
	float convertRPM2RadSec(float);
	float convertRadSec2RPM(float);
	float convertAngVelMot2Wrist(float);
	float convertAngVelWrist2Mot(float);
	float convertForce2Torque(float);
	float deg2Rad(float);
	float rad2Deg(float);
	float fixForceAroundZero(float);
	float *analyzeForceFreqMag(float);
	float computeIndexInstab(float*, float);
	float computeStiffFct(float);
	void adjustAdmCtrlParameters(float, float);
	float applyTustinTransform(float, float, float, double);
	float computeGravCompForce(float, float, float, float);
	float checkCtrlCmdLimits(float);
	float getAngPosEncRad();
	float getAngPosEncDeg();
	float getLastAngPosEncRad();
	float getAngVelPosEncRadSec();
	float getAngVelMotEncRPM();
	float getAngVelMotEncRadSec();
	float getForceMeas();
	float getForceOffset();
	float getMotCurMeas();
	float getBatVltMeas();
	float getBatCurMeas();
	String getVmtStatus();
	float getVmtInComingString();
	float getSubjectHandWeight();
	float getSubjectGainGravExt();
	float getSubjectGainGravFlex();
	float getSubjectGainSEMGExt();
	float getSubjectGainSEMGFlex();
	void setAngPosEncDeg(float);
	void setLastAngPosEncRad(float);
	void setAngVelPosEncRadSec(float);
	void setForceOffset(float);
	void setVmtStatus(String);

	float averageVar_AngVelVirt(float);
	float averageVar_AngAccMot(float);
	float averageVar_AngVelMeas(float);
	float averageVar_controlCommand_AngVel(float);
	float averageVar_EMGFlex(float);
	float averageVar_EMGExt(float);
	float movingWindowAverage(float, float*, float*, int);

	float mpuAccX;
	float mpuAccY;
	float mpuAccZ;

	String incomingString;

private:

	Subject mySubject;

	// ewrist variables
	int loadCellDownADC, loadCellUpADC;
	int motCurADC, motSpdADC, posEncADC, batVltADC, batCurADC;
	unsigned int timerBlinkVoltage;
	unsigned long recordCounter;
	float angVelMeas, angAccMeas;
	float angVelVirt, angAccVirt;
	float angPosEncDeg, angPosEncRad;
	float lastAngPosEncRad;
	float angVelPosEncRadSec;
	float angVelMotEncRadSec, angVelMotEncRPM;
	float forceMeas, forceOffset;
	float motCurMeas, batVltMeas, batCurMeas;
	float averageAngPosEncRad, averageForceMeas, averageAngAccVirt, averageAngVelVirt;
	bool forceMeasSign, lastForceMeasSign;
	float idxForceFreqMag[2];
	int forceSignChangeArray[WIN_SIZE_FFM];
	int forceSignChangeArrayTemp[WIN_SIZE_FFM];
	float forceMagSquaredArray[WIN_SIZE_FFM];
	float forceMagSquaredArrayTemp[WIN_SIZE_FFM];
	float virtualMass, virtualDamping;
	bool fix_var_adm_param = true;

	// EMG variables
	String subStrEMG3, subStrEMG4, subStrEMG7, subStrEMG8;
	int stringLen;
	int commaIndex1, commaIndex2, commaIndex3;
	float EMG3, EMG4, EMG7, EMG8;
	float EMGVal[2];
	float EMGExtVal, EMGFlexVal;

	// IMU variables
	float IMUVal[2];

	// PID variables
	float currentCommand;
	float speedCommand;
	float targetAngleRad;

	// VMT variables
	String vmtStatus;
	String vmtInComingString;

	// simple averaging variables
	unsigned int counterAverage = 0;
	float sumAverage = 0;

	// moving window averaging variables
	float forceMeasArray[WIN_SIZE_ADC];
	float forceMeasArrayTemp[WIN_SIZE_ADC];
	float angPosEncRadArray[WIN_SIZE_ADC];
	float angPosEncRadArrayTemp[WIN_SIZE_ADC];
	float angVelPosEncRadSecArray[WIN_SIZE_DERIV];
	float angVelPosEncRadSecArrayTemp[WIN_SIZE_DERIV];
	float angVelMotEncRPMArray[WIN_SIZE_DERIV];
	float angVelMotEncRPMArrayTemp[WIN_SIZE_DERIV];
	//float motCurMeasArray[WIN_SIZE_ADC];
	//float motCurMeasArrayTemp[WIN_SIZE_ADC];
	float batVltMeasArray[WIN_SIZE_ADC];
	float batVltMeasArrayTemp[WIN_SIZE_ADC];
	//float batCurMeasArray[WIN_SIZE_ADC];
	//float batCurMeasArrayTemp[WIN_SIZE_ADC];
	//float angVelVirtArray[WIN_SIZE_ADC];
	//float angVelVirtArrayTemp[WIN_SIZE_ADC];
	//float angAccArray[WIN_SIZE_ADC];
	//float angAccArrayTemp[WIN_SIZE_ADC];
	//float angVelMeasArray[WIN_SIZE_ADC];
	//float angVelMeasArrayTemp[WIN_SIZE_ADC];
	//float angPosEncDegArray[WIN_SIZE_ADC];
	//float angPosEncDegArrayTemp[WIN_SIZE_ADC];
	//float EMGFlexArray[WIN_SIZE_EMG];
	//float EMGFlexArrayTemp[WIN_SIZE_EMG];
	//float EMGExtArray[WIN_SIZE_EMG];
	//float EMGExtArrayTemp[WIN_SIZE_EMG];
	float pitchAngleArray[WIN_SIZE_IMU];
	float pitchAngleArrayTemp[WIN_SIZE_IMU];
	float rollAngleArray[WIN_SIZE_IMU];
	float rollAngleArrayTemp[WIN_SIZE_IMU];
};
#endif
