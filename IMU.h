/*
IMU.h
Author: Charles Lambelet
Created: October, 2019
*/

/*
REMARKS:
- the MPU6050 chip must be calibrated before use. The script for calibration is /teensy/mpu6050/calibrateMPU6050/calibrateMPU6050.ino.
- do not try to assess the functionality (i.e. delayed response and drift) of the MPU6050 chip with the wrong calibration, i.e. the one of another chip.
=> with the wrong calibration parameters, the reponse of the IMU can show delay and drift even though the chip works perfectly fine!!!
*/

#ifndef IMU_h
#define IMU_h

#include <MPU6050.h>

// IMU MPU6050 in use.
//#define IMU_BRD1_IN_USE									// this board is defect
#define IMU_BRD2_IN_USE									// i.e. on left-handed exo
//#define IMU_BRD3_IN_USE									// i.e. on right-handed exo
//#define IMU_BRD4_IN_USE
//#define IMU_BRD5_IN_USE
//#define IMU_BRD6_IN_USE
//#define IMU_BRD7_IN_USE
//#define IMU_BRD8_IN_USE


//////////////////
// IMU BOARD #1 //
//////////////////
// Calibration parameters for IMU board MPU6050 #1. The fit is a 3rd order curve: y = ax^3 + bx^2 + cx + d
// (see /home/carolek/Polybox/ethz/doctorate/project/python/ewrist/calibration/calib_mpu6050dmp.ipynb)
// However this calibration has been done without knowing that the board (not used anymore) was malfunctioning already from factory settings => default defect!
// Therefore, check that the board indicates adequate angles according to its spatial orientation and responds quickly without overshooting!
#define A_PITCH_IMU_BRD1 0.6471922
#define B_PITCH_IMU_BRD1 -0.0813680
#define C_PITCH_IMU_BRD1 2.1427159
#define D_PITCH_IMU_BRD1 -0.0500168
#define A_ROLL_IMU_BRD1 0.6231320
#define B_ROLL_IMU_BRD1 0.0745627
#define C_ROLL_IMU_BRD1 2.1691233
#define D_ROLL_IMU_BRD1 0.0314724

// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #1
// Sensor readings with offsets:	0	9	16383	0	1	1
// Your offsets:	-1868	5	1667	139	-69	-10
// Sensor readings with offsets:	-5	15	16379	1	1	-1
// Your offsets:	-1865	7	1667	145	-68	-10
// Sensor readings with offsets:	2	5	16398	0	0	-1
// Your offsets:	-1866	4	1668	145	-68	-10

//Data is printed as: acelX acelY acelZ giroX giroY giroZ
//Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD1 -1866
#define ACCELY_OFFSET_IMU_BRD1 5
#define ACCELZ_OFFSET_IMU_BRD1 1667
#define GYROX_OFFSET_IMU_BRD1 143
#define GYROY_OFFSET_IMU_BRD1 -68
#define GYROZ_OFFSET_IMU_BRD1 -10


//////////////////
// IMU BOARD #2 //
//////////////////
// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #2
// Sensor readings with offsets:	-5	-7	16377	-1	2	0
// Your offsets:	-929	913	1445	61	-64	-17
// Sensor readings with offsets:	0	-7	16392	0	0	0
// Your offsets:	-929	919	1446	61	-65	-17
// Sensor readings with offsets:	-3	-1	16394	0	-1	-1
// Your offsets:	-930	921	1447	61	-65	-17

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD2 -929
#define ACCELY_OFFSET_IMU_BRD2 918
#define ACCELZ_OFFSET_IMU_BRD2 1450  // this offset was evaluated empirically to get optimal results
#define GYROX_OFFSET_IMU_BRD2 61
#define GYROY_OFFSET_IMU_BRD2 -65
#define GYROZ_OFFSET_IMU_BRD2 -17


//////////////////
// IMU BOARD #3 //
//////////////////
// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #3
// Sensor readings with offsets:	4	-2	16371	0	-1	0
// Your offsets:	-1409	740	1212	85	59	-25
// Sensor readings with offsets:	-3	4	16385	2	-2	0
// Your offsets:	-1411	730	1206	86	59	-25
// Sensor readings with offsets:	-8	-8	16385	-1	-1	-2
// Your offsets:	-1407	739	1209	85	61	-20

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD3 -1409
#define ACCELY_OFFSET_IMU_BRD3 736
#define ACCELZ_OFFSET_IMU_BRD3 1209
#define GYROX_OFFSET_IMU_BRD3 85
#define GYROY_OFFSET_IMU_BRD3 60
#define GYROZ_OFFSET_IMU_BRD3 23


//////////////////
// IMU BOARD #4 //
//////////////////
// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #4
// Sensor readings with offsets:	-2	0	16377	-1	1	0
// Your offsets:	-762	391	1397	50	-33	4
// Sensor readings with offsets:	-4	-6	16356	-2	0	-2
// Your offsets:	-761	391	1395	49	-33	3
// Sensor readings with offsets:	8	0	16373	1	0	1
// Your offsets:	-764	384	1393	50	-33	2

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD4 -762
#define ACCELY_OFFSET_IMU_BRD4 389
#define ACCELZ_OFFSET_IMU_BRD4 1395
#define GYROX_OFFSET_IMU_BRD4 50
#define GYROY_OFFSET_IMU_BRD4 -33
#define GYROZ_OFFSET_IMU_BRD4 3


//////////////////
// IMU BOARD #5 //
//////////////////
// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #5
// Sensor readings with offsets:	0	10	16379	0	0	0
// Your offsets:	-3561	-974	797	-298	-16	-86
// Sensor readings with offsets:	-9	-2	16355	-1	-1	-2
// Your offsets:	-3557	-969	794	-298	-17	-83
// Sensor readings with offsets:	-8	0	16387	1	-1	1
// Your offsets:	-3556	-966	795	-298	-17	-78

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD5 -3558
#define ACCELY_OFFSET_IMU_BRD5 -970
#define ACCELZ_OFFSET_IMU_BRD5 795
#define GYROX_OFFSET_IMU_BRD5 -298
#define GYROY_OFFSET_IMU_BRD5 -17
#define GYROZ_OFFSET_IMU_BRD5 -82


//////////////////
// IMU BOARD #6 //
//////////////////
// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #6
// Sensor readings with offsets:	0	6	16381	0	1	-1
// Your offsets:	-2850	-501	1963	-226	203	-21
// Sensor readings with offsets:	-5	4	16386	2	-1	-1
// Your offsets:	-2849	-504	1963	-226	201	-19
// Sensor readings with offsets:	3	-2	16371	0	0	-2
// Your offsets:	-2854	-513	1959	-225	203	-21

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD6 -2851
#define ACCELY_OFFSET_IMU_BRD6 -506
#define ACCELZ_OFFSET_IMU_BRD6 1962
#define GYROX_OFFSET_IMU_BRD6 -226
#define GYROY_OFFSET_IMU_BRD6 202
#define GYROZ_OFFSET_IMU_BRD6 -20


//////////////////
// IMU BOARD #7 //
//////////////////
// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #7
// Sensor readings with offsets:	-4	-6	16392	1	-1	-1
// Your offsets:	-399	-196	1563	71	-52	-8
// Sensor readings with offsets:	-6	-2	16392	0	-1	1
// Your offsets:	-396	-200	1562	71	-52	-10
// Sensor readings with offsets:	1	-3	16389	1	-1	-3
// Your offsets:	-399	-204	1562	71	-52	-9

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD7 -398
#define ACCELY_OFFSET_IMU_BRD7 -200
#define ACCELZ_OFFSET_IMU_BRD7 1562
#define GYROX_OFFSET_IMU_BRD7 71
#define GYROY_OFFSET_IMU_BRD7 -52
#define GYROZ_OFFSET_IMU_BRD7 -9


//////////////////
// IMU BOARD #8 //
//////////////////
// 3 readouts from arduino sketch calibrateMPU6050.ino for IMU board MPU6050 #8
// Sensor readings with offsets:	-1	-9	16383	0	0	-1
// Your offsets:	642	-500	1031	-282	-30	-63
// Sensor readings with offsets:	6	0	16359	0	0	0
// Your offsets:	644	-496	1029	-282	-29	-63
// Sensor readings with offsets:	8	10	16374	0	-1	-1
// Your offsets:	647	-493	1029	-282	-29	-63

// Data is printed as: acelX acelY acelZ giroX giroY giroZ
// Check that your sensor readings are close to 0 0 16384 0 0 0

#define ACCELX_OFFSET_IMU_BRD8 644
#define ACCELY_OFFSET_IMU_BRD8 -496
#define ACCELZ_OFFSET_IMU_BRD8 1030
#define GYROX_OFFSET_IMU_BRD8 -282
#define GYROY_OFFSET_IMU_BRD8 -29
#define GYROZ_OFFSET_IMU_BRD8 -63


class IMU {
public:

	IMU();						// constructor
	~IMU();						// destructor

	void static setMpuIntStatus(uint8_t status);

	void initialize();			// initialize IMU.
	void compute();				// perform IMU calculations.
	void initMPU6050DMP();
	void computeMPU6050DMP();
	void resetFIFO();
	void calibrateMPU6050DMP(float, float, float, float, float, float, float, float);

	float getYaw();
	float getPitch();
	float getRoll();
	float getGravityZ();

	volatile static bool mpuInterrupt;     // indicates whether MPU interrupt pin has gone high

private:

	const int mpuIntPin = 12;

	MPU6050 mpu;
	//MPU6050 mpu(0x69); // <-- use for AD0 high

	// MPU control/status variables
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// orientation/motion variables
	Quaternion q;           // [w, x, y, z]         	quaternion container
	VectorInt16 aa;         // [x, y, z]            	accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            	gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            	world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            	gravity vector
	float euler[3];         // [psi, theta, phi]    	Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   	yaw/pitch/roll container in rad
	float ypr_calib[3];     // [yaw, pitch, roll]	  	yaw/pitch/roll container in rad
	float yaw_init;
	int counter_init;		// used to initialize the axis after 1200 readings (cycles)
};
#endif
