#include "your_code.h"
#include "debug.h"
/***
 *
 * This file is where you should add you tasks. You already know the structure
 * Required to do so from the work with the simulator.
 *
 * The function yourCodeInit() is set to automatically execute when the
 * quadrotor is started. This is where you need to create your tasks. The
 * scheduler that runs the tasks is already up and running so you should
 * NOT make a call to vTaskStartScheduler();.
 *
 * Below that you can find a few examples of useful function calls and code snippets.
 *
 * For further reference on how this is done. Look into the file stabilizer.c
 * which is usually handles the control of the crazyflie.
 *
 ***/
#define DEG2RAD 0.0175f  // TODO: CHECK ALL UNITS DEG/RAD and SIGNS (axes and K)
 // Declare functions
static void compFilt(void* pvParameters);
static void getRef(void* pvParameters);
static void cant(void* pvParameters);
static void testcase(void* pvParameters);

// Declare global variables
float roll, pitch, droll, dpitch, dyaw;
xTaskHandle compFiltHandle, getRefHandle, cantHandle, testcaseHandle;
SemaphoreHandle_t writingAngle, writingRef, sensorsCalibrated;
float Ts = 0.01f;  // sampling time
static setpoint_t setpoint;
static state_t state;  // check this, update in cant?

float ctrl1 = 0.0;
float ctrl2 = 0.0;
float ctrl3 = 0.0;
float ctrl4 = 0.0;

float accumulated_gyro_roll = 0.0;
float accumulated_pitch_roll = 0.0;

sensorData_t sensorData;

void yourCodeInit(void)
{
	sensorsCalibrated = xSemaphoreCreateBinary();
	writingAngle = xSemaphoreCreateBinary();
	xSemaphoreGive(writingAngle);
	writingRef = xSemaphoreCreateBinary();
	xSemaphoreGive(writingRef);

    xTaskCreate(compFilt, "Complementary filter task", SENSORS_TASK_STACKSIZE, NULL, 3, &compFiltHandle);
    xTaskCreate(getRef, "Reference setting task", configMINIMAL_STACK_SIZE, NULL, 1, &getRefHandle);
    xTaskCreate(cant, "Controller task", STABILIZER_TASK_STACKSIZE, NULL, 2, &cantHandle);
	//xTaskCreate(testcase, "testcase task", STABILIZER_TASK_STACKSIZE, NULL, 4, &testcaseHandle);
}


void testcase(void *pvParameters){
	uint32_t lastWakeTime;
	xSemaphoreTake(sensorsCalibrated, portMAX_DELAY);
	DEBUG_PRINT("\nTestcase entered\n");
	while(1){
		lastWakeTime = xTaskGetTickCount();
		if (roll > 1.0f){
			motorsSetRatio(MOTOR_M4, 0x0FFF);
			motorsSetRatio(MOTOR_M1, 0x0FFF);
		} else{
			motorsSetRatio(MOTOR_M4, 0x0000);
			motorsSetRatio(MOTOR_M1, 0x0000);
		}
		vTaskDelayUntil(&lastWakeTime, M2T(10));
	}
}


// complementary filter
void compFilt(void *pvParameters){

	// calibrate the sensors
	DEBUG_PRINT("\nCalibrating sensors...\n");
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(!sensorsAreCalibrated()){
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
	}
	xSemaphoreGive(sensorsCalibrated);
	DEBUG_PRINT("\nSensors cablibrated!\n");
	int sensorsInit = 1;


	float w;  // acc vector norm
	float roll_acc, pitch_acc;
	float roll_gyro = 0.0f, pitch_gyro = 0.0f;  // gyro estimates
	uint32_t tick = 1;

    while(1){
    	lastWakeTime = xTaskGetTickCount();

	    // acquire latest sensor data
	    sensorsAcquire(&sensorData, tick);
		//motorsSetRatio(MOTOR_M2, 0x0FFF);

	    // normalize acc readings
	    w = sqrtf(sensorData.acc.x*sensorData.acc.x + sensorData.acc.y*sensorData.acc.y + sensorData.acc.z*sensorData.acc.z);
		sensorData.acc.x /= w;  // (CHANGE SIGN?)
	    sensorData.acc.y /= w;
	    sensorData.acc.z /= w;  // (CHANGE SIGN?)

	    // estimate roll and pitch from accelerometer
	    roll_acc = atan2f(sensorData.acc.y, sensorData.acc.z);
	    pitch_acc = atan2f(-sensorData.acc.x, sqrtf(sensorData.acc.y*sensorData.acc.y + sensorData.acc.z*sensorData.acc.z));
	    roll_acc /= DEG2RAD;  // convert to degrees
	    pitch_acc /= DEG2RAD;

	    // estimate angles from gyro
	    roll_gyro += Ts * sensorData.gyro.x;  // sensor data is in degrees
	    pitch_gyro += Ts * sensorData.gyro.y;
        accumulated_gyro_roll = roll_gyro;
        accumulated_pitch_roll = pitch_gyro;

	    // combine estimates - surround with a semaphore
	    xSemaphoreTake(writingAngle, portMAX_DELAY);
	    // roll and pitch in degrees
	    roll = 0.9f * roll_acc + 0.1f * roll_gyro;
	    pitch = 0.9f * pitch_acc + 0.1f * pitch_gyro;
	    pitch = pitch;

	    // update ang. velocity measurements
	    droll = sensorData.gyro.x;
	    dpitch = sensorData.gyro.y;
	    dyaw = sensorData.gyro.z;
	    xSemaphoreGive(writingAngle);

	    //motorsSetRatio(MOTOR_M3, 0x0FFF);

	    tick++;

	    //if (sensorsInit){
            //DEBUG_PRINT("\nRoll angle: %f\n", roll);
            //DEBUG_PRINT("\nPitch angle: %f\n", pitch);
	    //}
        sensorsInit = 0;
	    vTaskDelayUntil(&lastWakeTime, M2T(10));  // allow other tasks to execute
    }
}

// reference
void getRef(void *pvParameters){
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1){
		xSemaphoreTake(writingRef, portMAX_DELAY);
		commanderGetSetpoint(&setpoint, &state);  // update state?
		xSemaphoreGive(writingRef);
		vTaskDelayUntil(&lastWakeTime, M2T(10));
	}
}

// controller
void cant(void *pvParameters){
	uint32_t lastWakeTime;

	// states (roll, pitch, droll, dpitch, dyaw, iroll, ipitch, idyaw)
	float ref_pitch = 0.0f, ref_roll = 0.0f, ref_yaw = 0.0f;
	float x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // elements of the K matrix
    float K_roll = 0.0586; //0.00389;
    float K_pitch = 0.0602; //0.0050;
    float K_droll = 0.0108; //0.0009;
    float K_dpitch = 0.0111; //0.0010;
    //float K_ipitch = 0.0048;
    float K_ref_roll = -0.0586;
    float K_ref_pitch = -0.0602;

    // float K_roll = 0.00389;
    // float K_pitch = 0.0050; 
    // float K_droll = 0.0009; 
    // float K_dpitch = 0.0010;
    // float K_ref_roll = 0.0;
    // float K_ref_pitch = 0.0;


    float K[4][6] = {{-K_roll, -K_pitch, -K_droll, -K_dpitch, K_ref_roll,  K_ref_pitch},
               		 {-K_roll,  K_pitch, -K_droll,  K_dpitch, K_ref_roll, -K_ref_pitch},
               		 { K_roll,  K_pitch,  K_droll,  K_dpitch, -K_ref_roll, -K_ref_pitch},
               		 { K_roll, -K_pitch,  K_droll, -K_dpitch, -K_ref_roll,  K_ref_pitch}};

	// control input
    float u[] = {0.0, 0.0, 0.0, 0.0};
    float base_thrust = 0.0;
    uint16_t uPWM[4];

    xSemaphoreTake(sensorsCalibrated, portMAX_DELAY);
	DEBUG_PRINT("\nTurning controller on.\n");

    while(1){
    	lastWakeTime = xTaskGetTickCount();

		// read the measurements
		xSemaphoreTake(writingAngle, portMAX_DELAY);
		x[0] = roll * DEG2RAD;  // convert to radians for controller calculations
		x[1] = pitch * DEG2RAD;
		x[2] = droll * DEG2RAD;
		x[3] = dpitch * DEG2RAD;
		xSemaphoreGive(writingAngle);

		// read reference
		xSemaphoreTake(writingRef, portMAX_DELAY);
		ref_roll = setpoint.attitude.roll*DEG2RAD;
		ref_pitch = -setpoint.attitude.pitch*DEG2RAD;
		base_thrust = setpoint.thrust;
		ref_yaw = setpoint.attitude.yaw;  // used as a switch
		xSemaphoreGive(writingRef);

		x[4] = ref_roll;
        x[5] = ref_pitch;

        for (int i=0;i<4;i++){
        	u[i] = 0.0f;
        	for (int j=0;j<6;j++){  // change index
        		u[i] += -10*K[i][j] * x[j];
        	}
        	u[i] = u[i] * 65535 / 0.06f / 9.81f;
        	if (ref_roll < 0.0f){
        		u[i] += base_thrust;
        	} else u[i] = base_thrust;
        	

        	// saturation [0,65535]
        	u[i] = u[i] < 0.0f ? 0.0f : u[i];
        	u[i] = u[i] > 65535.0f ? 65535.0f : u[i];
        	uPWM[i] = (uint16_t) u[i];
        	//uPWM[i] = uPWM[i] + 0x1388;  // add base thrust (4E20 <=> 20000)
        	uPWM[i] = uPWM[i] < 0x88B8 ? uPWM[i] : 0x88B8;//0.0010;20;  // clip to 10000
        }

        if (ref_roll > 0.0f) {
			motorsSetRatio(MOTOR_M1, 0x0FFF);
			motorsSetRatio(MOTOR_M2, 0x0FFF);
			motorsSetRatio(MOTOR_M3, 0x0000);
			motorsSetRatio(MOTOR_M4, 0x0000);
		} else if (ref_pitch > 0.0f) {
			motorsSetRatio(MOTOR_M1, 0x0FFF);
			motorsSetRatio(MOTOR_M2, 0x0000);
			motorsSetRatio(MOTOR_M3, 0x0000);
			motorsSetRatio(MOTOR_M4, 0x0FFF);
		} else if (x[0] > 1.3f || x[0] < -1.3f || x[1] > 1.3f || x[1] < -1.3f) {
			motorsSetRatio(MOTOR_M1, 0x0000);
			motorsSetRatio(MOTOR_M2, 0x0000);
			motorsSetRatio(MOTOR_M3, 0x0000);
			motorsSetRatio(MOTOR_M4, 0x0000);
		} else {
			motorsSetRatio(MOTOR_M1, uPWM[0]);
			motorsSetRatio(MOTOR_M2, uPWM[1]);
			motorsSetRatio(MOTOR_M3, uPWM[2]);
			motorsSetRatio(MOTOR_M4, uPWM[3]);
		}

  //       ctrl1 = uPWM[0];
  //       ctrl2 = uPWM[1];
  //       ctrl3 = uPWM[2];
  //       ctrl4 = uPWM[3];

		// // set motor speed
		// // uint16_t value_i
		// motorsSetRatio(MOTOR_M1, uPWM[0]);
		// motorsSetRatio(MOTOR_M2, uPWM[1]);
		// motorsSetRatio(MOTOR_M3, uPWM[2]);
		// motorsSetRatio(MOTOR_M4, uPWM[3]);

		vTaskDelayUntil(&lastWakeTime, M2T(10));
	}
}

// log the estimated angles
LOG_GROUP_START(testinglog)
LOG_ADD(LOG_FLOAT, roll, &roll)
LOG_ADD(LOG_FLOAT, pitch, &pitch)
LOG_ADD(LOG_FLOAT, droll, &droll)
LOG_ADD(LOG_FLOAT, dpitch, &dpitch)
LOG_ADD(LOG_FLOAT, m1, &ctrl1)
LOG_ADD(LOG_FLOAT, m2, &ctrl2)
LOG_ADD(LOG_FLOAT, m3, &ctrl3)
LOG_ADD(LOG_FLOAT, m4, &ctrl4)
LOG_ADD(LOG_FLOAT, accum_gyro, &accumulated_gyro_roll)
LOG_ADD(LOG_FLOAT, accum_pitch, &accumulated_pitch_roll)
LOG_GROUP_STOP(testinglog)


/*************************************************
 * WAIT FOR SENSORS TO BE CALIBRATED
 ************************************************/
// lastWakeTime = xTaskGetTickCount ();
// while(!sensorsAreCalibrated()) {
//     vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
// }



/*************************************************
 * RETRIEVE THE MOST RECENT SENSOR DATA
 *
 * The code creates a variable called sensorData and then calls a function
 * that fills this variable with the latest data from the sensors.
 *
 * sensorData_t sensorData = struct {
 *     Axis3f acc;
 *     Axis3f gyro;
 *     Axis3f mag;
 *     baro_t baro;
 *     zDistance_t zrange;
 *     point_t position;
 * }
 *
 ************************************************/
// sensorData_t sensorData;
// sensorsAcquire(&sensorData);



/*************************************************
 * RETRIEVE THE SET POINT FROM ANY EXTERNAL COMMAND INTERFACE
 *
 * The code creates a variable called setpoint and then calls a function
 * that fills this variable with the latest command input.
 *
 * setpoint_t setpoint = struct {
 *     uint32_t timestamp;
 *
 *     attitude_t attitude;      // deg
 *     attitude_t attitudeRate;  // deg/s
 *     quaternion_t attitudeQuaternion;
 *     float thrust;
 *     point_t position;         // m
 *     velocity_t velocity;      // m/s
 *     acc_t acceleration;       // m/s^2
 *     bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame
 *
 *     struct {
 *         stab_mode_t x;
 *         stab_mode_t y;
 *         stab_mode_t z;
 *         stab_mode_t roll;
 *         stab_mode_t pitch;
 *         stab_mode_t yaw;
 *         stab_mode_t quat;
 *     } mode;
 * }
 *
 ************************************************/
// setpoint_t setpoint;
// commanderGetSetpoint(&setpoint);



/*************************************************
 * SENDING OUTPUT TO THE MOTORS
 *
 * The code sends an output to each motor. The output should have the be
 * of the typ unsigned 16-bit integer, i.e. use variables such as:
 * uint16_t value_i
 *
 ************************************************/
// motorsSetRatio(MOTOR_M1, value_1);
// motorsSetRatio(MOTOR_M2, value_2);
// motorsSetRatio(MOTOR_M3, value_3);
// motorsSetRatio(MOTOR_M4, value_4);


/*************************************************
 * LOGGING VALUES THAT CAN BE PLOTTEN IN PYTHON CLIENT
 *
 * We have already set up three log blocks to for the accelerometer data, the
 * gyro data and the setpoints, just uncomment the block to start logging. Use
 * them as reference if you want to add custom blocks.
 *
 ************************************************/


LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)



LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)


/*
LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)
*/
