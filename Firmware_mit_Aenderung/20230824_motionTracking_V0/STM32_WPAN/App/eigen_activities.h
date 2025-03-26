/*
 * eigen_activities.h
 *
 *  Created on: Dec 28, 2023
 *      Author: Linda Hahn
 */

#ifndef APP_EIGEN_ACTIVITIES_H_
#define APP_EIGEN_ACTIVITIES_H_

#define NUMBER_OF_MAX_SAVED_DATA 3000
#define BYTES_IN_SAVED_DATA 36

#define ACCELEROMETER_VALUES_PER_SIGNAL 256  // Do not change the value of 256, because of meanX = sumAccX >> 8 ...
#define SIGNALS_PER_TRANSMISSION 118						// 16 für 1,5 min;   118 für 10 Minuten
#define SIGNALS_PER_TRANSMISSION_CASE_NO_ACTIVITY 350		// 32 für 3 min;     350 für 30 Minuten
#define MAX_NUMBER_NO_ACTIVITY_SIGNALS 7031					// 48 für 4 Minuten; 7031 für 10 Stunden

// Get_Y_Axis_Orientation and Get_XY_Axis_Orientation
#define MEAN_LIMIT_FOR_Y_UP 800
#define MEAN_LIMIT_FOR_Y_DOWN -800
#define MEAN_LIMIT_FOR_X_ORIENTATION 0

// Get_Posture
#define MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP 600
#define MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN -600
#define MEAN_X_LIMIT_FOR_POSTURES 0
#define MEAN_Z_LIMIT_FOR_POSTURES 0

// Movement differentiation
#define Y_STARTING_RANGE_VALUE_FOR_JUMPS_IF_Y_UP 4000
#define Y_VALUE_TO_TERMINATE_JUMPS_IF_Y_UP 100
#define Y_STARTING_RANGE_VALUE_FOR_JUMPS_IF_Y_DOWN -4000
#define Y_VALUE_TO_TERMINATE_JUMPS_AND_RUNS_IF_Y_DOWN -100
#define MAX_NUMBER_VALUES_TO_DETECT_JUMPS 40	// if 50 Hz

#define Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_UP 2500
#define Y_VALUE_TO_TERMINATE_RUNS_IF_Y_UP 100
#define Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_DOWN -2500
#define Y_VALUE_TO_TERMINATE_JUMPS_AND_RUNS_IF_Y_DOWN -100
#define MAX_NUMBER_VALUES_TO_DETECT_RUNS 40			// if 50 Hz

#define Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_IF_Y_UP 1300
#define Y_VALUE_TO_TERMINATE_WALKINGSTEPS_IF_Y_UP 800
#define Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_IF_Y_DOWN -1300
#define Y_VALUE_TO_TERMINATE_WALKINGSTEPS_IF_Y_DOWN -800
#define MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS_ONLY 60   		// if 50 Hz

#define X_STARTING_RANGE_VALUE_FOR_SITUPS_IF_X_FORWARD 600
#define X_STARTING_RANGE_VALUE_FOR_SITUPS_IF_X_BACKWARD -600
#define X_VALUE_TO_TERMINATE_SITUPS_IF_X_FORWARD 300
#define X_VALUE_TO_TERMINATE_SITUPS_IF_X_BACKWARD -300
#define Y_LIMIT_FOR_SITUPS_IF_Y_UP 100
#define Y_LIMIT_FOR_SITUPS_IF_Y_DOWN -100

// Only if function Count_Pushups used
#define X_LIMIT_FOR_PUSHUPS_IF_X_FORWARD -700
#define X_LIMIT_FOR_PUSHUPS_IF_X_BACKWARD 700
#define Y_LIMIT_FOR_PUSHUPS_IF_Y_UP 600
#define Y_LIMIT_FOR_PUSHUPS_IF_Y_DOWN -600
#define Z_LIMIT_FOR_PUSHUPS 700
#define Z_VALUE_TO_TERMINATE_PUSHUPS 500
#define MIN_NUMBER_VALUES_TO_DETECT_PUSHUPS 30		// if 50 Hz
#define MAX_NUMBER_VALUES_TO_DETECT_PUSHUPS 90		// if 50 Hz

// Only if function Count_WalkingSteps_And_Squats used
#define Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_AND_SQUATS_IF_Y_UP 1300
#define Y_VALUE_TO_TERMINATE_WALKINGSTEPS_AND_SQUATS_IF_Y_UP 800
#define Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_AND_SQUATS_IF_Y_DOWN -1300
#define Y_VALUE_TO_TERMINATE_WALKINGSTEPS_AND_SQUATS_IF_Y_DOWN -800
// #define MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS 50 		// if 100 Hz
// #define MAX_NUMBER_VALUES_TO_DETECT_SQUATS 80         	// if 100 Hz
#define MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS 25   	// if 50 Hz
#define MAX_NUMBER_VALUES_TO_DETECT_SQUATS 60         	// if 50 Hz
#define X_MIN_LIMIT_FOR_SQUATS_IF_X_FORWARD -700
#define X_MAX_LIMIT_FOR_SQUATS_IF_X_FORWARD -170
#define X_MIN_LIMIT_FOR_SQUATS_IF_X_BACKWARD 300
#define X_MAX_LIMIT_FOR_SQUATS_IF_X_BACKWARD 800
#define RESET_VALUE_X_MIN_IF_X_FORWARD 0
#define RESET_VALUE_X_MIN_IF_X_BACKWARD 800
#define RESET_VALUE_X_MAX_IF_X_FORWARD -800
#define RESET_VALUE_X_MAX_IF_X_BACKWARD 0

// Only if function Detect_Fall used
#define NUMBER_VALUES_FOR_ABSOLUTE_MAXIMA_DETERMINATION 25	// if 50 Hz
#define FALL_LIKE_ACCELERATION 1000
#define Y_LIMIT_FOR_LYING_IF_Y_UP 300
#define Y_LIMIT_FOR_LYING_IF_Y_DOWN -300
#define Y_LIMIT_FOR_UPRIGHT_IF_Y_UP 800
#define Y_LIMIT_FOR_UPRIGHT_IF_Y_DOWN -800


typedef enum
{
	False,
	True
} bool;

typedef enum
{
	yUndefined,
	yUp,
	yDown

} yAxisOrientations;

typedef enum
{
	xyUndefined,
	yUpXForward,		// front left y up
	yUpXBackward,		// front right y up
	yDownXForward,		// front right y down
	yDownXBackward		// front left y down
} xyAxisOrientations;

typedef enum
{
	postureUndefined,
	postureLying,
	postureUpright,
	postureLyingOnBack,
	postureLyingOnFront,
	postureLyingOnRightSide,
	postureLyingOnLeftSide
} postures;

typedef enum
{
	activityLevelZero,
	activityLevelVeryLow,
	activityLevelLow,
	activityLevelMedium,
	activityLevelHigh,
	activityLevelVeryHigh
} activityLevels;


typedef struct
{
	uint8_t Year;
	uint8_t Month;
	uint8_t Date;
	uint8_t Hours;
	uint8_t Minutes;
	uint8_t Seconds;
} datetimeTypeDef;


void Determine_RawData(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ);
void Determine_Activities(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ);
uint8_t Get_Y_Axis_Orientation(int16_t meanY);
uint8_t Get_XY_Axis_Orientation(int16_t meanX, int16_t meanY);
void Count_Jumps(int16_t accelerationValueY, uint8_t yAxisOrientation);
void Count_Runs(int16_t accelerationValueY, uint8_t yAxisOrientation);
void Count_WalkingSteps(int16_t accelerationValueY, uint8_t yAxisOrientation);
void Count_WalkingSteps_And_Squats(int16_t accelerationValueX, int16_t accelerationValueY, uint8_t xyAxisOrientation, uint8_t yAxisOrientation);
void Count_Situps(int16_t accelerationValueX, int16_t accelerationValueY, uint8_t xyAxisOrientation, uint8_t yAxisOrientation);
void Count_Pushups(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ, uint8_t xyAxisOrientation);
void Detect_Fall(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ);
uint8_t Get_Posture(int16_t meanX, int16_t meanY, int16_t meanZ, uint8_t xyAxisOrientation);
uint8_t Get_Short_Term_Activity_Level();
uint8_t Get_Long_Term_Activity_Level();
void Get_maxAbsoluteOverallAcceleration_And_sumMeanAbsoluteOverallAcceleration(int16_t meanX, int16_t meanY, int16_t meanZ);
void Get_Average_Speed();
void Save_Data(uint8_t shortTermActivityLevel, uint8_t longTermActivityLevel);
uint16_t Get_Number_Saved_Data();
void Move_NextOut();
void Move_NextIn();


#endif /* APP_EIGEN_ACTIVITIES_H_ */
