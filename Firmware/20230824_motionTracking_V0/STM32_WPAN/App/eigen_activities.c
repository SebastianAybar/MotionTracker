/*
 * eigen_activities.c
 *
 *  Created on: Dec 28, 2023
 *      Author: Linda Hahn
 */

#include "custom_stm.h"
#include "custom_app.h"
#include "eigen_activities.h"
#include <math.h>
#include <stdlib.h>
#include "ble.h"
#include "BMI323_eigen.h"

extern uint32_t countedSteps;
extern RTC_HandleTypeDef hrtc;

uint8_t savedData[NUMBER_OF_MAX_SAVED_DATA][BYTES_IN_SAVED_DATA] = {0};
uint16_t nextIn = 0;
uint16_t nextOut = 0;

// Determine_RawData
uint8_t rawData[768] = {0};
uint16_t accelerationValueCounterForRawData = 0;

// Determine_Activities
uint8_t postureOld = postureUndefined;
uint16_t accelerationValueCounter = 0;
uint16_t receivedSignalsCounter = 0;
uint16_t noActivitySignalsCounter = 0;
int16_t accX[ACCELEROMETER_VALUES_PER_SIGNAL] = {0};
int16_t accY[ACCELEROMETER_VALUES_PER_SIGNAL] = {0};
int16_t accZ[ACCELEROMETER_VALUES_PER_SIGNAL] = {0};
int32_t sumAccX = 0;
int32_t sumAccY = 0;
int32_t sumAccZ = 0;
float maxAbsoluteOverallAcceleration = 0.0;
float sumMeanAbsoluteOverallAcceleration = 0.0;
uint32_t averageSpeed = 0.0;
uint8_t dateTimeArray[6] = {0};
datetimeTypeDef datetimeStart = {0};
datetimeTypeDef datetimeEnd = {0};

// Movement differentiation
uint8_t yAxisOrientation = yUndefined;
uint8_t xyAxisOrientation = xyUndefined;
uint8_t yRange4jumpsReached = False;
uint8_t yRange4runsReached = False;
uint8_t yRange4walkingStepsReached = False;
uint8_t situpStart = False;
uint8_t pushupStart = False;
uint8_t counter4valuesInRangeJumps = 0;
uint8_t counter4valuesInRangeRuns = 0;
uint8_t counter4valuesInRangeWalkingSteps = 0;
uint8_t counter4valuesInRangePushups = 0;
uint16_t jumps = 0;
uint16_t runs = 0;
uint16_t walkingSteps = 0;
uint16_t situps = 0;

// Only if function Count_Pushups used
uint16_t pushups = 0;

// Only if function Count_WalkingSteps_And_Squats used
uint16_t squats = 0;
int16_t xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
int16_t xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
int16_t xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
int16_t xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;

// Fall detection
uint8_t fallWatchCounter = 0;
uint16_t absMaxX = 0;
uint16_t absMaxY = 0;
uint16_t absMaxZ = 0;
uint16_t prevAbsMaxX = 0;
uint16_t prevAbsMaxY = 0;
uint16_t prevAbsMaxZ = 0;
uint8_t fallLikeAccelerations = False;
uint8_t fallDetected = False;
uint8_t fallConfirmed = False;
uint8_t uprightAfterFall = False;
uint8_t remainedUprightAfterFall = False;
uint16_t fallDetectCounter = 0;
int32_t sumAccY4Detect = 0;
uint16_t fallConfCounter = 0;
int32_t sumAccY4Fall= 0;

//uint16_t counter = 0;	// Counts the calls of Determine_Activities()

void Determine_RawData(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ) {

    rawData[3 * accelerationValueCounterForRawData + 1] = accelerationValueX;
    rawData[3 * accelerationValueCounterForRawData + 2] = accelerationValueY;
    rawData[3 * accelerationValueCounterForRawData + 3] = accelerationValueZ;
 
    accelerationValueCounterForRawData ++;

}


void Determine_Activities(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ)
{
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef rtcDate;

	uint8_t yAxisOrientationNew = yUndefined;
	uint8_t xyAxisOrientationNew = xyUndefined;
	uint8_t postureNew = postureUndefined;
	uint8_t shortTermActivityLevel = activityLevelZero;
	uint8_t longTermActivityLevel = activityLevelZero;
	int16_t meanX = 0;
	int16_t meanY = 0;
	int16_t meanZ = 0;

	if(accelerationValueCounter == 0 && receivedSignalsCounter == 0)
	{	// Start of transmission unit
		HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
		printf("\r\n%d:%d:%d:%lu eigen_activities.c accel counter start time\r\n",
				rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds, 
                1000* (rtcTime.SecondFraction - rtcTime.SubSeconds) / (rtcTime.SecondFraction +1));
		printf("%d:%d:%d eigen_activities.c accel counter start date\r\n", rtcDate.Year, rtcDate.Month, rtcDate.Date);
		printf("Got datetimeStart. \r\n\r\n");
		datetimeStart.Year = rtcDate.Year;
		datetimeStart.Month = rtcDate.Month;
		datetimeStart.Date = rtcDate.Date;
		datetimeStart.Hours = rtcTime.Hours;
		datetimeStart.Minutes = rtcTime.Minutes;
		datetimeStart.Seconds = rtcTime.Seconds;
	}

	accX[accelerationValueCounter] = accelerationValueX;
	accY[accelerationValueCounter] = accelerationValueY;
	accZ[accelerationValueCounter] = accelerationValueZ;

	sumAccX += accelerationValueX;
	sumAccY += accelerationValueY;
	sumAccZ += accelerationValueZ;

	accelerationValueCounter ++;

	// Detect and count movements
	Count_Jumps(accelerationValueY, yAxisOrientation);
	Count_Runs(accelerationValueY, yAxisOrientation);
	// Use function Count_WalkingSteps or Count_WalkingSteps_And_Squats to avoid double counting of walking steps!!!
    // Count_WalkingSteps(accelerationValueY, yAxisOrientation);
	Count_WalkingSteps_And_Squats(accelerationValueX, accelerationValueY, xyAxisOrientation, yAxisOrientation);
	Count_Situps(accelerationValueX, accelerationValueY, xyAxisOrientation, yAxisOrientation);
	Count_Pushups(accelerationValueX, accelerationValueY, accelerationValueZ, xyAxisOrientation);
    // Detect fall (needs a method to send a warning)
    // Detect_Fall(accelerationValueX, accelerationValueY, accelerationValueZ);


	// Signal is complete
	if(accelerationValueCounter == ACCELEROMETER_VALUES_PER_SIGNAL)
	{
		accelerationValueCounter = 0;
		receivedSignalsCounter ++;
		// Calculate mean for each axis (meanX = sumAccX / ACCELEROMETER_VALUES_PER_SIGNAL)
		meanX = sumAccX >> 8;
		meanY = sumAccY >> 8;
		meanZ = sumAccZ >> 8;

		sumAccX = 0;
		sumAccY = 0;
		sumAccZ = 0;

		yAxisOrientationNew = Get_Y_Axis_Orientation(meanY);
		xyAxisOrientationNew = Get_XY_Axis_Orientation(meanX, meanY);
		postureNew = Get_Posture(meanX, meanY, meanZ, xyAxisOrientation);

        
		printf("postureNew = %d\r\n", postureNew);
        
		if (yAxisOrientation != yAxisOrientationNew)
		{
			yAxisOrientation = yAxisOrientationNew;
			printf("y-Axisorientation = %d\r\n", yAxisOrientation);
		}

		if (xyAxisOrientation != xyAxisOrientationNew && xyAxisOrientationNew != xyUndefined)
		{
			xyAxisOrientation = xyAxisOrientationNew;
			printf("xy-Axisorientation = %d\r\n", xyAxisOrientation);
		}



		// First data set or data for old posture has just been saved
		if(postureOld == postureUndefined)
		{
			postureOld = postureNew;
		}

		// Posture changed
		if(postureOld != postureNew)
		{
            shortTermActivityLevel = Get_Short_Term_Activity_Level();
            longTermActivityLevel = Get_Long_Term_Activity_Level();
            Get_Average_Speed();    
            Save_Data(shortTermActivityLevel, longTermActivityLevel);
            Get_maxAbsoluteOverallAcceleration_And_sumMeanAbsoluteOverallAcceleration(meanX, meanY, meanZ);

            // Set variables for new signal
            postureOld = postureNew;
            receivedSignalsCounter = 1;
            datetimeStart = datetimeEnd;
		}

		// Posture didn´t change
		else
		{
			Get_maxAbsoluteOverallAcceleration_And_sumMeanAbsoluteOverallAcceleration(meanX, meanY, meanZ);
			shortTermActivityLevel = Get_Short_Term_Activity_Level();
			longTermActivityLevel = Get_Long_Term_Activity_Level();
			Get_Average_Speed();
			// No activity
			if(shortTermActivityLevel == activityLevelZero && longTermActivityLevel == activityLevelZero)
			{
				noActivitySignalsCounter ++;
				// Limit for lack of activity is reached
				if(receivedSignalsCounter == SIGNALS_PER_TRANSMISSION_CASE_NO_ACTIVITY)
				{
					Save_Data(shortTermActivityLevel, longTermActivityLevel);
					postureOld = postureUndefined;
					receivedSignalsCounter = 0;
				}
				if(noActivitySignalsCounter == MAX_NUMBER_NO_ACTIVITY_SIGNALS)
				{
					printf("SEIT 10 STUNDEN KEINE AKTIVITÄT!!!");	// Replace by a method to send a warning
					noActivitySignalsCounter = 0;
				}
			}
			// One activity level is greater zero
			else
			{
				// Transmission unit is complete
				if(receivedSignalsCounter >= SIGNALS_PER_TRANSMISSION)
				{
					shortTermActivityLevel = Get_Short_Term_Activity_Level();
					longTermActivityLevel = Get_Long_Term_Activity_Level();
					Get_Average_Speed();
					Save_Data(shortTermActivityLevel, longTermActivityLevel);
					postureOld = postureUndefined;
					receivedSignalsCounter = 0;
					noActivitySignalsCounter = 0;
				}
			}
		}
	}
}


uint8_t Get_Y_Axis_Orientation(int16_t meanY)
{
    if(meanY > MEAN_LIMIT_FOR_Y_UP)
    {
    	return yUp;
    }
    if(meanY < MEAN_LIMIT_FOR_Y_DOWN)
    {
    	return yDown;
    }
    return yUndefined;
}

uint8_t Get_XY_Axis_Orientation(int16_t meanX, int16_t meanY)
{
	if(meanY > MEAN_LIMIT_FOR_Y_UP && meanX < MEAN_LIMIT_FOR_X_ORIENTATION)
	{
		return yUpXForward;
	}
    if(meanY > MEAN_LIMIT_FOR_Y_UP && meanX > MEAN_LIMIT_FOR_X_ORIENTATION)
    {
    	return yUpXBackward;
    }
    if(meanY < MEAN_LIMIT_FOR_Y_DOWN && meanX < MEAN_LIMIT_FOR_X_ORIENTATION)
    {
    	return yDownXForward;
    }
    if(meanY < MEAN_LIMIT_FOR_Y_DOWN && meanX > MEAN_LIMIT_FOR_X_ORIENTATION)
    {
    	return yDownXBackward;
    }
    return xyUndefined;
}

void Count_Jumps(int16_t accelerationValueY, uint8_t yAxisOrientation)
{
	if(yAxisOrientation == yUp)
	{
	    if(accelerationValueY > Y_STARTING_RANGE_VALUE_FOR_JUMPS_IF_Y_UP)
	    {
	        yRange4jumpsReached = True;
	    }
	    if(yRange4jumpsReached == True)
	    {
	        counter4valuesInRangeJumps ++;
	    }
	    if(yRange4jumpsReached == True && accelerationValueY < Y_VALUE_TO_TERMINATE_JUMPS_IF_Y_UP)
	    {
	        jumps ++;
	        printf("jumps = %d\r\n", jumps);
	        yRange4jumpsReached = False;
	        yRange4runsReached = False;
	        yRange4walkingStepsReached = False;
	        counter4valuesInRangeJumps = 0;
	        counter4valuesInRangeRuns = 0;
	        counter4valuesInRangeWalkingSteps = 0;

	        // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
	    }
	    if(counter4valuesInRangeJumps > MAX_NUMBER_VALUES_TO_DETECT_JUMPS)
	    {
	        jumps ++;
			printf("jumps = %d\r\n", jumps);
	        yRange4jumpsReached = False;
	        yRange4runsReached = False;
	        yRange4walkingStepsReached = False;
	        counter4valuesInRangeJumps = 0;
	        counter4valuesInRangeRuns = 0;
	        counter4valuesInRangeWalkingSteps = 0;

	        // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
	    }
	}

	if(yAxisOrientation == yDown)
	{
	    if(accelerationValueY < Y_STARTING_RANGE_VALUE_FOR_JUMPS_IF_Y_DOWN)
	    {
	        yRange4jumpsReached = True;
	    }
	    if(yRange4jumpsReached == True)
	    {
	        counter4valuesInRangeJumps ++;
	    }
	    if(yRange4jumpsReached == True && accelerationValueY > Y_VALUE_TO_TERMINATE_JUMPS_AND_RUNS_IF_Y_DOWN)
	    {
	        jumps ++;
	        printf("jumps = %d\r\n", jumps);
	        yRange4jumpsReached = False;
	        yRange4runsReached = False;
	        yRange4walkingStepsReached = False;
	        counter4valuesInRangeJumps = 0;
	        counter4valuesInRangeRuns = 0;
	        counter4valuesInRangeWalkingSteps = 0;

	        // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
	    }
	    if(counter4valuesInRangeJumps > MAX_NUMBER_VALUES_TO_DETECT_JUMPS)
	    {
	        jumps ++;
			printf("jumps = %d\r\n", jumps);
	        yRange4jumpsReached = False;
	        yRange4runsReached = False;
	        yRange4walkingStepsReached = False;
	        counter4valuesInRangeJumps = 0;
	        counter4valuesInRangeRuns = 0;
	        counter4valuesInRangeWalkingSteps = 0;

	        // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
	    }
	}
}

void Count_Runs(int16_t accelerationValueY, uint8_t yAxisOrientation)
{
    if(yAxisOrientation == yUp)
    {
        if(accelerationValueY > Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_UP
            && accelerationValueY <= Y_STARTING_RANGE_VALUE_FOR_JUMPS_IF_Y_UP)
        {
            yRange4runsReached = True;
        }
        if(yRange4runsReached == True)
        {
            counter4valuesInRangeRuns ++;
        }
        if(yRange4runsReached == True && accelerationValueY < Y_VALUE_TO_TERMINATE_RUNS_IF_Y_UP)
        {
            runs ++;
            printf("runs = %d\r\n", runs);
            yRange4runsReached = False;
            yRange4walkingStepsReached = False;
            counter4valuesInRangeRuns = 0;
            counter4valuesInRangeWalkingSteps = 0;

            // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
        if(counter4valuesInRangeRuns > MAX_NUMBER_VALUES_TO_DETECT_RUNS && yRange4jumpsReached == False)
        {
            runs ++;
			printf("runs = %d\r\n", runs);
            yRange4runsReached = False;
            yRange4walkingStepsReached = False;
            counter4valuesInRangeRuns = 0;
            counter4valuesInRangeWalkingSteps = 0;

            // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
    }

    if(yAxisOrientation == yDown)
    {
        if(accelerationValueY < Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_DOWN
            && accelerationValueY >= Y_STARTING_RANGE_VALUE_FOR_JUMPS_IF_Y_DOWN)
        {
            yRange4runsReached = True;
        }
        if(yRange4runsReached == True)
        {
            counter4valuesInRangeRuns ++;
        }
        if(yRange4runsReached == True && accelerationValueY > Y_VALUE_TO_TERMINATE_JUMPS_AND_RUNS_IF_Y_DOWN)
        {
            runs ++;
			printf("runs = %d\r\n", runs);
            yRange4runsReached = False;
            yRange4walkingStepsReached = False;
            counter4valuesInRangeRuns = 0;
            counter4valuesInRangeWalkingSteps = 0;

            // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
        if(counter4valuesInRangeRuns > MAX_NUMBER_VALUES_TO_DETECT_RUNS && yRange4jumpsReached == False)
        {
            runs ++;
			printf("runs = %d\r\n", runs);
            yRange4runsReached = False;
            yRange4walkingStepsReached = False;
            counter4valuesInRangeRuns = 0;
            counter4valuesInRangeWalkingSteps = 0;

            // Only if function Count_WalkingSteps_And_Squats used
	        xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
	        xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
	        xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MAX_IF_X_FORWARD;
	        xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
    }
}

void Count_WalkingSteps(int16_t accelerationValueY, uint8_t yAxisOrientation)
{
    if(yAxisOrientation == yUp)
    {
        if(accelerationValueY > Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_IF_Y_UP
            && accelerationValueY <= Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_UP)
        {
            yRange4walkingStepsReached = True;
        }
        if(yRange4walkingStepsReached == True)
        {
            counter4valuesInRangeWalkingSteps ++;
        }
        if(yRange4walkingStepsReached == True
            && accelerationValueY < Y_VALUE_TO_TERMINATE_WALKINGSTEPS_IF_Y_UP
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            walkingSteps ++;
			printf("walkingSteps = %d\r\n", walkingSteps);
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
        }
        if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS_ONLY
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            walkingSteps ++;
            printf("walkingSteps = %d\r\n", walkingSteps);
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
        }
    }

    if(yAxisOrientation == yDown)
    {
        if(accelerationValueY < Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_IF_Y_DOWN
            && accelerationValueY >= Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_DOWN)
        {
            yRange4walkingStepsReached = True;
        }
        if(yRange4walkingStepsReached == True)
        {
            counter4valuesInRangeWalkingSteps ++;
        }
        if(yRange4walkingStepsReached == True
            && accelerationValueY > Y_VALUE_TO_TERMINATE_WALKINGSTEPS_IF_Y_DOWN
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            walkingSteps ++;
			printf("walkingSteps = %d\r\n", walkingSteps);
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
        }
        if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS_ONLY
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            walkingSteps ++;
			printf("walkingSteps = %d\r\n", walkingSteps);
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
        }
    }
}

void Count_WalkingSteps_And_Squats(int16_t accelerationValueX, int16_t accelerationValueY, uint8_t xyAxisOrientation, uint8_t yAxisOrientation)
{
    if(xyAxisOrientation == yUpXForward && yAxisOrientation == yUp)
    {
        if(accelerationValueY > Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_AND_SQUATS_IF_Y_UP
            && accelerationValueY <= Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_UP)
        {
            yRange4walkingStepsReached = True;
        }
        if(yRange4walkingStepsReached == True)
        {
            counter4valuesInRangeWalkingSteps ++;
            if(accelerationValueX < xMinInWalkingStepsAndSquatsIfXForward)
            {
                xMinInWalkingStepsAndSquatsIfXForward = accelerationValueX;
            }
            if(accelerationValueX > xMaxInWalkingStepsAndSquatsIfXForward)
            {
                xMaxInWalkingStepsAndSquatsIfXForward = accelerationValueX;
            }
        }
        if(yRange4walkingStepsReached == True
            && accelerationValueY < Y_VALUE_TO_TERMINATE_WALKINGSTEPS_AND_SQUATS_IF_Y_UP
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS
                && xMinInWalkingStepsAndSquatsIfXForward > X_MIN_LIMIT_FOR_SQUATS_IF_X_FORWARD
                && xMaxInWalkingStepsAndSquatsIfXForward < X_MAX_LIMIT_FOR_SQUATS_IF_X_FORWARD)
            {
                squats ++;
                printf("squats = %d\r\n", squats);
            }
            else
            {
                walkingSteps ++;
				printf("walkingSteps = %d\r\n", walkingSteps);
            }
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
            xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
        }
        if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_SQUATS)
        {
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
            xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
        }
    }

    if (xyAxisOrientation == yUpXBackward && yAxisOrientation == yUp)
    {
        if(accelerationValueY > Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_AND_SQUATS_IF_Y_UP
            && accelerationValueY <= Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_UP)
        {
            yRange4walkingStepsReached = True;
        }
        if(yRange4walkingStepsReached == True)
        {
            counter4valuesInRangeWalkingSteps ++;
            if(accelerationValueX < xMinInWalkingStepsAndSquatsIfXBackward)
            {
                xMinInWalkingStepsAndSquatsIfXBackward = accelerationValueX;
            }
            if(accelerationValueX > xMaxInWalkingStepsAndSquatsIfXBackward)
            {
                xMaxInWalkingStepsAndSquatsIfXBackward = accelerationValueX;
            }
        }
        if(yRange4walkingStepsReached == True
            && accelerationValueY < Y_VALUE_TO_TERMINATE_WALKINGSTEPS_AND_SQUATS_IF_Y_UP
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS
                && xMinInWalkingStepsAndSquatsIfXBackward > X_MIN_LIMIT_FOR_SQUATS_IF_X_BACKWARD
                && xMaxInWalkingStepsAndSquatsIfXBackward < X_MAX_LIMIT_FOR_SQUATS_IF_X_BACKWARD)
            {
                squats ++;
				printf("squats = %d\r\n", squats);
            }
            else
            {
                walkingSteps ++;
				printf("walkingSteps = %d\r\n", walkingSteps);
            }
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
            xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
        if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_SQUATS)
        {
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
            xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
    }

    if(xyAxisOrientation == yDownXForward && yAxisOrientation == yDown)
    {
        if(accelerationValueY < Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_AND_SQUATS_IF_Y_DOWN
            && accelerationValueY >= Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_DOWN)
        {
            yRange4walkingStepsReached = True;
        }
        if(yRange4walkingStepsReached == True)
        {
            counter4valuesInRangeWalkingSteps ++;
            if(accelerationValueX < xMinInWalkingStepsAndSquatsIfXForward)
            {
                xMinInWalkingStepsAndSquatsIfXForward = accelerationValueX;
            }
            if(accelerationValueX > xMaxInWalkingStepsAndSquatsIfXForward)
            {
                xMaxInWalkingStepsAndSquatsIfXForward = accelerationValueX;
            }
        }
        if(yRange4walkingStepsReached == True
            && accelerationValueY > Y_VALUE_TO_TERMINATE_WALKINGSTEPS_AND_SQUATS_IF_Y_DOWN
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS
                && xMinInWalkingStepsAndSquatsIfXForward > X_MIN_LIMIT_FOR_SQUATS_IF_X_FORWARD
                && xMaxInWalkingStepsAndSquatsIfXForward < X_MAX_LIMIT_FOR_SQUATS_IF_X_FORWARD)
            {
                squats ++;
				printf("squats = %d\r\n", squats);
            }
            else
            {
                walkingSteps ++;
				printf("walkingSteps = %d\r\n", walkingSteps);
            }
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
            xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
        }
        if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_SQUATS)
        {
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
            xMaxInWalkingStepsAndSquatsIfXForward = RESET_VALUE_X_MIN_IF_X_FORWARD;
        }
    }

    if(xyAxisOrientation == yDownXBackward && yAxisOrientation == yDown)
    {
        if(accelerationValueY < Y_STARTING_RANGE_VALUE_FOR_WALKINGSTEPS_AND_SQUATS_IF_Y_DOWN
            && accelerationValueY >= Y_STARTING_RANGE_VALUE_FOR_RUNS_IF_Y_DOWN)
        {
            yRange4walkingStepsReached = True;
        }
        if(yRange4walkingStepsReached == True)
        {
            counter4valuesInRangeWalkingSteps ++;
        }
            if(accelerationValueX < xMinInWalkingStepsAndSquatsIfXBackward)
            {
                xMinInWalkingStepsAndSquatsIfXBackward = accelerationValueX;
            }
            if(accelerationValueX > xMaxInWalkingStepsAndSquatsIfXBackward)
            {
                xMaxInWalkingStepsAndSquatsIfXBackward = accelerationValueX;
            }
        if(yRange4walkingStepsReached == True
            && accelerationValueY > Y_VALUE_TO_TERMINATE_WALKINGSTEPS_AND_SQUATS_IF_Y_DOWN
            && yRange4jumpsReached == False && yRange4runsReached == False)
        {
            if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_WALKINGSTEPS
                && xMinInWalkingStepsAndSquatsIfXBackward > X_MIN_LIMIT_FOR_SQUATS_IF_X_BACKWARD
                && xMaxInWalkingStepsAndSquatsIfXBackward < X_MAX_LIMIT_FOR_SQUATS_IF_X_BACKWARD)
            {
                squats ++;
				printf("squats = %d\r\n", squats);
            }
            else
            {
                walkingSteps ++;
				printf("walkingSteps = %d\r\n", walkingSteps);
            }
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
            xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
        if(counter4valuesInRangeWalkingSteps > MAX_NUMBER_VALUES_TO_DETECT_SQUATS)
        {
            yRange4walkingStepsReached = False;
            counter4valuesInRangeWalkingSteps = 0;
            xMinInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MIN_IF_X_BACKWARD;
            xMaxInWalkingStepsAndSquatsIfXBackward = RESET_VALUE_X_MAX_IF_X_BACKWARD;
        }
    }
}

void Count_Situps(int16_t accelerationValueX, int16_t accelerationValueY, uint8_t xyAxisOrientation, uint8_t yAxisOrientation)
{
    if(yAxisOrientation == yUndefined)
    {
        if(xyAxisOrientation == yUpXForward)
        {
            if(accelerationValueX > X_STARTING_RANGE_VALUE_FOR_SITUPS_IF_X_FORWARD
                && accelerationValueY < Y_LIMIT_FOR_SITUPS_IF_Y_UP)
            {
                situpStart = True;
            }
        }

        if(xyAxisOrientation == yUpXBackward)
        {
            if(accelerationValueX < X_STARTING_RANGE_VALUE_FOR_SITUPS_IF_X_BACKWARD
                && accelerationValueY < Y_LIMIT_FOR_SITUPS_IF_Y_UP)
            {
                situpStart = True;
            }
        }

        if(xyAxisOrientation == yDownXForward)
        {
            if(accelerationValueX > X_STARTING_RANGE_VALUE_FOR_SITUPS_IF_X_FORWARD
                && accelerationValueY > Y_LIMIT_FOR_SITUPS_IF_Y_DOWN)
            {
                situpStart = True;
            }
        }

        if(xyAxisOrientation == yDownXBackward)
        {
            if(accelerationValueX < X_STARTING_RANGE_VALUE_FOR_SITUPS_IF_X_BACKWARD
                && accelerationValueY > Y_LIMIT_FOR_SITUPS_IF_Y_DOWN)
            {
                situpStart = True;
            }
        }


        if(xyAxisOrientation == yUpXForward || xyAxisOrientation == yDownXForward)
        {
            if(situpStart == True
                && accelerationValueX < X_VALUE_TO_TERMINATE_SITUPS_IF_X_FORWARD)
            {
                situps ++;
                situpStart = False;
				printf("situps = %d\r\n", situps);
            }
        }

        if(xyAxisOrientation == yUpXBackward || xyAxisOrientation == yDownXBackward)
        {
            if(situpStart == True
                && accelerationValueX > X_VALUE_TO_TERMINATE_SITUPS_IF_X_BACKWARD)
            {
                situps ++;
                situpStart = False;
				printf("situps = %d\r\n", situps);
            }
        }
    }
}

void Count_Pushups(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ, uint8_t xyAxisOrientation)
{
    if(xyAxisOrientation == yUpXForward || xyAxisOrientation == yDownXForward)
    {
        if(accelerationValueX < X_LIMIT_FOR_PUSHUPS_IF_X_FORWARD
            && accelerationValueY < Y_LIMIT_FOR_PUSHUPS_IF_Y_UP
            && accelerationValueY > Y_LIMIT_FOR_PUSHUPS_IF_Y_DOWN
            && accelerationValueZ > Z_LIMIT_FOR_PUSHUPS)
        {
            pushupStart = True;
        }
    }

    if(xyAxisOrientation == yUpXBackward || xyAxisOrientation == yDownXBackward)
    {
        if(accelerationValueX > X_LIMIT_FOR_PUSHUPS_IF_X_BACKWARD
            && accelerationValueY < Y_LIMIT_FOR_PUSHUPS_IF_Y_UP
            && accelerationValueY > Y_LIMIT_FOR_PUSHUPS_IF_Y_DOWN
            && accelerationValueZ > Z_LIMIT_FOR_PUSHUPS)
        {
            pushupStart = True;
        }
    }

    if(pushupStart == True)
    {
        counter4valuesInRangePushups ++;
        if(counter4valuesInRangePushups > MAX_NUMBER_VALUES_TO_DETECT_PUSHUPS)
        {
            pushupStart = False;
            counter4valuesInRangePushups = 0;
        }
    }
    if(pushupStart == True && accelerationValueZ < Z_VALUE_TO_TERMINATE_PUSHUPS)
    {
        if(counter4valuesInRangePushups > MIN_NUMBER_VALUES_TO_DETECT_PUSHUPS)
        {
            pushups ++;
            printf("pushups = %d\r\n", pushups);
        }
        pushupStart = False;
        counter4valuesInRangePushups = 0;
    }
}

void Detect_Fall(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ)
{
	int16_t meanAccY = 0;
	int16_t meanAccYDetect;

    if(abs(accelerationValueX) > absMaxX)
    {
        absMaxX = abs(accelerationValueX);
    }
    if(abs(accelerationValueY) > absMaxY)
    {
        absMaxY = abs(accelerationValueY);
    }
    if(abs(accelerationValueZ) > absMaxZ)
    {
        absMaxZ = abs(accelerationValueZ);
    }

    if(fallWatchCounter < NUMBER_VALUES_FOR_ABSOLUTE_MAXIMA_DETERMINATION)
    {
        fallWatchCounter ++;
    }
    else
    {
        fallWatchCounter = 0;
        if((absMaxX > FALL_LIKE_ACCELERATION || prevAbsMaxX > FALL_LIKE_ACCELERATION)
            && (absMaxY > FALL_LIKE_ACCELERATION || prevAbsMaxY > FALL_LIKE_ACCELERATION)
            && (absMaxZ > FALL_LIKE_ACCELERATION || prevAbsMaxZ > FALL_LIKE_ACCELERATION))
        {
            fallLikeAccelerations = True;
        }
        prevAbsMaxX = absMaxX;
        prevAbsMaxY = absMaxY;
        prevAbsMaxZ = absMaxZ;
        absMaxX = 0;
        absMaxY = 0;
        absMaxZ = 0;
    }

    if(fallLikeAccelerations == True)
    {
        fallDetectCounter ++;
        // Don´t use the first 52 values for calculation of meanAccYDetect. 564 - 52 = (1<<9)
        if(fallDetectCounter > 52 && fallDetectCounter < 564)
        {
            sumAccY4Detect += accelerationValueY;
            fallDetectCounter ++;
            if(fallDetectCounter == 564)
            {
                meanAccYDetect = sumAccY4Detect >> 9;
                sumAccY4Detect = 0;
                fallDetectCounter = 0;
                fallLikeAccelerations = False;
                if(meanAccYDetect < Y_LIMIT_FOR_LYING_IF_Y_UP && meanAccYDetect > Y_LIMIT_FOR_LYING_IF_Y_DOWN)
                {
                    fallDetected = True;
                }
            }
        }
    }

    if(fallDetected == True && fallConfirmed == False)
    {
        fallConfCounter ++;
        sumAccY4Fall += accelerationValueY;
        if(fallConfCounter == 2048)
        {
			meanAccY = sumAccY4Fall >> 11;
			sumAccY4Fall = 0;
			fallConfCounter = 0;
			if(meanAccY < Y_LIMIT_FOR_LYING_IF_Y_UP && meanAccY > Y_LIMIT_FOR_LYING_IF_Y_DOWN)
			{
				fallConfirmed = True;
				printf("Sturz bestätigt.");		// Replace by a method to send a message
				fallDetected = False;
				remainedUprightAfterFall = False;
			}
        }
    }

    if(fallConfirmed == True && uprightAfterFall == False)
    {
        fallConfCounter ++;
        sumAccY4Fall += accelerationValueY;
        if(fallConfCounter == 256)
        {
			meanAccY = sumAccY4Fall >> 8;
			sumAccY4Fall = 0;
			fallConfCounter = 0;
			if(meanAccY > Y_LIMIT_FOR_UPRIGHT_IF_Y_UP || meanAccY < Y_LIMIT_FOR_UPRIGHT_IF_Y_DOWN)
			{
				uprightAfterFall = True;
				printf("Nach Sturz aufrecht.");		// Replace by a method to send a message
			}
        }
    }

    if(fallConfirmed == True && uprightAfterFall == True)
    {
        fallConfCounter ++;
        sumAccY4Fall += accelerationValueY;
        if(fallConfCounter == 2048)
        {
			meanAccY = sumAccY4Fall >> 11;
			sumAccY4Fall = 0;
			fallConfCounter = 0;
			if(meanAccY > Y_LIMIT_FOR_UPRIGHT_IF_Y_UP || meanAccY < Y_LIMIT_FOR_UPRIGHT_IF_Y_DOWN)
			{
				fallConfirmed = False;
				remainedUprightAfterFall = True;
				printf("Aufrecht geblieben.");		// Replace by a method to send a message
			}
			uprightAfterFall = False;
        }
    }
}


uint8_t Get_Posture(int16_t meanX, int16_t meanY, int16_t meanZ, uint8_t xyAxisOrientation)
{
    if(meanY >= MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP || meanY <= MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN)
    {
    	return postureUpright;
    }

    if(xyAxisOrientation == xyUndefined)
    {
        if(meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN)
        {
        	return postureLying;
        }
    }

    if(xyAxisOrientation == yUpXForward)
    {
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
        {
        	return postureLyingOnBack;
        }
        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnFront;
		}
        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnRightSide;
		}
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnLeftSide;
		}
        return postureLying;
    }

    if(xyAxisOrientation == yUpXBackward)
    {
        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnBack;
		}
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnFront;
		}

        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnRightSide;
		}
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY < MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_UP && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnLeftSide;
		}
        return postureLying;
    }

    if(xyAxisOrientation == yDownXForward)
    {
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnBack;
		}
        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnFront;
		}
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnRightSide;
		}
        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnLeftSide;
		}
        return postureLying;
    }

    if(xyAxisOrientation == yDownXBackward)
    {
        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnBack;
		}
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnFront;
		}
        if(meanX > MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ < MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnRightSide;
		}
        if(meanX < MEAN_X_LIMIT_FOR_POSTURES && meanY > MEAN_Y_LIMIT_FOR_POSTURE_UPRIGHT_Y_DOWN && meanZ > MEAN_Z_LIMIT_FOR_POSTURES)
		{
			return postureLyingOnLeftSide;
		}
        return postureLying;
    }
    return postureUndefined;
}

uint8_t Get_Short_Term_Activity_Level()
{
	if(maxAbsoluteOverallAcceleration <= 1)
	{
		return activityLevelZero;
	}

	if(maxAbsoluteOverallAcceleration > 1 && maxAbsoluteOverallAcceleration <= 3)
	{
		return activityLevelVeryLow;
	}

	if(maxAbsoluteOverallAcceleration > 3 && maxAbsoluteOverallAcceleration <= 5)
	{
		return activityLevelLow;
	}

	if(maxAbsoluteOverallAcceleration > 5 && maxAbsoluteOverallAcceleration <= 10)
	{
		return activityLevelMedium;
	}

	if(maxAbsoluteOverallAcceleration > 10 && maxAbsoluteOverallAcceleration <= 30)
	{
		return activityLevelHigh;
	}

	// if(maxAbsoluteOverallAcceleration > 30)
	return activityLevelVeryHigh;
}


uint8_t Get_Long_Term_Activity_Level()
{
	float averageMeanAcceleration = 0.0;
	averageMeanAcceleration = sumMeanAbsoluteOverallAcceleration / receivedSignalsCounter;

	if(averageMeanAcceleration <= 0.3)
	{
		return activityLevelZero;
	}

	if(averageMeanAcceleration > 0.3 && averageMeanAcceleration <= 0.6)
	{
		return activityLevelVeryLow;
	}

	if(averageMeanAcceleration > 0.6 && averageMeanAcceleration <= 1)
	{
		return activityLevelLow;
	}

	if(averageMeanAcceleration > 1 && averageMeanAcceleration <= 2)
	{
		return activityLevelMedium;
	}

	if(averageMeanAcceleration > 2 && averageMeanAcceleration <= 7)
	{
		return activityLevelHigh;
	}

	// if(averageMeanAcceleration > 7)
	return activityLevelVeryHigh;
}

void Get_maxAbsoluteOverallAcceleration_And_sumMeanAbsoluteOverallAcceleration(int16_t meanX, int16_t meanY, int16_t meanZ)
{
	float absoluteOverallAcceleration = 0.0;
	float sumAbsoluteOverallAccelerations = 0.0;
//	float vectorX = 0.0;
//	float vectorY = 0.0;
//	float vectorZ = 0.0;
//	float totalVectorLength = 0.0;


	for(uint16_t i = 0; i < ACCELEROMETER_VALUES_PER_SIGNAL; i++)
	{
		// Next line is replaced by following 5 lines
		absoluteOverallAcceleration = sqrt(pow(abs(accX[i] - meanX), 2) + pow(abs(accY[i] - meanY), 2) + pow(abs(accZ[i] - meanZ), 2)) * 9.81 / 1000;
//		vectorX = abs(accX[i] - meanX);
//		vectorY = abs(accY[i] - meanY);
//		vectorZ = abs(accZ[i] - meanZ);
//		totalVectorLength = sqrt(vectorX * vectorX + vectorY * vectorY + vectorZ * vectorZ);
//		absoluteOverallAcceleration = totalVectorLength * 9.81 / 1000;

		if(maxAbsoluteOverallAcceleration < absoluteOverallAcceleration)
		{
			maxAbsoluteOverallAcceleration = absoluteOverallAcceleration;
		}

		sumAbsoluteOverallAccelerations += absoluteOverallAcceleration;

	}

	sumMeanAbsoluteOverallAcceleration += (sumAbsoluteOverallAccelerations / ACCELEROMETER_VALUES_PER_SIGNAL);
}

void Get_Average_Speed()
{
	float averageMeanAcceleration = 0.0;
	float transmissionUnitTime = 0.0;

	averageMeanAcceleration = sumMeanAbsoluteOverallAcceleration / receivedSignalsCounter;
	transmissionUnitTime = receivedSignalsCounter * 256 / 50;
	averageSpeed = (uint32_t) averageMeanAcceleration * transmissionUnitTime;
}

void Save_Data(uint8_t shortTermActivityLevel, uint8_t longTermActivityLevel) {
	uint32_t stepCounterRead = 0;
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef rtcDate;

	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

	printf("\r\n%d:%d:%d:%lu eigen_activities.c accel counter start time\r\n",
			rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds, 1000* (rtcTime.SecondFraction - rtcTime.SubSeconds) /
			(rtcTime.SecondFraction +1));
	printf("%d:%d:%d eigen_activities.c accel counter start date\r\n",
			rtcDate.Year, rtcDate.Month, rtcDate.Date);
	printf("Got datetimeEnd. \r\n\r\n");

	datetimeEnd.Year = rtcDate.Year;
	datetimeEnd.Month = rtcDate.Month;
	datetimeEnd.Date = rtcDate.Date;
	datetimeEnd.Hours = rtcTime.Hours;
	datetimeEnd.Minutes = rtcTime.Minutes;
	datetimeEnd.Seconds = rtcTime.Seconds;

	stepCounterRead = Read_Step_Counter();

	savedData[nextIn][0] = datetimeStart.Year;
	savedData[nextIn][1] = datetimeStart.Month;
	savedData[nextIn][2] = datetimeStart.Date;
	savedData[nextIn][3] = datetimeStart.Hours;
	savedData[nextIn][4] = datetimeStart.Minutes;
	savedData[nextIn][5] = datetimeStart.Seconds;

	savedData[nextIn][6] = datetimeEnd.Year;
	savedData[nextIn][7] = datetimeEnd.Month;
	savedData[nextIn][8] = datetimeEnd.Date;
	savedData[nextIn][9] = datetimeEnd.Hours;
	savedData[nextIn][10] = datetimeEnd.Minutes;
	savedData[nextIn][11] = datetimeEnd.Seconds;

	savedData[nextIn][12] = postureOld;
	savedData[nextIn][13] = shortTermActivityLevel;
	savedData[nextIn][14] = longTermActivityLevel;

	savedData[nextIn][15] = stepCounterRead >> 24;
	savedData[nextIn][16] = stepCounterRead >> 16;
	savedData[nextIn][17] = stepCounterRead >> 8;
	savedData[nextIn][18] = stepCounterRead;

	savedData[nextIn][19] = jumps >> 8;
	savedData[nextIn][20] = jumps;

	savedData[nextIn][21] = runs >> 8;
	savedData[nextIn][22] = runs;

	savedData[nextIn][23] = walkingSteps >> 8;
	savedData[nextIn][24] = walkingSteps;

	savedData[nextIn][25] = situps >> 8;
	savedData[nextIn][26] = situps;

	savedData[nextIn][27] = squats >> 8;
	savedData[nextIn][28] = squats;

	savedData[nextIn][29] = pushups >> 8;
	savedData[nextIn][30] = pushups;

	savedData[nextIn][31] = averageSpeed >> 24;
	savedData[nextIn][32] = averageSpeed >> 16;
	savedData[nextIn][33] = averageSpeed >> 8;
	savedData[nextIn][34] = averageSpeed;

	jumps = 0;
	runs = 0;
	walkingSteps = 0;
	squats = 0;
	situps = 0;
	pushups = 0;
	maxAbsoluteOverallAcceleration = 0.0;
	sumMeanAbsoluteOverallAcceleration = 0.0;
	averageSpeed = 0.0;

// reset step counter
// Clear_Step_Counter_Reg()

	printf("\r\nold nextIn = %d\r\n", nextIn);

	// If memory full, move nextOut to next element. So nextIn will never be equal nextOut.
	// Max number to read is NUMBER_OF_MAX_SAVED_DATA -1, one element is lost.
	if(Get_Number_Saved_Data() >= (NUMBER_OF_MAX_SAVED_DATA - 1))
	{
		Move_NextOut();
	}
	Move_NextIn();
}

uint16_t Get_Number_Saved_Data()
{
	if(nextIn >= nextOut)
	{
		return nextIn - nextOut;
	}
	else
	{
		return NUMBER_OF_MAX_SAVED_DATA - nextOut + nextIn;
	}
}

void Move_NextOut()
{
	if(nextOut < (NUMBER_OF_MAX_SAVED_DATA - 1))
	{
		nextOut ++;
	}
	else
	{
		nextOut = 0;
	}
	printf("\r\nnextOut = %d\r\n", nextOut);
	Update_Number_Of_Activity_Data(Get_Number_Saved_Data());
}

void Move_NextIn()
{
	if(nextIn < (NUMBER_OF_MAX_SAVED_DATA - 1))
	{
		nextIn ++;
	}
	else
	{
		nextIn = 0;
	}
	printf("new nextIn = %d\r\n", nextIn);
	Update_Number_Of_Activity_Data(Get_Number_Saved_Data());
}
