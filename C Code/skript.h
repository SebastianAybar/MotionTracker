
#define MAX_FILE_SIZE 100000 // Puffergröße für Dateiinhalt
#define MAX_VALUES 100000     // Maximale Anzahl an Werten
#define ACCELEROMETER_VALUES_PER_SIGNAL 256 

typedef enum {
	yUndefined,
	yUp,
	yDown
} yAxisOrientations;

typedef enum {
	xyUndefined,
	yUpXForward,		// front left y up
	yUpXBackward,		// front right y up
	yDownXForward,		// front right y down
	yDownXBackward		// front left y down
} xyAxisOrientations;

typedef enum {
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
	False,
	True
} bool;

void Determine_Activities(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ);
uint8_t Get_Y_Axis_Orientation(int16_t meanY);
uint8_t Get_XY_Axis_Orientation(int16_t meanX, int16_t meanY);
uint8_t Get_Posture(int16_t meanX, int16_t meanY, int16_t meanZ, uint8_t xyAxisOrientation);
void Count_Jumps(int16_t accelerationValueY, uint8_t yAxisOrientation);
void Detect_Fall(int16_t accelerationValueX, int16_t accelerationValueY, int16_t accelerationValueZ);
uint16_t Count_Stairs(int16_t meanX, int16_t meanY, int16_t meanZ, uint8_t yAxisOrientation);



