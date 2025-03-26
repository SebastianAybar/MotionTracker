

#define DEVICE_ADDRESS ((0x68) << 1)
#define CHIP_ID_REG 0x00
#define ERR_REG 0x01
#define STATUS_REG 0x02
#define ACC_CONFIG_REG 0x20
#define GYR_CONFIG_REG 0x21
#define ACC_DATA_X_REG 0x03
#define ACC_DATA_Y_REG 0x04
#define ACC_DATA_Z_REG 0x05
#define GYR_DATA_X_REG 0x06
#define GYR_DATA_Y_REG 0x07
#define GYR_DATA_Z_REG 0x08
#define INT_MAP1_REG 0x3A
#define INT_MAP2_REG 0x3B
#define IO_INT_CTRL_REG 0x38
#define SENSOR_TIME_0_REG 0x0A
#define SENSOR_TIME_1_REG 0x0B
#define FEATURE_IO_0_REG 0x10
#define FEATURE_IO_1_REG 0x11
#define FEATURE_IO_2_REG 0x12
#define FEATURE_IO_3_REG 0x13
#define FEATURE_IO_STATUS_REG 0x14
#define FEATURE_CTRL_REG 0x40
#define SENSOR_TIME_0_REG 0x0A
#define SENSOR_TIME_1_REG 0x0B
#define SENSOR_FIFO_CONF 0x36
#define SENSOR_FIFO_FILL_LEVEL_READ 0x15
#define SENSOR_FIFO_DATA 0x16
#define SENSOR_FIFO_WATERMARK_SET 0x35
#define FEATURE_DATA_TX 0x42
#define FEATURE_DATA_ADDR 0x41
#define FEATURE_DATA_STATUS 0x43

// the following defines are registers inside the extended register map and have to be addressed via FEATURE_DATA_ADD (see datasheet)
#define EXTENDED_REGISTER_SC1 0x10



typedef enum
{
	dataRate25,
	dataRate50,
	dataRate100,
	dataRate200,

} bmiDataRate;

typedef enum
{
	lowPower,
	highPerformance

} bmiPowerMode;

void Sensor_Init();
void Check_Error_Reg();
void Check_Status_Reg();
//void Clear_Interrupt();
void Configure_Interrupt_Fifo_Watermark();
void Read_Sensor_Data();
void First_Read_Sensor_Id();
void Configure_Accelerometer(uint8_t dataRate, uint8_t powerMode);
void Configure_Gyroscope(uint8_t dataRate, uint8_t powerMode);
void Collect_Data_For_Ble(int32_t *dataToCollect);
void Enable_Step_Counter();
void Enable_Feature_Engine();
void Configure_Fifo();
uint32_t Read_Step_Counter();
void Clear_Step_Counter_Reg();
