
#include "main.h"
#include "BMI323_eigen.h"
#include "eigen_ble.h"
#include "custom_stm.h"
#include "eigen_activities.h"

extern I2C_HandleTypeDef hi2c3;
uint8_t sensorIsInitialised=0;

uint8_t dataArrayForBluetooth[240]={0};
uint32_t dataArrayCounter=0;
int64_t stepCounterTare=0; // has to be so big to be able to store the result of a subtraction of two uint32_t variables

void Sensor_Init()
{
	// SDO has to be pulled to GND to enable default DEVICE_ADDRESS
	// the following steps are described in the datasheet
	printf("BMI323_eigen.c sensor init\r\n");

	First_Read_Sensor_Id();

	Check_Error_Reg();
	Check_Status_Reg();

	Enable_Feature_Engine(); // has to be done before acc and gyr init

	Configure_Fifo(); // FIFO has to be activated before accelerometer is activated or before low power mode is turned on
	Configure_Interrupt_Fifo_Watermark();
	Configure_Accelerometer(dataRate50, lowPower);

	Enable_Step_Counter();

//	Configure_Gyroscope(dataRate100, highPerformance);

	Check_Error_Reg();
	Check_Status_Reg();

	sensorIsInitialised=1;
}

void First_Read_Sensor_Id()
{
	uint8_t receivedData[4]={0};
	HAL_StatusTypeDef ret=0;
	// the ID has to be read to "activate" the sensor the first time after power up

	printf("BMI323_eigen.c reading chip id\r\n");
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, CHIP_ID_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{

		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c chip id is: %u\r\n", receivedData[2]); // nur das LSB enthält die Addresse, oberes Byte ist reserved
	}
}

void Configure_Accelerometer(uint8_t dataRate, uint8_t powerMode)
{
	// sets the accelerometer parameters
	HAL_StatusTypeDef ret=0;
	uint8_t transmitData[2]={0};

	// configure dataRate
	switch(dataRate)
	{
		case dataRate25:
			transmitData[0] |= 0b0110;
		break;

		case dataRate50:
			transmitData[0] |= 0b0111;
		break;

		case dataRate100:
			transmitData[0] |= 0b1000;

		break;

		case dataRate200:
			transmitData[0] |= 0b1001;
		break;
	}

	// configure scale of the accelerometer
	transmitData[0] |= (0b011 << 4); // 16 g range

	// configure power mode
	switch(powerMode)
	{
		case lowPower:
			transmitData[1] |= (0b011 <<4);
		break;

		case highPerformance:
			transmitData[1] |= (0b111 <<4);
	}

	// write data to the accelerometer register
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, ACC_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c writing data to accelerometer configuration\r\n");
	}
}

void Configure_Gyroscope(uint8_t dataRate, uint8_t powerMode)
{
	HAL_StatusTypeDef ret=0;
	uint8_t transmitData[2]={0};
	// sets the gyroscope parameters
	// configure dataRate
	switch(dataRate)
	{
		case dataRate25:
			transmitData[0] |= 0b0110;
		break;

		case dataRate50:
			transmitData[0] |= 0b0111;
		break;

		case dataRate100:
			transmitData[0] |= 0b1000;

		break;

		case dataRate200:
			transmitData[0] |= 0b1001;
		break;
	}

	// configure scale of the accelerometer
	transmitData[0] |= (0b011 << 4); // +-2000 °/s

	// configure power mode
	switch(powerMode)
	{
		case lowPower:
			transmitData[1] |= (0b011 <<4);
		break;

		case highPerformance:
			transmitData[1] |= (0b111 <<4);
	}

	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, GYR_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c writing data to gyroscope configuration\r\n");
	}


}

	// the FIFO fill level should be the value specified for the water level interrupt during initialization of the sensor
	// however, if there is a delay between the last measurement value and the execution of this function, there MIGHT be
	// a different number stored in the FIFO
	// just to be sure, the exact fill level of the FIFO is read and subsequently this number of data packets is read

void Read_Sensor_Data()
{
	uint8_t receivedData[10]={0};
	HAL_StatusTypeDef ret=0;
	int16_t accelerationValueX=0;
	int16_t accelerationValueY=0;
	int16_t accelerationValueZ=0;
	uint16_t fifoFillLevel = 0;
	uint16_t loopCounter = 0;
	uint16_t sensorTime = 0;

	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, SENSOR_FIFO_FILL_LEVEL_READ, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		fifoFillLevel = (receivedData[3] << 8) | receivedData[2]; // reads the current fill level (amount of data words) in the FIFO
	}
	loopCounter = 0;
	while(loopCounter < fifoFillLevel/4) // during each read, 4 words are read
	{
		ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, SENSOR_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, receivedData, 10, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			printf("BMI323_eigen.c error in communication I2C\r\n");
		} else
		{	// I do not know why sensorTime is max 255, it should be a 16 bit word acc. to datasheet
			sensorTime = (receivedData[8] << 8) | receivedData[9]; 
			accelerationValueZ = (receivedData[7] << 8) | receivedData[6];
			accelerationValueZ = accelerationValueZ/2; // actually /2,05 acc. to data sheet but dividing by two is faster

			accelerationValueY = (receivedData[5] << 8) | receivedData[4];
			accelerationValueY = accelerationValueY/2;

			accelerationValueX = (receivedData[3] << 8) | receivedData[2];
			accelerationValueX = accelerationValueX/2;

			Determine_Activities(accelerationValueX, accelerationValueY, accelerationValueZ);

			//Methode für eigene Characteristic
			Determine_RawData(accelerationValueX, accelerationValueY, accelerationValueZ);
		}
		loopCounter++;
	}
	
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, SENSOR_FIFO_FILL_LEVEL_READ, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		fifoFillLevel = (receivedData[3] << 8) | receivedData[2];	// two 8 bit to 16 bit
//		printf("BMI323_eigen.c FIFO fill level is: %u (should now be 0)\r\n", fifoFillLevel);
		// if the execution of this loop takes too long, additional data values will be added to the FIFO
		// this will result in the FIFO fill level after the loop not being equal to 0
	}
	printf("BMI323_eigen.c processed data from FIFO, step counter: %lu\r\n", Read_Step_Counter());
//	printf("BMI323_eigen.c end read sensor data\r\n");
}

uint32_t Read_Step_Counter(void)
{
//	printf("Read_Step_Counter\r\n");
	// this function reads and returns the current step counter value
	uint8_t receivedData[4]={0};
	uint16_t stepCounterValueLow=0;
	uint16_t stepCounterValueHigh=0;
	uint32_t stepCounterValue=0;
	HAL_StatusTypeDef ret=0;
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_2_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		stepCounterValueLow = (receivedData[3] << 8) | receivedData[2];	// contains the lower 16 bit of step counter
//		printf("BMI323_eigen.c lower step counter value is: %u\r\n", stepCounterValueLow);
	}
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_3_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		stepCounterValueHigh = (receivedData[3] << 8) | receivedData[2];	// contains the higher 16 bit of step counter
//		printf("BMI323_eigen.c higher step counter value is: %u\r\n", stepCounterValueHigh);
		stepCounterValue = (stepCounterValueHigh << 16) | stepCounterValueLow;
//		printf("BMI323_eigen.c final step counter value is: %lu\r\n", stepCounterValue);
	}
	stepCounterValue = stepCounterValue - stepCounterTare;
	if(stepCounterValue < 0)
	{
		printf("BMI323_eigen.c there was an overflow of the step counter register\r\n");
		stepCounterValue = stepCounterValue + 2^32; // step counter is always a positive value
													// if the step counter reg overflows, it starts at zero again
													// so if the result of the subtraction is negative, you can be sure that there was an overflow
													// and 2^32 has to be added to the result
	}
	return stepCounterValue;
}

void Clear_Step_Counter_Reg()
{
	printf("BMI323_eigen.c clear step counter reg\r\n");

	// this function sets a tare variable that will be added to the step counter in order to make the result 0

	uint8_t receivedData[4]={0};
	uint16_t stepCounterValueLow=0;
	uint16_t stepCounterValueHigh=0;
	uint32_t stepCounterValue=0;
	HAL_StatusTypeDef ret=0;
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_2_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		stepCounterValueLow = (receivedData[3] << 8) | receivedData[2];	// contains the lower 16 bit of step counter
//		printf("BMI323_eigen.c lower step counter value is: %u\r\n", stepCounterValueLow);
	}
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_3_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		stepCounterValueHigh = (receivedData[3] << 8) | receivedData[2];	// contains the higher 16 bit of step counter
//		printf("BMI323_eigen.c higher step counter value is: %u\r\n", stepCounterValueHigh);
		stepCounterValue = (stepCounterValueHigh << 16) | stepCounterValueLow;
//		printf("BMI323_eigen.c final step counter value is: %lu\r\n", stepCounterValue);
	}

	stepCounterTare = stepCounterValue; // sets a tare value, so the step counter can be set to 0 without changing the register of the sensor




	// ich habe das nicht zum Laufen bekommen! Im Datasheet gibt es lediglich einen Absatz zu dem Thema (Extended Memory)
	// Nichts von dem hat funktioniert. Möchte sich jemand daran versuchen?
	// this function accesses the extended register map (see datasheet!)
//	HAL_StatusTypeDef ret=0;
//	uint16_t temp = 0;
//	uint8_t transmitData[2]={};
//	uint8_t receivedData[2]={0};
//
//
//	// reading register to get value of "reserved" bits
//	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_DATA_ADDR, I2C_MEMADD_SIZE_8BIT, receivedData, 2, HAL_MAX_DELAY);
//	if(ret != HAL_OK)
//	{
//		printf("BMI323_eigen.c error in communication I2C\r\n");
//	}
//	printf("data in the register: %u - %u\r\n", receivedData[1], receivedData[0]);
//
//	// keeping the value of the "reserved" bits and adding the new configuration of non reserved bits
//	// writing the target address of the extended register map
//	temp = receivedData[1]<<8 | receivedData[0];
//	temp = temp | EXTENDED_REGISTER_SC1;
//
//	printf("temp: %u", temp);
//
//	transmitData[0]= temp; // write target address
//	transmitData[1]= temp >> 8;
//	printf("sending data: %u - %u\r\n", transmitData[1], transmitData[0]);
//	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, FEATURE_DATA_ADDR, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
//	if(ret != HAL_OK)
//	{
//		printf("BMI323_eigen.c error in communication I2C\r\n");
//	}
//
////	uint8_t dataTxReady = 0;
////	while (dataTxReady == 0)
////	{
////		printf("polling data tx rdy\r\n");
////		ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_DATA_STATUS, I2C_MEMADD_SIZE_8BIT, receivedData, 2, HAL_MAX_DELAY);
////		if(ret != HAL_OK)
////		{
////			printf("BMI323_eigen.c error in communication I2C\r\n");
////		}
////		printf("receivedData: %u - %u\r\n", receivedData[1], receivedData[0]);
////		dataTxReady = receivedData[0] & 0b00000010;
////	}
//
//	// write data to register when data tx is ready
//	transmitData[0]= 1; // writing a 1 to that register resets the step counter register
//	transmitData[1]=0;
//	printf("sending data: %u - %u\r\n", transmitData[1], transmitData[0]);
//	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, FEATURE_DATA_TX, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
//	if(ret != HAL_OK)
//	{
//		printf("BMI323_eigen.c error in communication I2C\r\n");
//	}
}

void Configure_Interrupt_Fifo_Watermark()
{
	HAL_StatusTypeDef ret=0;
	uint8_t transmitData[2]={};
	//enable interrupt on pin INT2
	printf("BMI323_eigen.c Configure_Interrupt_Fifo_Watermark\r\n");

	// FIFO hast a size of 2048 Bytes, that is 1024 2-Byte-Words
	uint16_t newFifoWatermarkLevel = 900; // number of words for the FIFO water mark
	transmitData[0]= newFifoWatermarkLevel & 0b11111111; // set FIFO watermark level. Interrupt will be generated if FIFO is filled with more words than specified here
	transmitData[1]= newFifoWatermarkLevel >> 8;
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, SENSOR_FIFO_WATERMARK_SET, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c configuring FIFO watermark level\r\n");
	}

	transmitData[0]=0b00000000;
	transmitData[1]=0b00100000; // map FIFO watermark interrupt to INT2 (pin)
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, INT_MAP2_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c connecting watermark interrupt to INT2\r\n");
	}

	// for debugging purposes, step detector is routed to pin of INT1
	transmitData[0]=0b00000000;
	transmitData[1]=0b00000001; // step detector interrupt mapped to INT1 (pin)
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, INT_MAP1_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c writing data to INT1\r\n");
	}


	printf("BMI323_eigen.c enabling INT2 output\r\n");

	transmitData[0]=0b00000100; // INT1 is enabled for debugging. Without enabling: 0b00000000
	transmitData[1]=0b00000100; // INT2 enable, push-pull, active low
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, IO_INT_CTRL_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c enable INT2\r\n");
	}
}

void Configure_Fifo()
{
	HAL_StatusTypeDef ret=0;
	uint8_t transmitData[2]={};

	transmitData[0]=0b00000001; // stop writing to FIFO when full
	transmitData[1]=0b00000011; // store accelerometer and sensor time
	// to save FIFO space (and improve current consumption) sensor time does not need to be stored in FIFO
	// but will be used for now for error checking and debugging purposes (to see if data gets lost)
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, SENSOR_FIFO_CONF, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c configuring FIFO\r\n");
	}
}

void Check_Error_Reg()
{
	// checking the error reg (for debugging purposes)
	uint8_t receivedData[4]={0};
	HAL_StatusTypeDef ret=0;
	printf("BMI323_eigen.c reading error reg id\r\n");
		ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, ERR_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			printf("BMI323_eigen.c error in communication I2C\r\n");
		} else
		{
			if((receivedData[2] & 0b00000001) != 0)
			{
				printf("### BMI323_eigen.c fatal sensor error\r\n");
			}
			if((receivedData[2] & 0b00000100) != 0)
			{
				printf("### BMI323_eigen.c overload of feature engine detected\r\n");
			}
			if((receivedData[2] & 0b00010000) != 0)
			{
				printf("### BMI323_eigen.c watchdog of feature engine triggered\r\n");
			}
			if((receivedData[2] & 0b00100000) != 0)
			{
				printf("### BMI323_eigen.c unsupported accelerometer configuration\r\n");
			}
			if((receivedData[2] & 0b01000000) != 0)
			{
				printf("### BMI323_eigen.c unsupported gyroscope configuration\r\n");
			}
			printf("BMI323_eigen.c data received: %u\r\n", receivedData[2]); // nur das untere Byte enthält relevante Informationen für I2C
		}
}

void Enable_Step_Counter()
{
	// this function enables the step counter
	uint8_t receivedData[4]={0};
	HAL_StatusTypeDef ret=0;
	uint8_t transmitData[2]={};

	printf("BMI323_eigen.c Enable_Step_Counter\r\n");
	// step detector: generates an interrupt on INT pin each time a step is detected
	// step counter: counts the steps in the step counter register
	// no step counter interrupt is needed

	// activate step counter
	transmitData[0]=0b00000000; // step counter activated
	transmitData[1]=0b00000011; // for debugging, also step detector is activated, 0b00000010 for only step counter
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_0_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c writing new data to FEATURE_IO_0\r\n");
	}

	// writing a 1 to feature io status reg sends the changes to the feature engine
	transmitData[0]=0b00000001;
	transmitData[1]=0b00000000;
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_STATUS_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c saving changes to FEATURE_IO_0\r\n");
	}


	// check if everything has been written correctly
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_0_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c FEATURE_IO_0 data received: %u, %u\r\n", receivedData[2],receivedData[3]);
	}

	Check_Error_Reg(); // optional, to check for errors
	Check_Status_Reg(); // optional, to check for errors
}

void Enable_Feature_Engine()
{
	// this function has to be called before any acc or gyr is initialized
	// it enables the feature engine that is required for the step counter and things like motion detection
	printf("BMI323_eigen.c enabling the feature engine\r\n");

	uint8_t receivedData[4]={0};
	HAL_StatusTypeDef ret=0;
	uint8_t transmitData[2]={};

	// to enable the feature engine, 0x012C has to be written to FEATURE_IO_2_REG before acc and gyr are initialized
	transmitData[0]=0x2C;
	transmitData[1]=0x01;
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_2_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c writing data to FEATURE_IO_2\r\n");
	}

	// if the following read operation returns a 1, the feature engine has been written to successfully
	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_STATUS_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c FEATURE_IO_2 data received: %u, %u\r\n", receivedData[2],receivedData[3]);
		if(receivedData[3] == 0 && receivedData[2]==1)
		{
			printf("BMI323_eigen.c feature engine ok\r\n");
		}
	}

	// now the feature control engine has to be set to 1
	transmitData[0]=1;
	transmitData[1]=0;
	ret = HAL_I2C_Mem_Write(&hi2c3, DEVICE_ADDRESS, FEATURE_CTRL_REG, I2C_MEMADD_SIZE_8BIT, transmitData, 2, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		printf("BMI323_eigen.c error in communication I2C\r\n");
	} else
	{
		printf("BMI323_eigen.c writing data to feature ctrl reg\r\n");
	}

	uint8_t exitCondition=0;
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1); // if the blue light is continuously turned on after the device started up, there is an error

	// according to datasheet, this register has to be read until it returns the value 1
	printf("BMI323_eigen.c polling feature io reg until it returns 1\r\n");
	while(exitCondition==0)
	{
		ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, FEATURE_IO_1_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			printf("BMI323_eigen.c error in communication I2C\r\n");
		} else
		{
			printf("BMI323_eigen.c FEATURE_IO_1 data received: %u, %u\r\n", receivedData[2],receivedData[3]);
			if(receivedData[2] == 1)
			{
				printf("BMI323_eigen.c feature engine started\r\n");
				exitCondition=1;
			}
		}
	}
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);


}

void Check_Status_Reg()
{
	// checking the status reg (for debugging purposes)
	uint8_t receivedData[4]={0};
	HAL_StatusTypeDef ret=0;
	printf("BMI323_eigen.c reading status reg\r\n");
		ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, STATUS_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
		if(ret != HAL_OK)
		{
			printf("BMI323_eigen.c error in communication I2C\r\n");
		} else
		{
			if((receivedData[2] & 0b00000001) != 0)
			{
				printf("BMI323_eigen.c power on reset detected\r\n");
			}
			if((receivedData[2] & 0b00100000) != 0)
			{
				printf("BMI323_eigen.c data ready for temperature\r\n");
			}
			if((receivedData[2] & 0b01000000) != 0)
			{
				printf("BMI323_eigen.c data ready for gyroscope\r\n");
			}
			if((receivedData[2] & 0b10000000) != 0)
			{
				printf("BMI323_eigen.c data ready for accelerometer\r\n");
			}

			printf("BMI323_eigen.c data received: %u\r\n", receivedData[2]); // nur das untere Byte enthält relevante Informationen für I2C
		}
}

//void Clear_Interrupt()
//{
//	uint8_t receivedData[4]={0};
//	HAL_StatusTypeDef ret=0;
//
//	ret = HAL_I2C_Mem_Read(&hi2c3, DEVICE_ADDRESS, ACC_DATA_X_REG, I2C_MEMADD_SIZE_8BIT, receivedData, 4, HAL_MAX_DELAY);
//	if(ret != HAL_OK)
//	{
//		printf("BMI323_eigen.c error in communication I2C\r\n");
//	} else
//	{
//		printf("BMI323_eigen.c data received: %u, %u\r\n", receivedData[2],receivedData[3]);
//	}
//}

