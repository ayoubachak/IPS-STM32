//?????#include "hal.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

#include "stm32xxx_hal.h"
#include <string.h>

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
#define VL53L0X_OsDelay(...) HAL_Delay(2)


#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif
//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
#ifndef VL53L0X_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_GetI2cBus(...) (void)0
#endif

#ifndef VL53L0X_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_PutI2cBus(...) (void)0
#endif

#ifndef VL53L0X_OsDelay
#   define  VL53L0X_OsDelay(...) (void)0
#endif


uint8_t _I2CBuffer[64];

int _I2CWrite(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Transmit(Dev->I2cHandle, Dev->I2cDevAddr, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

int _I2CRead(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE+ count* I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Receive(Dev->I2cHandle, Dev->I2cDevAddr|1, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index;
    memcpy(&_I2CBuffer[1], pdata, count);
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, count + 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, pdata, count);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    _I2CBuffer[1] = data;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    _I2CBuffer[1] = data >> 8;
    _I2CBuffer[2] = data & 0x00FF;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 3);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index;
    _I2CBuffer[1] = (data >> 24) & 0xFF;
    _I2CBuffer[2] = (data >> 16) & 0xFF;
    _I2CBuffer[3] = (data >> 8)  & 0xFF;
    _I2CBuffer[4] = (data >> 0 ) & 0xFF;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 5);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L0X_WrByte(Dev, index, data);
done:
    return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);
    if( status_int ){
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, data, 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);

    if( status_int ){
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];

done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // do nothing
    VL53L0X_OsDelay();
    return status;
}




/// AJOUT JMB



/**
 * Handle Error
 *
 * Set err on display and loop forever
 * @param err Error case code
 */
void HandleError(int err){
    char msg[16];
    sprintf(msg,"JMEr%d", err);
    F411_SetDisplayString(msg);
    while(1){};
}

/**
 * Reset all sensor then do presence detection
 *
 * All present devices are data initiated and assigned to their final I2C address
 * @return
 */

int nDevPresent=0;
I2C_HandleTypeDef XNUCLEO53L0A1_hi2c;

/**
 * Global ranging struct
 */
VL53L0X_RangingMeasurementData_t RangingMeasurementData;


/** leaky factor for filtered range
 *
 * r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)
 *
 * */
int LeakyFactorFix8 = (int)( 0.6 *256);
/** How many device detect set by @a DetectSensors()*/

/** bit is index in VL53L0XDevs that is not necessary the dev id of the BSP */
int nDevMask;


char *RangingConfigTxt[3] = {"LR", "HS", "HA"};
char *DemoModeTxt[2] = {"rng", "bar"};

/** range low (and high) in @a RangeToLetter()
 *
 * used for displaying  multiple sensor as bar graph
 */
int RangeLow=100;

/** range medium in @a RangeToLetter()
 *
 * used for displaying  multiple sensor as bar graph
 */
int RangeMedium=300;



VL53L0X_Dev_t VL53L0XDevs[]={
        {.Id=XNUCLEO53L0A1_DEV_CENTER, .DevLetter='c', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
    //    {.Id=XNUCLEO53L0A1_DEV_LEFT, .DevLetter='l', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
    //    {.Id=XNUCLEO53L0A1_DEV_RIGHT, .DevLetter='r', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
};








int VL53L0X_ResetId(int DevNo, int state) {
    int status=0;
/// AFAIRE RESET


    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
    HAL_Delay(100);
    if( state )
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);

    return status;
}

extern UART_HandleTypeDef huart2;

int F411_SetDisplayString(const char *str) {

	int status=1;
	 HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 0xFFFF);

    return status;
}





int DetectSensors(int SetDisplay) {
    int i;
    uint16_t Id;
    int status;
    int FinalAddress;

    debug_printf("Detect Sensor");
    /* Reset all */
    nDevPresent = 0;


        status = VL53L0X_ResetId(0, 0);


i =0;
    /* detect all sensors (even on-board)*/
  //  for (i = 0; i < 1; i++) {
        VL53L0X_Dev_t *pDev;
        pDev = &VL53L0XDevs[i];
        pDev->I2cDevAddr = 0x52;
        pDev->Present = 0;
        status = 0;  // // FAIRE FONCTION RESET
        VL53L0X_ResetId( pDev->Id, 1);        HAL_Delay(2);
        FinalAddress=0x52+(i+1+1)*2;

        do {
        	/* Set I2C standard mode (400 KHz) before doing the first register access */
        	if (status == VL53L0X_ERROR_NONE)
        		status = VL53L0X_WrByte(pDev, 0x88, 0x00);

        	/* Try to read one register using default 0x52 address */
            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
            if (status) {
                debug_printf("#%d Read id fail\n", i);
                break;
            }
            if (Id == 0xEEAA) {
				/* Sensor is found => Change its I2C address to final one */
                status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
                if (status != 0) {
                    debug_printf("#i VL53L0X_SetDeviceAddress fail\n", i);
                    break;
                }
                pDev->I2cDevAddr = FinalAddress;
                /* Check all is OK with the new I2C address and initialize the sensor */
                status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
                if (status != 0) {
					debug_printf("#i VL53L0X_RdWord fail\n", i);
					break;
				}

                status = VL53L0X_DataInit(pDev);
                if( status == 0 ){
                    pDev->Present = 1;
                }
                else{
                    debug_printf("VL53L0X_DataInit %d fail\n", i);
                    break;
                }
                trace_printf("VL53L0X %d Present and initiated to final 0x%x\n", pDev->Id, pDev->I2cDevAddr);
                nDevPresent++;
                nDevMask |= 1 << (i+1);
                pDev->Present = 1;
            }
            else {
                debug_printf("#%d unknown ID %x\n", i, Id);
                status = 1;
            }
        } while (0);
        /* if fail r can't use for any reason then put the  device back to reset */
        if (status) {
            VL53L0X_ResetId(i+1, 0);
        }
    //}
    return nDevPresent;
}

/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
void SetupSingleShot(RangingConfig_e rangingConfig){
    int i;
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

 //   for( i=0; i<3; i++){
	i=0;
        if( VL53L0XDevs[i].Present){
            status=VL53L0X_StaticInit(&VL53L0XDevs[i]);
            if( status ){
                debug_printf("VL53L0X_StaticInit %d failed\n",i);
            }

            status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

			status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i], &refSpadCount, &isApertureSpads);
			if( status ){
			   debug_printf("VL53L0X_PerformRefSpadManagement failed\n");
			}

            status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i], VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
            if( status ){
               debug_printf("VL53L0X_SetDeviceMode failed\n");
            }

            status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}

			status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}
			/* Ranging configuration */
            switch(rangingConfig) {
            case LONG_RANGE:
            	signalLimit = (FixPoint1616_t)(0.1*65536);
            	sigmaLimit = (FixPoint1616_t)(60*65536);
            	timingBudget = 33000;
            	preRangeVcselPeriod = 18;
            	finalRangeVcselPeriod = 14;
            	break;
            case HIGH_ACCURACY:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(18*65536);
				timingBudget = 200000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            case HIGH_SPEED:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(32*65536);
				timingBudget = 20000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            default:
            	debug_printf("Not Supported");
            }

            status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

			status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

            status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i],  timingBudget);
            if( status ){
               debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
            }

            status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
			if( status ){
			   debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

            status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
			if( status ){
			   debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

			status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings, &PhaseCal);
			if( status ){
			   debug_printf("VL53L0X_PerformRefCalibration  failed\n");
			}

            VL53L0XDevs[i].LeakyFirst=1;
        }

}



/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
    if( pRange->RangeStatus == 0 ){
        if( pDev->LeakyFirst ){
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }
        else{
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }
    else{
        pDev->LeakyFirst = 1;
    }
}





void ResetAndDetectSensor(int SetDisplay){
    int nSensor;
    nSensor = DetectSensors(SetDisplay);
    /* at least one sensor and if one it must be the built-in one  */
    if( (nSensor <=0) ||  (nSensor ==1 && VL53L0XDevs[0].Present==0) ){
        HandleError(ERR_DETECT);
    }
}
//end of file
