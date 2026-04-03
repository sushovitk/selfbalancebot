/* vl53l0x.c — VL53L0X time-of-flight distance sensor driver module
 *
 * This file contains only VL53L0X-specific initialisation and measurement
 * helper functions.  All HAL boilerplate (main, SystemClock_Config,
 * Error_Handler, __io_putchar) and peripheral handle definitions
 * (hi2c1, hlpuart1) live exclusively in main.c.
 */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

/* USER CODE BEGIN PD */
#define VL53L0X_ADDR 0x52
/* USER CODE END PD */

/*
 * hi2c1, hlpuart1, vl53_dev, and Dev are defined in main.c.
 * Reference them here with extern — do NOT redefine them.
 */
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef hlpuart1;
extern VL53L0X_Dev_t      vl53_dev;
extern VL53L0X_DEV        Dev;

/* USER CODE BEGIN 0 */

/**
 * @brief  Initialise the VL53L0X sensor (DataInit, StaticInit,
 *         SPAD management, reference calibration, device mode).
 * @retval VL53L0X_ERROR_NONE on success, error code otherwise.
 */
VL53L0X_Error VL53L0X_SensorInit(void)
{
    VL53L0X_Error status;
    uint32_t refSpadCount   = 0;
    uint8_t  isApertureSpads = 0;
    uint8_t  VhvSettings    = 0;
    uint8_t  PhaseCal       = 0;

    /* Allow sensor to boot */
    HAL_Delay(10);

    memset(&vl53_dev, 0, sizeof(vl53_dev));
    Dev->I2cDevAddr      = VL53L0X_ADDR;
    Dev->comms_type      = 1;
    Dev->comms_speed_khz = 100;

    status = VL53L0X_DataInit(Dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        printf("VL53L0X DataInit failed: %d\r\n", status);
        return status;
    }

    status = VL53L0X_StaticInit(Dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        printf("VL53L0X StaticInit failed: %d\r\n", status);
        return status;
    }

    status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    if (status != VL53L0X_ERROR_NONE)
    {
        printf("VL53L0X RefSpadManagement failed: %d\r\n", status);
        return status;
    }

    status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE)
    {
        printf("VL53L0X RefCalibration failed: %d\r\n", status);
        return status;
    }

    status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
    {
        printf("VL53L0X SetDeviceMode failed: %d\r\n", status);
        return status;
    }

    printf("VL53L0X ready\r\n");
    return VL53L0X_ERROR_NONE;
}

/**
 * @brief  Perform one single-ranging measurement.
 * @param  pRange  Output: range in millimetres (valid only on success).
 * @retval VL53L0X_ERROR_NONE on success, error code otherwise.
 */
VL53L0X_Error VL53L0X_ReadDistance(uint16_t *pRange)
{
    VL53L0X_RangingMeasurementData_t RangingData;
    VL53L0X_Error status;

    status = VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
    if (status == VL53L0X_ERROR_NONE)
    {
        *pRange = RangingData.RangeMilliMeter;
    }
    return status;
}

/* USER CODE END 0 */
