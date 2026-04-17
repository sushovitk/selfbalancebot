/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "BNO055_STM32.h"
#include "calib_flash.h"
#include "pixy2.h"
#include "ps2.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L0X_ADDR 0x52

/* ── Pixy2 tracking configuration ─────────────────────────────────────────
 * PIXY_TRACK_SIG     : Pixy2 signature to chase (1–7, trained in PixyMon).
 * PIXY_FRAME_CX      : Horizontal centre of the Pixy2 frame (316 px wide).
 * PIXY_STOP_WIDTH    : Block pixel-width at which we consider the object
 *                      "close enough" and stop driving toward it.
 *                      Tune by placing the robot at your desired stop
 *                      distance and reading pixy.blocks[0].width.
 * PIXY_MIN_WIDTH     : Ignore detections narrower than this — too noisy.
 * DRIVE_LEAN_DEG     : Setpoint offset (degrees) applied while chasing.
 *                      Positive = lean forward → PID drives the robot forward.
 *                      Tune together with Kp: too large → overshoot.
 * STEER_GAIN         : Converts lateral pixel error to differential PWM.
 *                      (lateral_err_px) * STEER_GAIN = steer_pwm_delta.
 *                      Start small (~0.3) and increase until turns are crisp.
 * PIXY_LOST_FRAMES   : Consecutive empty frames before tracking is cancelled
 *                      and the setpoint returns to 0°.
 * PIXY_POLL_DIVIDER  : Poll Pixy2 every Nth PID cycle to avoid SPI overhead
 *                      on every IMU callback. 2 → ~50 Hz at 100 Hz PID rate.
 * ─────────────────────────────────────────────────────────────────────────*/
#define PIXY_TRACK_SIG       1
#define PIXY_FRAME_CX        158
#define PIXY_STOP_WIDTH      120
#define PIXY_MIN_WIDTH       15
#define DRIVE_LEAN_DEG       2.5f
#define STEER_GAIN           0.3f
#define PIXY_LOST_FRAMES     10
#define PIXY_POLL_DIVIDER    2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c2_rx;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
VL53L0X_Dev_t vl53_dev;
VL53L0X_DEV Dev = &vl53_dev;
uint8_t OffsetDatas[22];
BNO055_Sensors_t BNO055;
uint8_t imu_raw_data[24];              // DMA dumps the 24 bytes here
volatile bool imu_data_ready = false; // Flag to tell the main loop data is fresh

#define PIXY2_SEND_SYNC_BYTE  0xae
#define PIXY2_SEND_SYNC_BYTE2 0xc1
#define PIXY2_RECV_SYNC_BYTE  0xaf
#define PIXY2_RECV_SYNC_BYTE2 0xc1

Pixy2 pixy;

PIDController pid;
uint32_t last_pid_tick = 0;

/* ── Pixy2 tracking state ──────────────────────────────────────────────── */
/* steer_output  : signed differential PWM applied between Motor A and B.
 *                 +ve steers toward the right, -ve toward the left.
 *                 Reset to 0 when no object is tracked.
 * pixy_lost_cnt : counts consecutive PID cycles with no valid detection.
 *                 When it reaches PIXY_LOST_FRAMES the robot stops chasing.
 * pixy_poll_ctr : counts PID cycles; Pixy2 is queried every PIXY_POLL_DIVIDER. */
static float    steer_output   = 0.0f;
static uint8_t  pixy_lost_cnt  = 0;
static uint8_t  pixy_poll_ctr  = 0;
static bool     pixy_enabled   = false;   /* set true after successful init */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ============ MOTOR AND HBRIDGE USER CODE 0 ============
/**
 * @brief  Set speed and direction for Motor A (ENA=PA0, IN1=PD4, IN2=PD5)
 * @param  duty     0–1000
 * @param  forward  1 = forward, 0 = reverse
 */
void MotorA_Set(uint16_t duty, uint8_t forward)
{
    if (duty > 1000) duty = 1000;

    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin,
                      forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin,
                      forward ? GPIO_PIN_RESET : GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}

/**
 * @brief  Set speed and direction for Motor B (ENB=PA1, IN3=PD6, IN4=PD7)
 * @param  duty     0–1000
 * @param  forward  1 = forward, 0 = reverse
 */
void MotorB_Set(uint16_t duty, uint8_t forward)
{
    if (duty > 1000) duty = 1000;

    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin,
                      forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin,
                      forward ? GPIO_PIN_RESET : GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty);
}

void MotorA_Brake(void)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
}

void MotorB_Brake(void)
{
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
}

void MotorA_Coast(void)
{
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
}

void MotorB_Coast(void)
{
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}



static inline void Pixy2_CS_Select(void) {
	HAL_GPIO_WritePin(PIXY_CS_GPIO_Port, PIXY_CS_Pin, GPIO_PIN_RESET);
}
static inline void Pixy2_CS_Deselect(void) {
	HAL_GPIO_WritePin(PIXY_CS_GPIO_Port, PIXY_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Apply PID output + lateral steering differential to both motors.
 *
 * @param  output  Signed PID output. Positive = forward, negative = backward.
 *                 Magnitude maps to PWM duty (0–PWM_MAX).
 * @param  steer   Signed steering bias (PWM counts). Applied as +steer to
 *                 Motor A and -steer to Motor B for differential turning.
 *                 Motor A is the LEFT motor — if the object is to the right
 *                 (positive lateral error) a positive steer speeds up the
 *                 left motor and slows the right, turning right.
 *                 Swap sign of steer here if your wiring is the other way.
 *
 * The deadband snap is applied AFTER mixing so both motors honour the
 * hardware minimum.  If a mixed value lands below the deadband but above
 * zero, it is clamped up to PWM_MIN_DEADBAND to ensure the motor actually
 * turns rather than buzzing motionless.
 */
static void PID_Drive_Motors_Steered(float output, float steer)
{
    /* ── Clamp raw PID output ─────────────────────────────────────────── */
    if      (output >  PWM_MAX) output =  PWM_MAX;
    else if (output < -PWM_MAX) output = -PWM_MAX;

    /* ── Mix base drive with steering differential ───────────────────── */
    float outA = output + steer;   /* Motor A — LEFT  */
    float outB = output - steer;   /* Motor B — RIGHT */

    /* ── Per-motor clamp to hardware limits ──────────────────────────── */
    if (outA >  PWM_MAX) outA =  PWM_MAX;
    if (outA < -PWM_MAX) outA = -PWM_MAX;
    if (outB >  PWM_MAX) outB =  PWM_MAX;
    if (outB < -PWM_MAX) outB = -PWM_MAX;

    /* ── Deadband: if nonzero, snap up to minimum effective duty ─────── */
    if (outA > 0.0f && outA < PWM_MIN_DEADBAND) outA = PWM_MIN_DEADBAND;
    if (outA < 0.0f && outA > -PWM_MIN_DEADBAND) outA = -PWM_MIN_DEADBAND;
    if (outB > 0.0f && outB < PWM_MIN_DEADBAND) outB = PWM_MIN_DEADBAND;
    if (outB < 0.0f && outB > -PWM_MIN_DEADBAND) outB = -PWM_MIN_DEADBAND;

    /* ── Apply to motors ─────────────────────────────────────────────── */
    if (outA > 0.0f)       MotorA_Set((uint16_t)outA, 1);
    else if (outA < 0.0f)  MotorA_Set((uint16_t)(-outA), 0);
    else                   MotorA_Brake();

    if (outB > 0.0f)       MotorB_Set((uint16_t)outB, 1);
    else if (outB < 0.0f)  MotorB_Set((uint16_t)(-outB), 0);
    else                   MotorB_Brake();
}

/**
 * @brief  Poll the Pixy2 and update the PID setpoint + steering bias.
 *
 * Called every PIXY_POLL_DIVIDER PID cycles from inside the main loop.
 * Never blocks — uses wait=false so the balance loop is never stalled.
 *
 * Tracking logic
 * ──────────────
 *  1. Request blocks for PIXY_TRACK_SIG only (non-blocking).
 *  2. If a valid block is found:
 *       a. If block width >= PIXY_STOP_WIDTH → object is close enough.
 *          Reset setpoint to 0 and clear steering (stop chasing).
 *       b. Otherwise → lean forward (setpoint = +DRIVE_LEAN_DEG) and
 *          compute a steering bias from the block's lateral position.
 *  3. If no block is found, increment pixy_lost_cnt.  After
 *     PIXY_LOST_FRAMES misses, cancel tracking (setpoint back to 0).
 *
 * @param  pid         Pointer to the active PID controller.
 * @param  steer_out   Output: updated steering differential (PWM counts).
 */
static void Pixy2_UpdateTracking(PIDController *pid, float *steer_out)
{
    int count = Pixy2_GetBlocks(&pixy, (uint8_t)(1u << (PIXY_TRACK_SIG - 1)),
                                 /*maxBlocks=*/1);

    if (count > 0 && pixy.blocks[0].width >= PIXY_MIN_WIDTH)
    {
        /* ── Valid detection ──────────────────────────────────────────── */
        pixy_lost_cnt = 0;

        uint16_t bw = pixy.blocks[0].width;
        uint16_t bx = pixy.blocks[0].x;

        if (bw >= PIXY_STOP_WIDTH)
        {
            /* ── Arrived: object is close enough — stop chasing ───────── */
            pid->setpoint = 0.0f;
            *steer_out    = 0.0f;
            printf("PIXY: arrived (w=%u) — holding balance\r\n", bw);
        }
        else
        {
            /* ── Tracking: lean forward and steer toward the object ────── */
            pid->setpoint = DRIVE_LEAN_DEG;

            /* Lateral error: positive = object is to the right of centre.
             * Multiply by STEER_GAIN to get a PWM differential.
             * The sign convention here assumes Motor A is LEFT:
             *   steer > 0 → A faster, B slower → turns right toward object.
             * If the robot turns the wrong way, negate the expression. */
            float lateral_err = (float)bx - (float)PIXY_FRAME_CX;
            *steer_out = lateral_err * STEER_GAIN;

            /* Clamp steering to a safe fraction of PWM_MAX so it cannot
             * overpower the balance correction on one side. */
            if (*steer_out >  (PWM_MAX * 0.25f)) *steer_out =  (PWM_MAX * 0.25f);
            if (*steer_out < -(PWM_MAX * 0.25f)) *steer_out = -(PWM_MAX * 0.25f);

            printf("PIXY: tracking sig=%u x=%u w=%u steer=%.1f\r\n",
                   pixy.blocks[0].signature, bx, bw, *steer_out);
        }
    }
    else
    {
        /* ── No detection this frame ─────────────────────────────────── */
        if (pixy_lost_cnt < PIXY_LOST_FRAMES)
        {
            pixy_lost_cnt++;
        }
        else
        {
            /* Object gone long enough — cancel tracking */
            if (pid->setpoint != 0.0f || *steer_out != 0.0f)
                printf("PIXY: target lost — resuming balance\r\n");
            pid->setpoint = 0.0f;
            *steer_out    = 0.0f;
        }
    }
}

void PID_Drive_Motors(float output)
{
    /* Hard clamp to PWM_MAX defined in pid.h (800) */
    if      (output >  PWM_MAX) output =  PWM_MAX;
    else if (output < -PWM_MAX) output = -PWM_MAX;
 
    /* Below deadband motors don't actually spin — skip to avoid buzzing */
    if (output > 0.0f && output <  PWM_MIN_DEADBAND) output =  PWM_MIN_DEADBAND;
    if (output < 0.0f && output > -PWM_MIN_DEADBAND) output = -PWM_MIN_DEADBAND;
 
    uint16_t duty = (uint16_t)fabsf(output);
 
    if (output > 0.0f) {
        /* Drive FORWARD — both motors forward */
        MotorA_Set(duty, 1);
        MotorB_Set(duty, 1);
    }
    else if (output < 0.0f) {
        /* Drive BACKWARD — both motors reverse */
        MotorA_Set(duty, 0);
        MotorB_Set(duty, 0);
    }
    else {
        /* Hard brake — robot is in deadzone */
        MotorA_Brake();
        MotorB_Brake();
    }
}

void Sensor_Init(void)
{
	BNO_Status_t Status = {0};

	//Init structure definition section
	BNO055_Init_t BNO055_InitStruct = {0};

	//Reset section
	ResetBNO055();
	HAL_Delay(800);
	/*============================ *BNO055 Initialization* ============================*/

	BNO055_InitStruct.ACC_Range = Range_4G;			//Range_X
	BNO055_InitStruct.Axis = DEFAULT_AXIS_REMAP;			//value will be entered by looking at the data sheet
	BNO055_InitStruct.Axis_sign = 0x04;		//value will be entered by looking at the data sheet
	BNO055_InitStruct.Clock_Source = CLOCK_EXTERNAL;		//CLOCK_EXTERNAL or CLOCK_INTERNAL
	BNO055_InitStruct.Mode = BNO055_NORMAL_MODE;			//BNO055_X_MODE   X:NORMAL, LOWPOWER, SUSPEND
	BNO055_InitStruct.OP_Modes = IMU;
	BNO055_InitStruct.Unit_Sel = (UNIT_ORI_ANDROID | UNIT_TEMP_CELCIUS | UNIT_EUL_DEG | UNIT_GYRO_DPS | UNIT_ACC_MS2);
									//(UNIT_ORI_X | UNIT_TEMP_X | UNIT_EUL_X | UNIT_GYRO_X | UNIT_ACC_X)
	BNO055_Init(BNO055_InitStruct);

	//------------------BNO055 Calibration------------------

	/*This function allows the sensor offset data obtained after BNO055 is calibrated once to be written
	 *to the registers. In this way, there is no need to calibrate the sensor every time it is powered on.
	 */
	//setSensorOffsets(OffsetDatas);

	/*-=-=-=-=-=-=Calibration Part-=-=-=-=-=-=*/
	uint8_t calibBuf[CALIB_DATA_SIZE];
	bool needs_calibration = false;

	/*
	if(Calibrate_BNO055())
	{
		getSensorOffsets(OffsetDatas);
	}
	else
	{
		printf("Sensor calibration failed.\nFailed to retrieve offset data\n");
	}*/

	// hold user button at boot to force recalibration
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
		printf("User button held — forcing recalibration.\r\n");
		Calib_Flash_Erase();
		needs_calibration = true;

	} else if (Calib_Flash_Load(calibBuf)) {
		// Restore saved offsets to BNO055

		setSensorOffsets(calibBuf);
		printf("Calibration restored from flash — skipping recalibration.\r\n");

	} else {

		// Flash load failed or was empty
		needs_calibration = true;
	}

	if (needs_calibration) {

		// No saved data — run full calibration
		if(Calibrate_BNO055())
			{
				getSensorOffsets(OffsetDatas);	// read offsets from BNO055
				Calib_Flash_Save(OffsetDatas);     // save to flash for next boot
			}
			else
			{
				printf("Sensor calibration failed.\nFailed to retrieve offset data\n");
			}

	}


	Check_Status(&Status);
	printf("Selftest Result: %d\t",Status.STresult);
	printf("System Status: %d\t",Status.SYSStatus);
	printf("System Error: %d\n",Status.SYSError);

}

void BNO055_setup() {
	// initialize the sensor
	Sensor_Init();

	// Fetch the current calibration levels
	Calib_status_t current_calib = {0};
	getCalibration(&current_calib);

	// Print them out to see if the flash memory injection worked
	printf("Boot Calibration Level -> System: %d | Gyro: %d | Accel: %d\r\n",
	       current_calib.System, current_calib.Gyro, current_calib.Acc);

}

uint8_t PS2_RX[9];
uint8_t PS2_TX[9] = { 0x01, 0x42};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	VL53L0X_Error status;
	uint32_t refSpadCount = 0;
	uint8_t isApertureSpads = 0;
	uint8_t VhvSettings = 0;
	uint8_t PhaseCal = 0;
	VL53L0X_RangingMeasurementData_t RangingData;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  /* Motor A */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  /* Motor B */

  // initialize controller
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  // TODO: CONFLICTS WITH CONTROLLER
  // Ensure all LEDs start off 
	// LD2 blue = PB7, LD3 red = PB14
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  // initialize the sensor
  BNO055_setup();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* Motor A enable */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   /* Motor B enable */
 
  /* Start with both motors braked */
  MotorA_Brake();
  MotorB_Brake();

  /*pixy init*/
  // 3 slow blinks on LD2 (blue) = board alive, about to attempt Pixy2 init
	for (int i = 0; i < 3; i++) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(500);
	}

	if (Pixy2_Init(&pixy, &hspi1, GPIOD, GPIO_PIN_14) == HAL_OK) {
		pixy_enabled = true;
		// 2 fast blinks on LD2 (blue) = Pixy2 init succeeded
		for (int i = 0; i < 2; i++) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_Delay(150);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_Delay(150);
		}
		printf("Pixy2 init OK — fw %u.%u\r\n",
		       pixy.version.firmware[0], pixy.version.firmware[1]);
	} else {
		pixy_enabled = false;
		// Single long red blink = Pixy2 failed, continuing without it
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		printf("Pixy2 init FAILED — running balance-only mode\r\n");
	}
    // Sensor boot up
  HAL_Delay(10);

  /*VL53l0x*/
  /*
  memset(&vl53_dev, 0, sizeof(vl53_dev));
  Dev->I2cDevAddr = VL53L0X_ADDR;
  Dev->comms_type = 1;
  Dev->comms_speed_khz = 100;

  status = VL53L0X_DataInit(Dev);
  if (status != VL53L0X_ERROR_NONE)
  {
    printf("DataInit failed: %d\r\n", status);
    Error_Handler();
  }

  status = VL53L0X_StaticInit(Dev);
  if (status != VL53L0X_ERROR_NONE)
  {
    printf("StaticInit failed: %d\r\n", status);
    Error_Handler();
  }

  status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
  if (status != VL53L0X_ERROR_NONE)
  {
    printf("RefSpadManagement failed: %d\r\n", status);
    Error_Handler();
  }

  status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
  if (status != VL53L0X_ERROR_NONE)
  {
    printf("RefCalibration failed: %d\r\n", status);
    Error_Handler();
  }

  status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  if (status != VL53L0X_ERROR_NONE)
  {
    printf("SetDeviceMode failed: %d\r\n", status);
    Error_Handler();
  }

  printf("VL53L0X ready\r\n");
  */

  PID_Init(&pid);
  pid.setpoint = 0.0f;
  last_pid_tick = HAL_GetTick();

  // start the DMA read
  HAL_StatusTypeDef BNO055_status = BNO055_Read_DMA(imu_raw_data);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* ========== PS2 CONTROLLER CODE (commented out) ========== */
    /*
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
    HAL_SPI_TransmitReceive(&hspi3, PS2_TX, PS2_RX, 9, 10);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
    */

	  if(imu_data_ready) {
		  // Immediately clear the flag
		  imu_data_ready = false;

      // compute delta time
      uint32_t now = HAL_GetTick();
      float dt = (now - last_pid_tick) / 1000.0f;
      last_pid_tick = now;

		  // Kick off the next DMA read. The CPU moves on instantly.
		  BNO055_status = BNO055_Read_DMA(imu_raw_data);

      /* ── Pixy2 tracking update ─────────────────────────────────────────
       * Polled every PIXY_POLL_DIVIDER PID cycles (~50 Hz at 100 Hz PID).
       * Updates pid.setpoint and steer_output BEFORE the PID runs, so any
       * lean command is in effect on this very cycle.
       * Never stalls — Pixy2_GetBlocks uses wait=false internally.        */
      if (pixy_enabled)
      {
          pixy_poll_ctr++;
          if (pixy_poll_ctr >= PIXY_POLL_DIVIDER)
          {
              pixy_poll_ctr = 0;
              Pixy2_UpdateTracking(&pid, &steer_output);
          }
      }

      /* Get pitch and pitch rate from BNO055 */
      float pitch      = BNO055.Euler.Y;    /* degrees from vertical */
      float pitch_rate = BNO055.Gyro.Y;     /* degrees/sec, used as D term */

      /* ── PID Control ────────────────────────────────────────────────── */
      bool ok = PID_Update(&pid, pitch, pitch_rate, dt);

      if (!ok) {
        /* Robot fell past recovery angle — hard brake, clear tracking     */
    	MotorA_Brake();
    	MotorB_Brake();
    	pid.setpoint  = 0.0f;   /* cancel lean so recovery starts level     */
    	steer_output  = 0.0f;
    	pixy_lost_cnt = PIXY_LOST_FRAMES;  /* force target re-acquire        */
        printf("FALLEN — STOPPED | Pitch: %.2f\r\n", pitch);
        continue;
      }

      if (pid.output == 0.0f) {
        /* Within deadzone — hard brake */
        MotorA_Brake();
        MotorB_Brake();
      } else {
        /* Drive motors: PID output is the shared base (forward/backward).
         * steer_output adds a differential between Motor A (LEFT) and
         * Motor B (RIGHT).  When steer_output == 0 this behaves identically
         * to the original PID_Drive_Motors(pid.output).                   */
        PID_Drive_Motors_Steered(pid.output, steer_output);
      }

      /* ── Debug print ─────────────────────────────────────────────────── */
      printf("P:%.2f R:%.2f SP:%.2f Out:%.0f Steer:%.0f\r\n",
              pitch, pitch_rate, pid.setpoint, pid.output, steer_output);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PIXY_CS_Pin|MOTOR_A_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_B_IN1_Pin
                          |MOTOR_B_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PIXY_CS_Pin MOTOR_A_IN1_Pin MOTOR_A_IN2_Pin MOTOR_B_IN1_Pin
                           MOTOR_B_IN2_Pin */
  GPIO_InitStruct.Pin = PIXY_CS_Pin|MOTOR_A_IN1_Pin|MOTOR_A_IN2_Pin|MOTOR_B_IN1_Pin
                          |MOTOR_B_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_14;  // LD2 blue | LD3 red
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);  // LD2 blue off
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // LD3 red  off
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C2) {

    	// call store function to store the information
    	BNO055_DMA_store(&BNO055);

        // Flag the main loop that ALL data is fresh!
        imu_data_ready = true;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */