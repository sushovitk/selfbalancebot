/*******************************************************************************
Copyright � 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/**
 * @file VL53L0X_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */
#include "vl53l0x_platform.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

static VL53L0X_Error vl53_to_error(HAL_StatusTypeDef hal_status)
{
  if (hal_status == HAL_OK)
  {
    return VL53L0X_ERROR_NONE;
  }
  return VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
  return vl53_to_error(
      HAL_I2C_Mem_Write(&hi2c1,
                        Dev->I2cDevAddr,
                        index,
                        I2C_MEMADD_SIZE_8BIT,
                        pdata,
                        count,
                        HAL_MAX_DELAY));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
  return vl53_to_error(
      HAL_I2C_Mem_Read(&hi2c1,
                       Dev->I2cDevAddr,
                       index,
                       I2C_MEMADD_SIZE_8BIT,
                       pdata,
                       count,
                       HAL_MAX_DELAY));
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
  return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(data >> 8);
  buffer[1] = (uint8_t)(data & 0xFF);
  return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
  uint8_t buffer[4];
  buffer[0] = (uint8_t)(data >> 24);
  buffer[1] = (uint8_t)(data >> 16);
  buffer[2] = (uint8_t)(data >> 8);
  buffer[3] = (uint8_t)(data & 0xFF);
  return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
  return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
  uint8_t buffer[2];
  VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buffer, 2);
  if (status == VL53L0X_ERROR_NONE)
  {
    *data = ((uint16_t)buffer[0] << 8) | buffer[1];
  }
  return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
  uint8_t buffer[4];
  VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buffer, 4);
  if (status == VL53L0X_ERROR_NONE)
  {
    *data = ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8)  |
            buffer[3];
  }
  return status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t andData, uint8_t orData)
{
  uint8_t data = 0;
  VL53L0X_Error status = VL53L0X_RdByte(Dev, index, &data);
  if (status != VL53L0X_ERROR_NONE)
  {
    return status;
  }

  data = (data & andData) | orData;
  return VL53L0X_WrByte(Dev, index, data);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
  (void)Dev;
  HAL_Delay(1);
  return VL53L0X_ERROR_NONE;
}
