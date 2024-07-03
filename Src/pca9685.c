/*
 * pca9685.c
 *
 *  Created on: 20.01.2019
 *      Author: Mateusz Salamon
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/nigdy-wiecej-multipleksowania-na-gpio!-max7219-w-akcji-cz-3/
 *      GitHub:  https://github.com/lamik/Servos_PWM_STM32_HAL
 */

#include "main.h"
#include "cmsis_os.h"

#include "pca9685.h"
#include "math.h"

I2C_HandleTypeDef *pca9685_i2c;

PCA9685_STATUS PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
	if(Value) Value = 1; 

	if( PCA9685_OK != PCA9685_ReadRegisters( Register, &tmp, 1) )
	{
		return PCA9685_ERROR;
	}
	tmp &= ~((1<<PCA9685_MODE1_RESTART_BIT)|(1<<Bit));
	tmp |= (Value&1)<<Bit;

	if( HAL_OK != HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_I2C_WRITE_ADDR, Register, 1, &tmp, 1, PCA9685_I2C_TIMEOUT ) )
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

/**
 * @brief 
 * @details 7.6 section of datasheet
 * 
 * @return PCA9685_STATUS 
 */
PCA9685_STATUS PCA9685_SoftwareReset(void)
{
	uint8_t reset_command = 0x06; // Comando de reinicio por software

	if( HAL_OK != HAL_I2C_Master_Transmit( pca9685_i2c, PCA9685_GENERAL_CALL_ADDRESS, &reset_command, 1, PCA9685_I2C_TIMEOUT ) )
	{
		return PCA9685_ERROR;
	}

	osDelay(10);

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, Enable);
}

PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, Enable);
}

PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, Enable);
}

PCA9685_STATUS PCA9685_SubaddressRespond(SubaddressBit Subaddress, uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, Subaddress, Enable);
}

PCA9685_STATUS PCA9685_AllCallRespond(uint8_t Enable)
{
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_ALCALL_BIT, Enable);
}

/**
 * @brief Set pwm frecuency
 * 
 * @param Frequency iz Hz
 * @return PCA9685_STATUS 
 */
PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency)
{
	float PrescalerVal;
	uint8_t Prescale;

	if(Frequency >= 1526)
	{
		Prescale = 0x03;
	}
	else if(Frequency <= 24)
	{
		Prescale = 0xFF;
	}
	else
	{
		PrescalerVal = (25000000 / (4096.0 * (float)Frequency)) - 1;
		Prescale = floor(PrescalerVal + 0.5);
	}

	uint8_t old_mode;

	if( PCA9685_OK != PCA9685_ReadRegisters( PCA9685_MODE1, &old_mode, 1 ) ) // 1 read old_mode
	{
		return PCA9685_ERROR;
	}

  	uint8_t new_mode = ( old_mode & 0x7F ) | 0x10; // sleep //2

	if( PCA9685_OK != PCA9685_WriteReg( PCA9685_MODE1, new_mode ) ) // 3 go to sleep
	{
		return PCA9685_ERROR;
	}

	if( PCA9685_OK != PCA9685_WriteReg( PCA9685_PRESCALE, Prescale ) ) // 4 Set prescale
	{
		return PCA9685_ERROR;
	}

	if( PCA9685_OK != PCA9685_WriteReg( PCA9685_MODE1, old_mode ) ) // 5
	{
		return PCA9685_ERROR;
	}

	osDelay(10);

	if( PCA9685_OK != PCA9685_WriteReg( PCA9685_MODE1, old_mode | 0xA0 ) ) //6 //  This sets the MODE1 register to turn on auto increment.
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
	uint8_t RegisterAddress;
	uint8_t data[4];

	RegisterAddress = PCA9685_LED0_ON_L + (4 * Channel);
	data[0] = OnTime & 0xFF;
	data[1] = OnTime>>8;
	data[2] = OffTime & 0xFF;
	data[3] = OffTime>>8;

	if(HAL_OK != HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_I2C_WRITE_ADDR, RegisterAddress, 1, data, 4, PCA9685_I2C_TIMEOUT ))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert)
{
  if(Value > 4095) Value = 4095;

  if (Invert) {
    if (Value == 0) {
      // Special value for signal fully on.
      return PCA9685_SetPwm(Channel, 4096, 0);
    }
    else if (Value == 4095) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(Channel, 0, 4095-Value);
    }
  }
  else {
    if (Value == 4095) {
      // Special value for signal fully on.
    	return PCA9685_SetPwm(Channel, 4096, 0);
    }
    else if (Value == 0) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(Channel, 0, Value);
    }
  }
}

#ifdef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
	float Value;
	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;

	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;

	return PCA9685_SetPin(Channel, (uint16_t)Value, 0);
}
#endif

PCA9685_STATUS PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
	pca9685_i2c = hi2c;
	uint8_t mode_1_std_op = 0x00; // modo operacion estandar
	uint8_t mode_2_std_op = 0x04; // salida de totem-pole en los pines PWM

	if ( PCA9685_ERROR == PCA9685_SoftwareReset() )
	{
		return PCA9685_ERROR;
	}

	if ( PCA9685_OK != PCA9685_WriteReg( PCA9685_MODE1, mode_1_std_op  ) )
	{
        return PCA9685_ERROR;
	}
    
    // Configurar el registro MODE2
    if ( PCA9685_OK != PCA9685_WriteReg( PCA9685_MODE2, mode_2_std_op  ) )
	{
        return PCA9685_ERROR; 
    }

#ifdef PCA9685_SERVO_MODE
	if ( PCA9685_ERROR == PCA9685_SetPwmFrequency( FREQUENCY ) )
#else
	if ( PCA9685_ERROR == PCA9685_SetPwmFrequency(1000) )
#endif
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;//PCA9685_AutoIncrement(1);
}

 /**
  * @brief Función para escribir en un registro del PCA9685
  * 
  * @param reg 
  * @param value 
  * @return PCA9685_STATUS 
  */
PCA9685_STATUS PCA9685_WriteReg(uint8_t reg, uint8_t value) 
{
    uint8_t data[2] = {reg, value};

    if ( HAL_OK != HAL_I2C_Master_Transmit( pca9685_i2c, PCA9685_I2C_WRITE_ADDR, data, 2, PCA9685_I2C_TIMEOUT ) ) 
	{
		return PCA9685_ERROR;
    }

    return PCA9685_OK;
}

PCA9685_STATUS PCA9685_ReadRegisters(uint8_t reg, uint8_t *pdata, uint8_t size_data)
{
	if( HAL_OK != HAL_I2C_Mem_Read(pca9685_i2c, PCA9685_I2C_READ_ADDR, reg, 1, pdata, size_data, PCA9685_I2C_TIMEOUT ) )
    {
		return PCA9685_ERROR;
	}

    return PCA9685_OK;
}
