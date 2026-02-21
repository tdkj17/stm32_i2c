/*
 * learn_oled.h
 *
 *  Created on: 2026年2月19日
 *      Author: jerry
 */

#ifndef INC_JERRY_OLED_H_
#define INC_JERRY_OLED_H_
#include "stdbool.h"

// process is call I2C_init -> call I2C_Start_send_bit -> call I2C_send_long_byte -> call I2C_Stop_send_bit

enum I2C_status{i2c_status_doing, i2c_status_free};

typedef struct {
	GPIO_TypeDef* GPIOX;
	uint16_t SCL;
	uint16_t SDA;
	uint16_t RST;
	enum I2C_status status;

} I2C_Pin_Content;

const int I2C_loop_nop_number = 20;

void I2C_Stop_send_bit(I2C_Pin_Content *PIN_CONTENT);

I2C_Pin_Content I2C_init(GPIO_TypeDef *GPIOX, uint16_t SCL, uint16_t SDA, uint16_t RST){
	I2C_Pin_Content temp_pin;
	temp_pin.GPIOX = GPIOX;
	temp_pin.SCL = SCL;
	temp_pin.SDA = SDA;
	temp_pin.RST = RST;
	temp_pin.status = i2c_status_free;
	return temp_pin;
}

void I2C_LOOP_nop(const int us){
	volatile int number = us * (SystemCoreClock/1000000);
	while(number--)
		__NOP();
}

void I2C_Start_send_bit(I2C_Pin_Content *PIN_CONTENT){
	if(PIN_CONTENT->status == i2c_status_doing){
		return ;
	}

	HAL_GPIO_WritePin(PIN_CONTENT->GPIOX, PIN_CONTENT->SDA, GPIO_PIN_SET);

	HAL_GPIO_WritePin(PIN_CONTENT->GPIOX, PIN_CONTENT->SCL, GPIO_PIN_SET);
	I2C_LOOP_nop(I2C_loop_nop_number);

	HAL_GPIO_WritePin(PIN_CONTENT->GPIOX, PIN_CONTENT->SDA, GPIO_PIN_RESET);

	I2C_LOOP_nop(I2C_loop_nop_number);
	HAL_GPIO_WritePin(PIN_CONTENT->GPIOX, PIN_CONTENT->SCL, GPIO_PIN_RESET);

	PIN_CONTENT->status = i2c_status_doing;
	/* call this function end
	 *  - SDA LOW
	 *  - SCL LOW
	 * */
}

const int over_read_count = 5000;

bool I2C_send_one_byte(I2C_Pin_Content *pin, const uint8_t data){
	bool result = true;
	if(pin->status == i2c_status_free)
		return false;
	for(int i = 0; i < 8; ++i){
		HAL_GPIO_WritePin(pin->GPIOX, pin->SCL, GPIO_PIN_RESET);
		I2C_LOOP_nop(I2C_loop_nop_number);

		if((data << i) & 0x80){
			HAL_GPIO_WritePin(pin->GPIOX, pin->SDA, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(pin->GPIOX, pin->SDA, GPIO_PIN_RESET);
		}

		HAL_GPIO_WritePin(pin->GPIOX, pin->SCL, GPIO_PIN_SET);
		I2C_LOOP_nop(I2C_loop_nop_number);
	}

	HAL_GPIO_WritePin(pin->GPIOX, pin->SCL, GPIO_PIN_RESET);
	I2C_LOOP_nop(I2C_loop_nop_number);

	HAL_GPIO_WritePin(pin->GPIOX, pin->SDA, GPIO_PIN_SET);
	I2C_LOOP_nop(I2C_loop_nop_number);

	HAL_GPIO_WritePin(pin->GPIOX, pin->SCL, GPIO_PIN_SET);
	for(int i = 0; HAL_GPIO_ReadPin(pin->GPIOX, pin->SDA); ++i){
		I2C_LOOP_nop(1);
		if(i == over_read_count){
			result = false;
			break;
		}
	}
	HAL_GPIO_WritePin(pin->GPIOX, pin->SCL, GPIO_PIN_RESET);
	I2C_LOOP_nop(I2C_loop_nop_number);
	if(result){

		HAL_GPIO_WritePin(pin->GPIOX, pin->SDA, GPIO_PIN_RESET);
		return result;
	}else{
		I2C_Stop_send_bit(pin);
		return result;
	}

	/* call this function end
	 *  - SCL LOW
	 *  - SDA LOW
	 * */
}

bool I2C_send_long_byte(I2C_Pin_Content *pin, const uint8_t* data, const size_t length){
	for(size_t i = 0; i < length ; ++i){
		if(!(I2C_send_one_byte(pin, (char)data[i]))){
			return false;
		}
	}
	return true;
}

void I2C_Stop_send_bit(I2C_Pin_Content *PIN_CONTENT){
	if(PIN_CONTENT->status == i2c_status_free)
		return ;

	HAL_GPIO_WritePin(PIN_CONTENT->GPIOX, PIN_CONTENT->SDA, GPIO_PIN_RESET);
    I2C_LOOP_nop(I2C_loop_nop_number);

	HAL_GPIO_WritePin(PIN_CONTENT->GPIOX, PIN_CONTENT->SCL, GPIO_PIN_SET);
	I2C_LOOP_nop(I2C_loop_nop_number);
	// BECOME ACK END STAtUs , STOP ERROR ACK

	HAL_GPIO_WritePin(PIN_CONTENT->GPIOX, PIN_CONTENT->SDA, GPIO_PIN_SET);
	I2C_LOOP_nop(I2C_loop_nop_number);
	PIN_CONTENT->status = i2c_status_free;
	/* call this function end
	 *  - SCL HIGH
	 *  - SDA HIGH
	 * */

}







#endif /* INC_JERRY_OLED_H_ */

