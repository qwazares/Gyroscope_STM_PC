/* File Name			: gyroscope.c*/
/* Autor				: Krzysztof Kopeæ
/**
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================

 (#) Configurate gyroscope L3GD20 fill L3DG20_Init_Struct and use L3GD20_Init() function
 (#) Read outputs by L3GD20_Read_Axes();
**/

#include "gyroscope.h"
/********************************************************************************/

uint8_t L3GD20_Init(L3GD20_Init_Struct *Init)
{

	uint8_t data;
	uint8_t data_r;
	int i;

	//check device
	for(i=0; i<10; i++)
	{
		L3GD20_Read(L3GD20_WHO_AM_I_ADDR, &data, 1, Init);
	}

	if(data != I_AM_L3GD20)
		return 1;


	//Set CTRL_REG1
	data = 0;
	data |= Init->output_datarate |	Init->bandwidth | Init->power_mode | Init->axes;
	data_r = data;

	L3GD20_Write(L3GD20_CTRL_REG1_ADDR,&data,1, Init);
	L3GD20_Read(L3GD20_CTRL_REG1_ADDR,&data,1, Init);

	if(data != data_r)
		return 1;


	//Set CTRL_REG2
	data = 0;
	data |= Init->high_pass_filter_mode | Init->high_pass_cutoff_frequency;
	data_r = data;

	L3GD20_Write(L3GD20_CTRL_REG2_ADDR,&data,1, Init);
	L3GD20_Read(L3GD20_CTRL_REG2_ADDR,&data,1, Init);

	if(data != data_r)
		return 1;


	//Set CTRL_REG3
	data = 0;
	data |= Init->int1_status | Init->int1_active_edge | Init->int2_status;
	data_r = data;

	L3GD20_Write(L3GD20_CTRL_REG3_ADDR,&data,1, Init);
	L3GD20_Read(L3GD20_CTRL_REG3_ADDR,&data,1, Init);

	if(data != data_r)
		return 1;


	//Set CTRL_REG4
	data = 0;
	data |= Init->block_data_update | Init->endian_data | Init->full_scale;
	data_r = data;

	L3GD20_Write(L3GD20_CTRL_REG4_ADDR,&data,1, Init);
	L3GD20_Read(L3GD20_CTRL_REG4_ADDR,&data,1, Init);

	if(data != data_r)
		return 1;

	//Set CTRL_REG5
	data = 0;
	data |= Init->boot_mode | Init->high_pass_filter_status;
	data_r = data;

	L3GD20_Write(L3GD20_CTRL_REG5_ADDR,&data,1, Init);
	L3GD20_Read(L3GD20_CTRL_REG5_ADDR,&data,1, Init);

	if(data != data_r)
		return 1;

	return 0;
}

/********************************************************************************/

uint8_t L3GD20_Read_Axes(L3GD20_out_reg_axes *Output, L3GD20_Init_Struct *Gyro_Struct)
{

	L3GD20_Read(L3GD20_OUT_X_L_ADDR, &Output->out_x_l, 6, Gyro_Struct);

}

/*******************************************************************************/
uint8_t Gyro_Read_Axes(L3GD20_out_reg_axes *Input, Gyro_out_axes *Output, L3GD20_Init_Struct *Gyro_Struct)
{

	if(Gyro_Struct->endian_data == L3GD20_BLE_LSB)
	{
		Output->x_axis = Input->out_x_l + (Input->out_x_h << 8);
		Output->y_axis = Input->out_y_l + (Input->out_y_h << 8);
		Output->z_axis = Input->out_z_l + (Input->out_z_h << 8);
	}
	else if(Gyro_Struct->endian_data == L3GD20_BLE_MSB)
	{
		Output->x_axis = Input->out_x_h + (Input->out_x_l << 8);
		Output->y_axis = Input->out_y_h + (Input->out_y_l << 8);
		Output->z_axis = Input->out_z_h + (Input->out_z_l << 8);
	}
	else
		return 1;

	return 0;

}

/********************************************************************************/

uint8_t L3GD20_Read(uint8_t Address, uint8_t *data, uint16_t Size, L3GD20_Init_Struct *Gyro_Struct)
{
	if(Gyro_Struct == NULL)
		return 1;


	if(Size == 1)
	{
		Address |= L3GD20_READ | L3GD20_SINGLE_RW;

		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_Transmit(Gyro_Struct->spi,&Address,1,3000);
		HAL_SPI_Receive(Gyro_Struct->spi,data,1,3000);
		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_SET);
	}
	else if(Size > 1)
	{
		Address |= L3GD20_READ | L3GD20_MULTIPLE_RW;

		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_Transmit(Gyro_Struct->spi,&Address,1,3000);
		HAL_SPI_Receive(Gyro_Struct->spi,data,Size,3000);
		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_SET);

	}

	return 0;
}

/********************************************************************************/

uint8_t L3GD20_Write(uint8_t Address, uint8_t *data, uint16_t Size, L3GD20_Init_Struct *Gyro_Struct)
{
	if(Gyro_Struct == NULL)
		return 1;


	if(Size == 1)
	{
		Address |= L3GD20_WRITE | L3GD20_SINGLE_RW;

		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_Transmit(Gyro_Struct->spi,&Address,1,3000);
		HAL_SPI_Transmit(Gyro_Struct->spi,data,1,3000);
		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_SET);
	}
	else if(Size > 1)
	{
		Address |= L3GD20_WRITE | L3GD20_MULTIPLE_RW;

		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_Transmit(Gyro_Struct->spi,&Address,1,3000);
		HAL_SPI_Receive(Gyro_Struct->spi,data,Size,3000);
		HAL_GPIO_WritePin(Gyro_Struct->CS_Port,Gyro_Struct->CS_Pin,GPIO_PIN_SET);

	}

	return 0;
}

/********************************************************************************/


