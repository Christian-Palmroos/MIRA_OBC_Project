#include "common_porting.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "bmp390_task.h"
#include "bmp3.h"

#if defined(USE_BMP390)

extern volatile uint8_t int1_flag;
extern volatile uint8_t int2_flag;

static uint8_t dev_addr = 0;

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            PDEBUG("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            PDEBUG("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            PDEBUG("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            PDEBUG("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            PDEBUG("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            PDEBUG("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            PDEBUG("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            PDEBUG("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
{
	int8_t rslt = BMP3_OK;

	if(bmp3 != NULL)
	{
		/* Bus configuration : I2C */
		if (intf == BMP3_I2C_INTF)
		{
			PDEBUG("I2C Interface\n");
			dev_addr = BMP3_ADDR_I2C_SEC;
			bmp3->read = SensorAPI_I2Cx_Read;
			bmp3->write = SensorAPI_I2Cx_Write;
			bmp3->intf = BMP3_I2C_INTF;
		}
		/* Bus configuration : SPI */
		else if (intf == BMP3_SPI_INTF)
		{
			PDEBUG("SPI Interface\n");
			dev_addr = 0;
			bmp3->read = SensorAPI_SPIx_Read;
			bmp3->write = SensorAPI_SPIx_Write;
			bmp3->intf = BMP3_SPI_INTF;
		}

		bmp3->delay_us = bmp3_delay_us;
		bmp3->intf_ptr = &dev_addr;
	}
	else
	{
		rslt = BMP3_E_NULL_PTR;
	}

	return rslt;
}


void StartBMP390Task(void const * argument)
{
	int8_t rslt = 0;
	#if defined(READ_SENSOR_DATA)
	/* Iteration count to run example code */
	#define ITERATION  UINT8_C(100)
	uint8_t loop = 0;
	uint8_t settings_sel;
	struct bmp3_dev dev;
	struct bmp3_data data = { 0 };
	struct bmp3_settings settings = { 0 };
	struct bmp3_status status = { { 0 } };

	/* Interface reference is given as a parameter
	*		   For I2C : BMP3_I2C_INTF
	*		   For SPI : BMP3_SPI_INTF
	*/
	#if defined(USE_I2C_INTERFACE)
	rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
	#elif defined(USE_SPI_INTERFACE)
	rslt = bmp3_interface_init(&dev, BMP3_SPI_INTF);
	#endif
	bmp3_check_rslt("bmp3_interface_init", rslt);

	rslt = bmp3_init(&dev);
	bmp3_check_rslt("bmp3_init", rslt);

	settings.int_settings.drdy_en = BMP3_ENABLE;
	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
	settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
	settings.odr_filter.odr = BMP3_ODR_100_HZ;

	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
	   BMP3_SEL_DRDY_EN;

	rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
	bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

	settings.op_mode = BMP3_MODE_NORMAL;
	rslt = bmp3_set_op_mode(&settings, &dev);
	bmp3_check_rslt("bmp3_set_op_mode", rslt);

	while (loop < ITERATION)
	{
		rslt = bmp3_get_status(&status, &dev);
		bmp3_check_rslt("bmp3_get_status", rslt);

		/* Read temperature and pressure data iteratively based on data ready interrupt */
		if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
		{
			/*
			* First parameter indicates the type of data to be read
			* BMP3_PRESS_TEMP : To read pressure and temperature data
			* BMP3_TEMP	   : To read only temperature data
			* BMP3_PRESS	   : To read only pressure data
			*/
			rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
			bmp3_check_rslt("bmp3_get_sensor_data", rslt);

			/* NOTE : Read status register again to clear data ready interrupt status */
			rslt = bmp3_get_status(&status, &dev);
			bmp3_check_rslt("bmp3_get_status", rslt);

			#ifdef BMP3_FLOAT_COMPENSATION
			PDEBUG("Data[%d]  T: %.2f deg C, P: %.2f Pa\n", loop, (data.temperature), (data.pressure));
			#else
			PDEBUG("Data[%d]  T: %ld deg C, P: %lu Pa\n", loop, (long int)(int32_t)(data.temperature / 100),
		  		 (long unsigned int)(uint32_t)(data.pressure / 100));
			#endif

			loop = loop + 1;
		}
	}
	while(1)
	{
	}
	#endif
}
#endif
