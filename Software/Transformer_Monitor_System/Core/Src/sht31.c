#include "sht31.h"
#include "i2c.h"

//I2C写2字节命令
uint8_t i2c_write_cmd(uint16_t cmd)
{
    uint8_t cmd_buff[2];
    cmd_buff[0] = cmd>>8;
    cmd_buff[1] = cmd;

    return HAL_I2C_Master_Transmit(&hi2c1,SHT_ADDR,cmd_buff,2,0xffff);
}

//CRC校验计算
#define CRC8_POLYNOMIAL 0x31
uint8_t CheckCrc8(uint8_t* message, uint8_t initial_value)
{
    uint8_t  remainder;    
    uint8_t  i = 0, j = 0;  

    remainder = initial_value;

    for(j = 0; j < 2;j++)
    {
        remainder ^= message[j];

        for (i = 0; i < 8; i++)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1)^CRC8_POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

//初始化代码
uint8_t sht31_init(void)
{
    //soft reset
    i2c_write_cmd(0x30a2);
    HAL_Delay(25);

    return i2c_write_cmd(0x2220);//repeat medium_2
}

//read temp&humi from sensor
//1-ERR
//0-OK
uint8_t sht31_sample(float *t, float *h)
{
    uint8_t read_buff[6] = {0};

    uint16_t temp_value;
    uint16_t humi_value;

    i2c_write_cmd(0xe000);//read for period mode

    //read value
    if(HAL_I2C_Master_Receive(&hi2c1,SHT_ADDR|0x01,read_buff,6,0xffff) != HAL_OK)
    {
        return HAL_ERROR;
    }

    //check crc
    if(CheckCrc8(read_buff, 0xFF) != read_buff[2] || CheckCrc8(&read_buff[3], 0xFF) != read_buff[5])
  {
        return HAL_ERROR;
    }

    //translate
     temp_value = ((uint16_t)read_buff[0]<<8)|read_buff[1];
   *t = -45 + 175*((float)temp_value/65535);

    humi_value = ((uint16_t)read_buff[3]<<8)|read_buff[4];
    *h = 100 * ((float)humi_value / 65535);

    return HAL_OK;
}
