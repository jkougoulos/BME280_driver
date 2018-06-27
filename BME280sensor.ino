#include <Wire.h>
#include "bme280.h"


struct bme280_dev dev;
int8_t rslt = BME280_OK;

void user_delay_ms(uint32_t period)
{
  delay( period );
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint16_t bytesread = 0;

    Wire.beginTransmission( dev_id );
    Wire.write( reg_addr );
    rslt = Wire.endTransmission( true );
    if( rslt )
      return rslt;

    bytesread = Wire.requestFrom( dev_id, (uint8_t)len, (uint8_t) true );

    if ( bytesread != len )
      return 99; //sanity check

    uint8_t i = 0;
    while( i < bytesread )
    {
      reg_data[i++] = Wire.read();
    }

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return 0;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */


    Wire.beginTransmission( dev_id );
    Wire.write( reg_addr );

    uint16_t i = 0;
    while( i < len )
    {
      Wire.write( reg_data[i++] );
    }

    rslt = Wire.endTransmission( true );

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return rslt;
}

void setup() {
  // put your setup code here, to run once:

  Wire.begin();

  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;

  dev.delay_ms = user_delay_ms;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;

  
  rslt = bme280_init(&dev);
}

void loop() {
  // put your main code here, to run repeatedly:

}
