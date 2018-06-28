

#include <Wire.h>
#include <utility/twi.h>
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


  Serial.begin(9600);
  Serial.println("Starting up!");
  
  Wire.begin();
#if F_CPU == 1000000        //koug
   TWBR = 0;  // get MAX speed on I2C bus (62.5Khz in 1Mhz CPU)
#endif
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;

  dev.delay_ms = user_delay_ms;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;

  
  rslt = bme280_init(&dev);
  if( rslt != BME280_OK )
  {
    Serial.print(rslt);
    Serial.println(" BME280 failed");
  }
  else
  {
    Serial.println("BME280 OK!");
  }
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_1X;
    dev->settings.osr_t = BME280_OVERSAMPLING_1X;
    dev->settings.filter = BME280_FILTER_COEFF_OFF;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL; // | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, dev);

    Serial.println("Temperature, Pressure, Humidity");
    /* Continuously stream sensor data */
    while (1) {
        unsigned long a,b,c;
        a = micros();
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        /* Wait for the measurement to complete and print data @25Hz */
        uint8_t bme_status = 0x08;
        
        while( bme_status & 0x08 )
        {
          rslt = bme280_get_regs(0xF3, &bme_status, 1, dev );
          delayMicroseconds(500);          
        }
        b = micros();
//        dev->delay_ms(1000);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        c = micros();
/*        
        Serial.print("it took ");
        Serial.print(b-a);
        Serial.print(" ");
        Serial.println(c-b);
*/
        print_sensor_data(&comp_data);
        delay(1000);
    }
    return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
        char data[40];
#ifdef BME280_FLOAT_ENABLE
//        printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);

        Serial.print(comp_data->temperature);
        Serial.print(" ");
        Serial.print(comp_data->humidity);
        Serial.print(" ");
        Serial.print(comp_data->pressure);
        Serial.println(" ");

#else
        sprintf(data,"%ld, %ld, %ld",(comp_data->temperature)/10, (long)((comp_data->humidity)/102.4), (comp_data->pressure)/10);
        Serial.println(data);
#endif

}


void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Loop!");
  stream_sensor_data_forced_mode( &dev );
//  delay(3000);

}
