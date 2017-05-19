/**
 ******************************************************************************
 * @file    multigas_sensor.h
 * @author  Boting Ren
 * @version V1.0.0
 * @date    19 May 2017
 * @brief   This file contains the class of a Mutichannel Gas Sensor library with I2C interface
 ******************************************************************************
 * @attention
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MBED_MULTIGAS_SENSOR_H
#define MBED_MULTIGAS_SENSOR_H

#include "mbed.h"

//#define _DEBUG
#define DEFAULT_SLAVE_ADDRESS  (0x04<<1)

#define ADDR_IS_SET             0           // This is read at initialization time, if 1126, set
#define ADDR_FACTORY_ADC_NH3    2
#define ADDR_FACTORY_ADC_CO     4
#define ADDR_FACTORY_ADC_NO2    6

#define ADDR_USER_ADC_HN3       8
#define ADDR_USER_ADC_CO        10
#define ADDR_USER_ADC_NO2       12
#define ADDR_IF_CALI            14          // IF USER HAD CALI

#define ADDR_I2C_ADDRESS        20

#define CH_VALUE_NH3            1
#define CH_VALUE_CO             2
#define CH_VALUE_NO2            3

#define CMD_ADC_RES0            1           // NH3
#define CMD_ADC_RES1            2           // CO
#define CMD_ADC_RES2            3           // NO2
#define CMD_ADC_RESALL          4           // ALL CHANNEL
#define CMD_CHANGE_I2C          5           // CHANGE I2C
#define CMD_READ_EEPROM         6           // READ EEPROM VALUE, RETURN UNSIGNED INT
#define CMD_SET_R0_ADC          7           // SET R0 ADC VALUE
#define CMD_GET_R0_ADC          8           // GET R0 ADC VALUE
#define CMD_GET_R0_ADC_FACTORY  9           // GET FACTORY R0 ADC VALUE
#define CMD_CONTROL_LED         10
#define CMD_CONTROL_PWR         11

/**
 * added a Cache feature for avoiding reload and recalculation within very short period, eg 500ms.
 * once the interval is bigger then the time, then reload and recalculation will run once.
 */
#define CATCH_TIME              500         // Cache expiration time for recalculate Ratio[x]

#ifdef _DEBUG
extern Serial pc;
#define DEBUG_PRINT(...) pc.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

enum{CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH};

/** MutichannelGasSensor class
 *
 *  MutichannelGasSensor: A library to measure gas data using Grove - Multichannel Gas Sensor (seeed)
 *
 *  Please refer the details of the sensor. http://wiki.seeed.cc/Grove-Multichannel_Gas_Sensor/
 *
 */

class MutichannelGasSensor{

public:
    /** Create a MutichannelGasSensor instance
     *  which is connected to specified I2C pins with specified address
     *
     * @param sda I2C-bus SDA pin
     * @param scl I2C-bus SCL pin
     * @param slave_adr (option) I2C-bus address (default: 0x04)
     */
    MutichannelGasSensor(PinName sda, PinName sck, char slave_adr = DEFAULT_SLAVE_ADDRESS);

    /** Create a MutichannelGasSensor instance
     *  which is connected to specified I2C pins with specified address
     *
     * @param i2c_obj I2C object (instance)
     * @param slave_adr (option) I2C-bus address (default: 0x04)
     */
    MutichannelGasSensor(I2C &i2c_obj, char slave_adr = DEFAULT_SLAVE_ADDRESS);

    /** Destructor of MutichannelGasSensor
     */
    virtual ~MutichannelGasSensor();

    /** Initializa MutichannelGasSensor
     */
    void initialize(void);

    //get gas concentration, unit: ppm
    float measure_CO(){return calcGas(CO);}
    float measure_NO2(){return calcGas(NO2);}
    float measure_NH3(){return calcGas(NH3);}
    float measure_C3H8(){return calcGas(C3H8);}
    float measure_C4H10(){return calcGas(C4H10);}
    float measure_CH4(){return calcGas(CH4);}
    float measure_H2(){return calcGas(H2);}
    float measure_C2H5OH(){return calcGas(C2H5OH);}

private:

    I2C  *i2c_p;
    I2C  &i2c;
    char address;     //I2C address of this MCU
    unsigned char __version;
    uint16_t adcValR0[3];
    uint16_t R0[3];        //sensors R0
    uint16_t Rs[3];        //sensors Rs
    float Ratio[3];        //calculated ration 0,1,2

    unsigned char readVersion(void);
    void HeaterPower(bool);
    int read_Multibytes(unsigned char, unsigned char,unsigned char, unsigned char );
    int readR0(void);
    int readRs(void);
    float calcGas(int gas);

    Timer  memo;
//    unsigned int readChAdcValue(int ch);

};



#endif // MBED_MULTIGAS_SENSOR_H
