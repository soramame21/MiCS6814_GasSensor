/**
 ******************************************************************************
 * @file    multigas_sensor.h
 * @author  Boting Ren
 * @version V1.0.1
 * @date    22 May 2017
 * @brief   This file contains the class of a Multichannel Gas Sensor library with I2C interface
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
 *
 * @par     History
 * - 19 May 2017, V1.0.0, Boting Ren
 * - 22 May 2017, V1.0.1, Boting Ren
 * 1. changed class name and private variables name
 * 2. removed Public APIs (measure_xxx())
 * 3. removed removed timer and Cache feature
 * 4. removed code working for version 1 firmware
 * 5. moved class private variables into member functions
 * 6. added more detail doxygen comments
 *
 */
#ifndef MBED_MULTIGAS_SENSOR_H
#define MBED_MULTIGAS_SENSOR_H

#include "mbed.h"

//#define _DEBUG

#define SLAVE_ADDRESS_MiCS6814  (0x04<<1)

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


#ifdef _DEBUG
extern Serial pc;
#define DEBUG_PRINT(...) pc.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

enum GAS_TYPE{NH3, CO, NO2, C3H8, C4H10, CH4, H2, C2H5OH};
enum READ_ERROR_TYPE{READ_OK, NULL_RETURN_BUFFER_PTR, WRITE_LENGTH_ERROR, READ_LENGTH_ERROR, CHECKSUM_ERROR};

/**
 *  @class   MiCS6814_GasSensor
 *  @brief   A mbed component library to measure gas data by using Grove - Multichannel Gas Sensor (seeed)
 *  Please refer the details of the sensor. http://wiki.seeed.cc/Grove-Multichannel_Gas_Sensor/
 *
 */

class MiCS6814_GasSensor{

public:
    /** Create a MiCS6814_GasSensor instance
     *  the sensor is connected to specified I2C pins with specified address
     *  1. create an I2C instance
     *  2. initialize private variables.
     *
     * @param[in] sda I2C-bus SDA pin
     * @param[in] scl I2C-bus SCL pin
     * @param[in] slave_adr (option) I2C-bus address (default: 0x04<<1)
     * @par sample code (K64F)
     *  MiCS6814_GasSensor gas(I2C_SDA, I2C_SCL);
     */
    MiCS6814_GasSensor(PinName sda, PinName sck, char slave_adr = SLAVE_ADDRESS_MiCS6814);

    /** Create a MiCS6814_GasSensor instance
     *  the sensor is connected to specified I2C pins with specified address
     *  1. pass an I2C instance,
     *  2. then initialize private variables.
     *
     * @param[in] i2c_obj I2C object (instance)
     * @param[in] slave_adr (option) I2C-bus address (default: 0x04<<1)
     * @par sample code (K64F)
     *  I2C MySlave(I2C_SDA, I2C_SC);
     *  MiCS6814_GasSensor gas(MySlave);
     */
    MiCS6814_GasSensor(I2C &i2c_obj, char slave_adr = SLAVE_ADDRESS_MiCS6814);

    /** Destructor of MiCS6814_GasSensor
     *  1. Power off heater
     *  2. Release allocated heap memory.
     */
    virtual ~MiCS6814_GasSensor();

    /** Initialize MiCS6814_GasSensor
     *  1. Read firmware version from sensor
     *  2. Power on heater
     *  3. Clear private variables
     */
    void initialize(void);

    /** Return a specific measured value, unit: ppm
     *
     *  @param[in] gas_type one of gas type defined at enum GAS_TYPE
     *  @return the measured concentration of specific gas type (ppm)
     *  @par sample code
     *   val = calcGas(C3H8);
     */
    float calcGas(const enum GAS_TYPE gas_type);

private:

    I2C  *_i2c_p;
    I2C  &_i2c;
    char _address;     //I2C address of this MCU
    /** Check firmware version of sensor
     *  only support 2
     */
    void CheckFirmwareVersion(void);

    /** Turn On/Off sensor heater
     *
     * @param[in] Heater_On_flag   True: Turn on,  False: Turn off
     */
    void HeaterPower(bool Heater_On_flag);
    /** Read 16-bit data from sensor
     *
     * @param[in] reg_addr  register adderss
     * @param[in] data_4reg  data for the register, it will be Zero in case no input data for above register.
     * @param[in] write_length  length for I2C writing, should be 1 or 2
     * @param[in] read_length  length for I2C reading, should be 2 or 4
     * @param[out] pointer for read result buffer 16-bit
     * @return status
     */
    READ_ERROR_TYPE read_Multibytes(unsigned char reg_addr, unsigned char data_4reg ,unsigned char write_length, unsigned char read_length, uint16_t *readBuf);

    /** Read A0_[x] of sensor
     *
     * @param[in] index  {0,1,2}
     * @return return A0_[x] for version 2
     */
    uint16_t readR0_A0(unsigned char _indix);

    /** Read An_[x] of sensor
     *
     * @param[in] index  {0,1,2}
     * @return return An_[x] for version 2
     */
    uint16_t readRs_An(unsigned char _indix);

};



#endif // MBED_MULTIGAS_SENSOR_H
