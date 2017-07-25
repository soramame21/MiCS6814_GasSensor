/**
 ******************************************************************************
 * @file    MiCS6814_GasSensor.h
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
 */

/**
 *  Library for "MiCS-6814 Multichannel Gas Sensor" from Switch Science
 *    https://www.switch-science.com/catalog/2512/
 *
 *  For more information about the Multichannel Gas Sensor:
 *    http://wiki.seeed.cc/Grove-Multichannel_Gas_Sensor/
 */


#ifndef MBED_MULTIGAS_SENSOR_H
#define MBED_MULTIGAS_SENSOR_H

#include "mbed.h"

#define SLAVE_ADDRESS_MiCS6814  (0x04<<1)

#define ADDR_IS_SET             0
#define ADDR_FACTORY_ADC_NH3    2
#define ADDR_FACTORY_ADC_CO     4
#define ADDR_FACTORY_ADC_NO2    6

#define ADDR_USER_ADC_HN3       8
#define ADDR_USER_ADC_CO        10
#define ADDR_USER_ADC_NO2       12
#define ADDR_IF_CALI            14

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

#define HEATER_ON               1
#define HEATER_OFF              0

#if !defined(I2C_SDA) && defined(SDA)
#define I2C_SDA SDA
#endif

#if !defined(I2C_SCL) && defined(SCL)
#define I2C_SCL SCL
#endif

#ifdef _DEBUG
extern Serial pc;
#define DEBUG_PRINT(...) pc.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

enum GAS_TYPE{NH3, CO, NO2, C3H8, C4H10, CH4, H2, C2H5OH};
enum READ_ERROR_TYPE{READ_OK, NULL_RETURN_BUFFER_PTR, WRITE_LENGTH_ERROR, READ_LENGTH_ERROR, CHECKSUM_ERROR};

/**  Interface for controlling MiCS-6814 Multichannel Gas Sensor
 *
 * @code
 * #include "mbed.h"
 * #include "MiCS6814_GasSensor.h"
 *
 * Serial pc(USBTX, USBRX);
 *
 * #if defined(TARGET_LPC1768)
 * MiCS6814_GasSensor sensor(p28, p27);
 * #else
 * MiCS6814_GasSensor sensor(I2C_SDA, I2C_SCL);
 * #endif
 *
 * int main() {
 *
 *     while(1) {
 *         pc.printf("NH3: %.2f ppm, CO: %.2f ppm, NO2: %.2f ppm, C3H8: %.2f ppm \r\n", sensor.getGas(NH3), sensor.getGas(CO), sensor.getGas(NO2), sensor.getGas(C3H8));
 *         pc.printf("C4H10: %.2f ppm, CH4: %.2f ppm, H2: %.2f ppm, C2H5OH: %.2f ppm \r\n", sensor.getGas(C4H10), sensor.getGas(CH4), sensor.getGas(H2), sensor.getGas(C2H5OH));
 *         wait(1);
 *     }
 * }
 *
 * @endcode
 */

/**
 *  @class   MiCS6814_GasSensor
 *  @brief   A mbed component library to measure concentration value for 8 type of gases by using MiCS6814 - Multichannel Gas Sensor (seeed)
 *
 */

class MiCS6814_GasSensor{

public:
    /** Create a MiCS6814_GasSensor instance
     *  the sensor is connected to specified I2C pins with specified address
     *
     * @param[in] sda I2C-bus SDA pin
     * @param[in] scl I2C-bus SCL pin
     * @param[in] slave_adr (option) I2C-bus address (default: 0x04<<1)
     */
    MiCS6814_GasSensor(PinName sda, PinName scl, char slave_adr = SLAVE_ADDRESS_MiCS6814);

    /** Create a MiCS6814_GasSensor instance
     *  the sensor is connected to specified I2C pins with specified address
     *
     * @param[in] i2c_obj I2C object (instance)
     * @param[in] slave_adr (option) I2C-bus address (default: 0x04<<1)
     */
    MiCS6814_GasSensor(I2C &i2c_obj, char slave_adr = SLAVE_ADDRESS_MiCS6814);

    /** Destructor of MiCS6814_GasSensor
     */
    virtual ~MiCS6814_GasSensor();

    /** Initialize MiCS6814_GasSensor
     *  Read firmware version from sensor and power on heater
     */
    void initialize(void);

    /** Return a specific measured value, unit: ppm
     *
     *  @param[in] gas_type one of gas type defined at enum GAS_TYPE
     *  @return the measured concentration of specific gas type (ppm)
     */
    float getGas(const enum GAS_TYPE gas_type);

private:

    I2C  *_i2c_p;
    I2C  &_i2c;
    char _address;       //I2C address of this MCU

    /** Check firmware version of sensor
     *  only support version 2
     */
    void CheckFirmwareVersion(void);

    /** Turn On/Off sensor heater
     *
     * @param[in] Switch_On   1: Turn on,  0: Turn off
     */
    void HeaterPower(const unsigned char Switch_On);

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
     * @return return A0_[x]
     */
    uint16_t readR0_A0(unsigned char index);

    /** Read An_[x] of sensor
     *
     * @param[in] index  {0,1,2}
     * @return return An_[x]
     */
    uint16_t readRs_An(unsigned char index);
};

#endif // MBED_MULTIGAS_SENSOR_H
