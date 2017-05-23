/**
 ******************************************************************************
 * @file    multigas_sensor.cpp
 * @author  Boting Ren
 * @version V1.0.1
 * @date    22 May 2017
 * @brief   MiCS6814_GasSensor class implementation
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
 * 1. renamed Class name and added prefix for several private variables
 * 2. removed printf output
 * 3. removed code working for version 1 firmware
 * 4. replaced class private variables by using local variable in stack area
 * 5. removed removed timer and Cache feature
 * 6. other small improvements
 *
 */
#include "mbed.h"
#include "multigas_sensor.h"

MiCS6814_GasSensor::MiCS6814_GasSensor(PinName sda, PinName scl, char slave_adr)
    :
    _i2c_p(new I2C(sda, scl)),
    _i2c(*_i2c_p),
    _address(slave_adr)
{
    initialize();
}

MiCS6814_GasSensor::MiCS6814_GasSensor(I2C &i2c_obj, char slave_adr)
    :
    _i2c_p(NULL),
    _i2c(i2c_obj),
    _address(slave_adr)
{
    initialize();
}

MiCS6814_GasSensor::~MiCS6814_GasSensor()
{
    HeaterPower(false);    // turn off the heater
    if (NULL != _i2c_p)
    {
        delete  _i2c_p;
    }
}


READ_ERROR_TYPE MiCS6814_GasSensor::read_Multibytes(unsigned char addr_reg, unsigned char __dta, unsigned char write_len, unsigned char read_len, uint16_t * ret_val)
{
    char cmd[2], val[4];     // write buf, read buf
    uint8_t checksum=0;

    if (ret_val==NULL) {
        return NULL_RETURN_BUFFER_PTR;     // ret_val is a NULL pointer
    }
    if(write_len>2 || write_len<1) {
        return WRITE_LENGTH_ERROR;         // write_len is wrong
    }
    if (read_len!=2 && read_len!=4) {
        return READ_LENGTH_ERROR;          // read_len is wrong
    }
    *ret_val=0;     // clear return buf

    cmd[0] = addr_reg;
    if (write_len==2) {
        cmd[1] = __dta;
    }
    _i2c.write(_address, cmd, write_len);

    _i2c.read(_address, val, read_len);

    if (read_len==2) {
        *ret_val = (uint16_t) (val[0]<<8 | val[1]) ;   // set return value to caller
        #ifdef _DEBUG
        if(*ret_val==0)  DEBUG_PRINT("read_Multibytes, *ret_val=0, addr_reg: 0x%x, write_len: %x, read_len: %x\r\n", addr_reg, write_len, read_len );
        #endif
    }else if (read_len==4) {
        checksum = (uint8_t)(val[0] + val[1] + val[2]);
        if(checksum != val[3]){
            DEBUG_PRINT("checksum failed,   val[4]: 0x%x,   0x%x,   0x%x,   0x%x, checksum: 0x%x\r\n", val[0], val[1], val[2], val[3], checksum );
            DEBUG_PRINT("checksum failed i2c.read, addr_reg: 0x%x, write_len: %x, read_len: %x\r\n", addr_reg, write_len, read_len );
            return CHECKSUM_ERROR;    //checksum is wrong
        }
        *ret_val = (uint16_t) ((val[1] << 8) + val[2]);
    }
    return READ_OK;
}

void MiCS6814_GasSensor::CheckFirmwareVersion()
{
    uint16_t readBuf=0;
    unsigned char ret_val = read_Multibytes(CMD_READ_EEPROM, ADDR_IS_SET, 2, 2, &readBuf);      // read firmware version
    // only support version = 2
    MBED_ASSERT(ret_val == READ_OK && readBuf == 1126);
}


void MiCS6814_GasSensor::HeaterPower(bool Switch_On)
{
    const unsigned char v2tmp[2]={0, 1};        // On, Off command for version 2
    char tmp[4];
    DEBUG_PRINT("Sensor heater Power :%s\r\n", Switch_On? "On":"Off");
    tmp[0] = 11;
    tmp[1] = v2tmp[Switch_On];
    _i2c.write(_address, tmp, 2);
}

uint16_t MiCS6814_GasSensor::readR0_A0(unsigned char _indix)
{
    uint16_t readBuf=0;
    const unsigned char A0_table[3]={ADDR_USER_ADC_HN3, ADDR_USER_ADC_CO, ADDR_USER_ADC_NO2};
    unsigned char ret_val;
    //  version 2 : read A0_[_indix]
    ret_val= read_Multibytes(CMD_READ_EEPROM, A0_table[_indix], 2, 2, &readBuf);
    DEBUG_PRINT("A0_[%d]: %d,    ", _indix, readBuf);
    if (ret_val == READ_OK) {
        return readBuf;
    }
    // read failed, Rs[_indix]
    DEBUG_PRINT("A0_[%d] read error: %d\r\n", _indix, ret_val);
    return 0;
}


uint16_t MiCS6814_GasSensor::readRs_An(unsigned char _indix)
{
    uint16_t readBuf=0;
    const unsigned char An_table[3]={CH_VALUE_NH3, CH_VALUE_CO, CH_VALUE_NO2};
    unsigned char ret_val;
    //  version 2 : read An_[_indix]
    ret_val= read_Multibytes(An_table[_indix], 0, 1, 2, &readBuf);
    DEBUG_PRINT("An_[%d]: %d,    ", _indix, readBuf);
    if (ret_val == READ_OK) {
        return readBuf;
    }
    // read failed, Rs[_indix]
    DEBUG_PRINT("An_[%d] read error: %d\r\n", _indix, ret_val);
    return 0;
}


float MiCS6814_GasSensor::calcGas(const GAS_TYPE gas_type)
{
    int A0_[3], An_[3];
    const unsigned char GasType_2index[8] ={/* NH3 */ 0,/* CO */ 1,/* NO2 */ 2,/*C3H8 */ 0,/* C4H10 */ 0,/* CH4 */ 1,/* H2 */ 1,/* C2H5OH */ 1};
    unsigned char _index = GasType_2index[gas_type];
    float Ratio[3];        //will be calculated. ration 0,1,2

    // prepare necessary Ratio[x] according to gas_type
    An_[_index] = readRs_An(_index);      // read An_[x]
    A0_[_index] = readR0_A0(_index);      // read R0[x]
    Ratio[_index] = (float)An_[_index]/(float)A0_[_index]*(1023.0-A0_[_index])/(1023.0-An_[_index]);
    DEBUG_PRINT("Ratio[%d]: %.3f,   ", _index, Ratio[_index]);

    float calcu_val = 0.0f;
    // calculate concentration value of specified gas_type
    switch(gas_type)
    {
        case CO:
        {
            calcu_val = pow(Ratio[1], -1.179f)*4.385f;
            break;
        }
        case NO2:
        {
            calcu_val = pow(Ratio[2], 1.007f)/6.855f;
            break;
        }
        case NH3:
        {
            calcu_val = pow(Ratio[0], -1.67f)/1.47f;
            break;
        }
        case C3H8:
        {
            calcu_val = pow(Ratio[0], -2.518f)*570.164f;
            break;
        }
        case C4H10:
        {
            calcu_val = pow(Ratio[0], -2.138f)*398.107f;
            break;
        }
        case CH4:
        {
            calcu_val = pow(Ratio[1], -4.363f)*630.957f;
            break;
        }
        case H2:
        {
            calcu_val = pow(Ratio[1], -1.8f)*0.73f;
            break;
        }
        case C2H5OH:
        {
            calcu_val = pow(Ratio[1], -1.552f)*1.622f;
            break;
        }
        default:
            break;
    }

    return calcu_val;
}



void MiCS6814_GasSensor::initialize()
{

    CheckFirmwareVersion();
    HeaterPower(true);   // turn on the heater.
}
