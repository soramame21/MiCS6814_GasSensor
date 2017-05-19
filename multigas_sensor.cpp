/**
 ******************************************************************************
 * @file    multigas_sensor.cpp
 * @author  Boting Ren
 * @version V1.0.0
 * @date    19 May 2017
 * @brief   MutichannelGasSensor class implementation
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
#include "mbed.h"
#include "multigas_sensor.h"

MutichannelGasSensor::MutichannelGasSensor(PinName sda, PinName scl, char slave_adr)
    :
    i2c_p(new I2C(sda, scl)), 
    i2c(*i2c_p),
    address(slave_adr)
{
    initialize();
}

MutichannelGasSensor::MutichannelGasSensor(I2C &i2c_obj, char slave_adr)
    :
    i2c_p(NULL), 
    i2c(i2c_obj),
    address(slave_adr)
{
    initialize();
}

MutichannelGasSensor::~MutichannelGasSensor()
{
	HeaterPower(false);    // turn off the heater
	if (NULL != i2c_p)
        delete  i2c_p;
}

/**
 * read multi-byte data
 */
int MutichannelGasSensor::read_Multibytes(unsigned char addr_reg, unsigned char __dta, unsigned char write_len, unsigned char read_len)
{
    char cmd[2], val[4];
    int ret=-1, n=-1;
    uint8_t checksum=0;

    if(write_len>2 || write_len<1) return -2;      // write_len is wrong
    if (read_len!=2 && read_len!=4) return -3;    // read_len is wrong

    cmd[0] = addr_reg;
    if (write_len==2)
        cmd[1] = __dta;
    n=i2c.write(address, cmd, write_len);
    if (n!=0)
        printf("!! A-01 i2c.write failed read_Multibytes! return n= %d, write_len: %x, cmd[0]: 0x%x, __dta: 0x%x\r\n", n, write_len, cmd[0], __dta );

    n=i2c.read(address, val, read_len);
    if (n!=0)
        printf("!! A-02 i2c.read failed-1 read_Multibytes! return n= %d, read_len: %x, addr_reg: 0x%x, __dta: 0x%x\r\n", n, read_len, addr_reg, __dta );

    if (read_len==2){
        ret = (uint16_t) (val[0]<<8 | val[1]) ;
    if(ret==0)  printf("A-03   read_Multibytes, ret=0, addr_reg: 0x%x, write_len: %x, read_len: %x\r\n", addr_reg, write_len, read_len );
//printf("!!!!!!   val[3]: 0x%x,   0x%x,   0x%x\r\n", val[0], val[1], val[2]);
        switch(addr_reg)
        {
            case CH_VALUE_NH3:
            case CH_VALUE_CO:
            case CH_VALUE_NO2:
                if(ret > 0)
                    adcValR0[addr_reg-1] = ret;
                else
                    ret = adcValR0[addr_reg-1];
                break;
            default:;
        }

    }else if (read_len==4){
    	checksum = (uint8_t)(val[0] + val[1] + val[2]);

        if(checksum != val[3]){
        	printf("!! A-3 checksum failed,   val[4]: 0x%x,   0x%x,   0x%x,   0x%x, checksum: 0x%x\r\n", val[0], val[1], val[2], val[3], checksum );
        	printf("!! A-4 failed i2c.read, addr_reg: 0x%x, write_len: %x, read_len: %x\r\n", addr_reg, write_len, read_len );
        	return -4;    //checksum is wrong
        }
        ret = (uint16_t) ((val[1] << 8) + val[2]);
    }

    return ret;
}

unsigned char MutichannelGasSensor::readVersion()
{
	if(read_Multibytes(CMD_READ_EEPROM, ADDR_IS_SET, 2, 2) == 1126)        // read firmware version
        __version = 2;
    else    __version = 1;
    printf("version = %d\r\n", __version);
    return __version;

}

/*********************************************************************************************************
** Function name:           powerOn / Off
** Descriptions:            power on sensor heater
*********************************************************************************************************/
void MutichannelGasSensor::HeaterPower(bool kk)
{
	const char v1tmp[2]={0x20, 0x21};
	const char v2tmp[2]={0, 1};
	char tmp[4];
	int n=-1;
    printf("Sensor heater Power :%s\r\n", kk? "On":"Off");
    if(__version == 1){
    	int n;
    	tmp[0] = v1tmp[kk];
    	n=i2c.write(address, tmp, 1);
    	if (n!=0)
            printf("!1 A-11 i2c.write failed V1, return n= %d, addr: %x, tmp[0]: 0x%x\r\n", n, address, tmp[0] );
    }else if(__version == 2)
    {
    	tmp[0] = 11;
    	tmp[1] = v2tmp[kk];
    	n=i2c.write(address, tmp, 2);
    	if (n!=0)
            printf("!! A-12 i2c.write failed V2, return n= %d, addr: %x, tmp[0]: 0x%x, tmp[1]: 0x%x\r\n", n, address, tmp[0], tmp[1] );
    }
}


/*********************************************************************************************************
** Function name:           readR0
** Descriptions:            read R0 stored in slave MCU
*********************************************************************************************************/
int MutichannelGasSensor::readR0(void)
{
    int rtn = -1;

    for(int k=0; k<CH_VALUE_NO2; k++){
        rtn = read_Multibytes(0x11+k, 0, 1, 4);
        if(rtn > 0)      R0[k] = rtn;
        else
        {
        	printf("!! A-21 read R0[%d] failed, 0x%x, rtn = %d\r\n",k, 0x11+k, rtn);
        	return rtn;         //unsuccessful
        }
    }

    printf("** A-22  READ R0[0-3] Succeeded  %d,   %d,   %d\r\n", R0[0], R0[1], R0[2]);

    return 1;//successful
}

/*********************************************************************************************************
** Function name:           readRs
** Descriptions:            read resistance value of each channel from slave MCU
*********************************************************************************************************/
int MutichannelGasSensor::readRs(void)
{
    int rtn = -1;

    for(int k=0; k<CH_VALUE_NO2; k++){
    	rtn = read_Multibytes(0x01+k, 0, 1, 4);
        if(rtn >= 0)      Rs[k] = rtn;
        else
        {
        	printf("!! A-31 read Rs[%d] failed, 0x%x, rtn = %d\r\n",k, 0x01+k, rtn);
        	return rtn;         //unsuccessful
        }
    }
    printf("A-32 read Rs[3], %d,   %d,   %d\\r\n", Rs[0], Rs[1], Rs[2]);
    return 1;//successful
}

/*********************************************************************************************************
** Function name:           calcGas
** Descriptions:            calculate gas concentration of each channel from slave MCU
** Parameters:
                            gas - gas type
** Returns:
                            float value - concentration of the gas
*********************************************************************************************************/
float MutichannelGasSensor::calcGas(int gas)
{

//    float ratio0=0.0f, ratio1=0.0f, ratio2=0.0f;
    int rtn=-1, A0_[3], An_[3];
    const unsigned char A0_table[3]={ADDR_USER_ADC_HN3, ADDR_USER_ADC_CO, ADDR_USER_ADC_NO2};
    const unsigned char An_table[3]={CH_VALUE_NH3, CH_VALUE_CO, CH_VALUE_NO2};

    int passtm=0;
    passtm=memo.read_ms();  //printf("passtm= %d  ", passtm);
    if(Ratio[0]==0.0f || passtm>CATCH_TIME){
        if(1 == __version)
        {
    	    rtn=readRs();
            if(rtn < 0){
                printf("!! A-41 readRs() failed, rtn = %d\r\n", rtn);
                return -2.0f;
            }
            rtn=readR0();
            if(rtn <= 0){
                printf("!! A-42 readR0() failed, rtn = %d\r\n", rtn);
                return -3.0f;
            }
            for (int p=0; p<CH_VALUE_NO2; p++)
                Ratio[p] = (float)Rs[p] / R0[p];
            printf("A-43 ratio x ; %f, %f, %f\r\n", Ratio[0], Ratio[1], Ratio[2]);
        }
        else if(2 == __version)
        {
            // how to calc ratio/123
            for (int p=0; p<CH_VALUE_NO2; p++)
                A0_[p] = read_Multibytes(CMD_READ_EEPROM, A0_table[p], 2, 2);

            printf("A-51 A0_[0]: %d, A0_[1]: %d, A0_[2]: %d\r\n", A0_[0], A0_[1], A0_[2]);
            for (int p=0; p<CH_VALUE_NO2; p++)
                An_[p] = read_Multibytes(An_table[p], 0, 1, 2);
            printf("A-52 An_[0]: %d, An_[1]: %d, An_[2]: %d\r\n", An_[0], An_[1], An_[2]);

            for (int p=0; p<CH_VALUE_NO2; p++)
                Ratio[p] = (float)An_[p]/(float)A0_[p]*(1023.0-A0_[p])/(1023.0-An_[p]);
            printf("A-53 ratio x ; %f, %f, %f\r\n", Ratio[0], Ratio[1], Ratio[2]);
        }
        memo.reset();
    }
    float c = 0;

    switch(gas)
    {
        case CO:
        {
            c = pow(Ratio[1], -1.179f)*4.385f;  //mod by jack
            break;
        }
        case NO2:
        {
            c = pow(Ratio[2], 1.007f)/6.855f;  //mod by jack
            break;
        }
        case NH3:
        {
            c = pow(Ratio[0], -1.67f)/1.47f;  //modi by jack
            break;
        }
        case C3H8:  //add by jack
        {
            c = pow(Ratio[0], -2.518f)*570.164f;
            break;
        }
        case C4H10:  //add by jack
        {
            c = pow(Ratio[0], -2.138f)*398.107f;
            break;
        }
        case CH4:  //add by jack
        {
            c = pow(Ratio[1], -4.363f)*630.957f;
            break;
        }
        case H2:  //add by jack
        {
            c = pow(Ratio[1], -1.8f)*0.73f;
            break;
        }
        case C2H5OH:  //add by jack
        {
            c = pow(Ratio[1], -1.552f)*1.622f;
            break;
        }
        default:
            break;
    }

    return isnan(c)?-3:c;
}



void MutichannelGasSensor::initialize()
{

    readVersion();
    HeaterPower(true);   // turn on the heater.
    for(int j=0; j<CH_VALUE_NO2; j++){
        adcValR0[j]=0;
        R0[j]=Rs[j]=0;
        Ratio[j]=0.0f;
    }
    memo.start();
}
 
