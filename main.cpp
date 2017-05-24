#include "BME280.h"
#include <iostream>

using namespace std;

int main(void)
{
    BME280 czujnik;

    unsigned char sleep         = 0x74;
    unsigned char dataHum       = 0x72;
    unsigned char dataTemp      = 0x74;
    unsigned char dataConfig    = 0x75;

    czujnik.writeRegister(sleep,0x00);
    czujnik.writeRegister(dataHum,0x02);
    czujnik.writeRegister(dataTemp,0x27);
    czujnik.writeRegister(dataConfig,0x90);

    return 0;
}
