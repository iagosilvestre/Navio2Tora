

#include <Common/MS5611.h>
#include <Common/MS5611.cpp>
#include "Common/I2Cdev.h"
#include "Common/I2Cdev.cpp"
#include <Common/Util.h>
#include <Common/Util.cpp>
#include <unistd.h>
#include <stdio.h>

#include <string>
#include <memory>
#include <stdint.h>
#include <sys/time.h>
#include <iostream>
#include <vector>

unsigned long int dtlong=0,auxCount=0,ledCount=0,count=0,countMax=25000;
float temperatura,pressao;
std::vector<int> baroData;
std::vector<int> baroData2;

int main()
{
    MS5611 barometer;
    struct timeval t0, t1, dt;
    barometer.initialize();

    while (count<5000) {
    	gettimeofday(&t0, NULL);
        barometer.refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer.readPressure();

        barometer.refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer.readTemperature();

        barometer.calculatePressureAndTemperature();

        temperatura=barometer.getTemperature();
		pressao=barometer.getPressure();

        gettimeofday(&t1, NULL);
        timersub(&t1, &t0, &dt);
		baroData.push_back(dt.tv_usec-20000);
        printf("Temperature(C): %f Pressure(millibar): %f\n",barometer.getTemperature(), barometer.getPressure());
        count++;
        usleep(10000);
    }

  //----------------Escreve os dados adquiridos em barometer.txt--------------------------------------///
    //---------------------------------------------------------------------------------------------//
	FILE *fBaro = fopen("barometerF.txt", "w");
	fprintf(fBaro, "count;dtBaro\n");
	fclose(fBaro);
	for (std::vector<int>::iterator it = baroData.begin() ; it != baroData.end(); ++it){
		auxCount++;
		FILE *fBaro = fopen("barometerF.txt", "a");
		fprintf(fBaro, "%d;%d\n",auxCount,*it);
		fclose(fBaro);
	}

    return 0;
}
