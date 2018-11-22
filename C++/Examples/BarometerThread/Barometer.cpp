

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
#include <pthread.h>

unsigned long int dtlong=0,auxCount=0,ledCount=0,count=0,countMax=25000;
float temperatura,pressao;
std::vector<int> baroData;
std::vector<int> baroData2;


void * acquireBarometerData(void * barom)
{
	struct timeval t0, t1, dt;
	//unsigned long int previoustime=0, currenttime=0;
    MS5611* barometer = (MS5611*)barom;
    while (count<countMax) {
    	gettimeofday(&t0, NULL);
        barometer->refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer->readPressure();

        barometer->refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer->readTemperature();

        barometer->calculatePressureAndTemperature();

        temperatura=barometer->getTemperature();

        pressao=barometer->getPressure();
        gettimeofday(&t1, NULL);
		timersub(&t1, &t0, &dt);
        baroData.push_back(dt.tv_usec-20000);

        //usleep(5000);
    }

    pthread_exit(NULL);
}


int main()
{
	MS5611 baro;

//    struct timeval t0, t1, dt;
    pthread_t baro_thread;
    baro.initialize();
//    	  pthread_create(&baro_thread, NULL, acquireBarometerData, (void *)&baro);
//    	    {
//    	        printf("Error: Failed to create barometer thread\n");
//    	        return 0;
//    	    }
    while (count<5000) {
//    	gettimeofday(&t0, NULL);
//        barometer.refreshPressure();
//        usleep(10000); // Waiting for pressure data ready
//        barometer.readPressure();
//
//        barometer.refreshTemperature();
//        usleep(10000); // Waiting for temperature data ready
//        barometer.readTemperature();
//
//        barometer.calculatePressureAndTemperature();
//
//        temperatura=barometer.getTemperature();
//		pressao=barometer.getPressure();
//
//        gettimeofday(&t1, NULL);
//        timersub(&t1, &t0, &dt);
//		baroData.push_back(dt.tv_usec-20000);
//        printf("Temperature(C): %f Pressure(millibar): %f\n",barometer.getTemperature(), barometer.getPressure());
        count++;
        usleep(10000);
    }

  //----------------Escreve os dados adquiridos em barometer.txt--------------------------------------///
    //---------------------------------------------------------------------------------------------//
	FILE *fBaro = fopen("barometerTF.txt", "w");
	fprintf(fBaro, "count;dtBaro\n");
	fclose(fBaro);
	for (std::vector<int>::iterator it = baroData.begin() ; it != baroData.end(); ++it){
		auxCount++;
		FILE *fBaro = fopen("barometerTF.txt", "a");
		fprintf(fBaro, "%d;%d\n",auxCount,*it);
		fclose(fBaro);
	}

    return 0;
}
