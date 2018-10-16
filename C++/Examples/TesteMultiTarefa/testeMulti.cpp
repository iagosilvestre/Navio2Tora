/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
inertial measurement unit: MPU9250 or LSM9DS1 over SPI on Raspberry Pi + Navio.

Navio's onboard sensors are connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1 (MPU9250), /dev/spidev0.3 (acc/gyro LSM9DS1)
and /dev/spidev0.2 (mag LSM9DS1).

To run this example navigate to the directory containing it and run following commands:
make
./AccelGyroMag -i [sensor name]
Sensors names: mpu is MPU9250, lsm is LSM9DS1.
For print help:
./AccelGyroMag -h
*/
#include <Navio2/Led_Navio2.h>
//#include <Common/Ublox.h>
#include <Common/MS5611.h>
#include <string>
#include <stdio.h>
#include <memory>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include <Common/Util.h>
#include <pthread.h>
#include <iostream>
#include <vector>
//#include <mutex>
// std::cout
// std::thread, std::this_thread::sleep_for




	    float ax, ay, az;
	    float gx, gy, gz;
	    float mx, my, mz;
	    float ax2, ay2, az2;
	    float gx2, gy2, gz2;
	    float mx2, my2, mz2;

	    struct timespec baro1,baro2,mpu1,mpu2,lsm1,lsm2,led1,led2,tot1,tot2;
		float dt;
		unsigned long int dtlong=0,auxCount=0,ledCount=0,count=0,countMax=25000;
		double dtMPU=0,dtLSM=0,dtLED=0,dtBaro=0,dtTot=0;


	    float temperatura,pressao;
		std::vector<int> baroData;
		std::vector<int> mpuData;
		std::vector<int> lsmData;
		std::vector<int> ledData;
		std::vector<int> totData;

using namespace std;

std::unique_ptr <Led> get_led()
{
        auto ptr = std::unique_ptr <Led>{ new Led_Navio2() };
        return ptr;
}


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
void * acquireMPUData(void * imuMPU)
{
	struct timeval t0, t1, dt;
	MPU9250* mpu=(MPU9250*)imuMPU;
	while(count<countMax){
		gettimeofday(&t0, NULL);
		mpu->update();
		mpu->read_accelerometer(&ax, &ay, &az);
		mpu->read_gyroscope(&gx, &gy, &gz);
		mpu->read_magnetometer(&mx, &my, &mz);
		gettimeofday(&t1, NULL);
		timersub(&t1, &t0, &dt);
		mpuData.push_back(dt.tv_usec);
		usleep(5000);
	}
	pthread_exit(NULL);
}
void * acquireLSMData(void * imuLSM)
{
	struct timeval t0, t1, dt;
	LSM9DS1* lsm=(LSM9DS1*)imuLSM;
	while(count<countMax){
		gettimeofday(&t0, NULL);
		lsm->update();
		lsm->read_accelerometer(&ax2, &ay2, &az2);
		lsm->read_gyroscope(&gx2, &gy2, &gz2);
		lsm->read_magnetometer(&mx2, &my2, &mz2);
		gettimeofday(&t1, NULL);
		timersub(&t1, &t0, &dt);
		lsmData.push_back(dt.tv_usec);
		usleep(5000);
	}
	pthread_exit(NULL);
}

void * acquireLedData(void * led)
{
	struct timeval t0, t1, dt;
	Led_Navio2* diode=(Led_Navio2*)led;
	while(count<countMax){
		gettimeofday(&t0, NULL);
		ledCount++;
    	if((ledCount%2)==0){
    		diode->setColor(Colors::Red);
    	}
    	else{
    		diode->setColor(Colors::Green);
    	}
    	gettimeofday(&t1, NULL);
		timersub(&t1, &t0, &dt);
		ledData.push_back(dt.tv_usec);
		usleep(500000);
	}

	pthread_exit(NULL);
}
std::unique_ptr <InertialSensor> get_inertial_sensor( std::string sensor_name)
{
    if (sensor_name == "mpu") {
        printf("Selected: MPU9250\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
    }
    else {
        return NULL;
    }
}
void print_help()
{
    printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
    printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
}

std::string get_sensor_name(int argc, char *argv[])
{
    if (get_navio_version() == NAVIO2) {

        if (argc < 2) {
            printf("Enter parameter\n");
            print_help();
            return std::string();
        }

        // prevent the error message
        opterr = 0;
        int parameter;

        while ((parameter = getopt(argc, argv, "i:h")) != -1) {
            switch (parameter) {
            case 'i': if (!strcmp(optarg,"mpu") ) return "mpu";
                            else return "lsm";
            case 'h': print_help(); return "-1";
            case '?': printf("Wrong parameter.\n");
                      print_help();
                      return std::string();
            }
        }

    } else { //sensor on NAVIO+

        return "mpu";
    }

}
//=============================================================================
int main(int argc, char *argv[])
{
	unsigned long int min=0,max=0,mem=0,media=0,sum=0;
	if (check_apm()) {
	        return 1;
	    }

	Led_Navio2 led;
	MS5611 baro;
	MPU9250 imuMPU;
	LSM9DS1 imuLSM;

	pthread_t baro_thread;
	pthread_t MPU_thread;
	pthread_t LSM_thread;
	pthread_t led_thread;

	baro.initialize();
	    if(pthread_create(&baro_thread, NULL, acquireBarometerData, (void *)&baro))
	    {
	        printf("Error: Failed to create barometer thread\n");
	        return 0;
	    }
	imuLSM.initialize();
			if(pthread_create(&LSM_thread, NULL, acquireLSMData, (void *)&imuLSM))
				{
					printf("Error: Failed to create lsm thread\n");
						return 0;
			}
	imuMPU.initialize();
		if(pthread_create(&MPU_thread, NULL, acquireMPUData, (void *)&imuMPU))
		    {
		        printf("Error: Failed to create mpu thread\n");
		        return 0;
		    }
	led.initialize();
	if(pthread_create(&led_thread, NULL, acquireLedData, (void *)&led))
				{
					printf("Error: Failed to create led thread\n");
						return 0;
			}

    while(count<countMax) {
    	count++;
//----------------Obtencao da leitura dos sensores---------------------------------//
    	usleep(10000);

    }
	FILE *fBaro = fopen("barometer.txt", "w");
	fprintf(fBaro, "count;dtBaro\n");
	fclose(fBaro);
	for (std::vector<int>::iterator it = baroData.begin() ; it != baroData.end(); ++it){
		auxCount++;
		FILE *fBaro = fopen("barometer.txt", "a");
		fprintf(fBaro, "%d;%d\n",auxCount,*it);
		fclose(fBaro);
	}


	auxCount=0;
	FILE *fMPU = fopen("mpu.txt", "w");
	fprintf(fMPU, "count;dtMPU\n");
	fclose(fMPU);
	for (std::vector<int>::iterator it = mpuData.begin() ; it != mpuData.end(); ++it){
		auxCount++;
		FILE *fMPU = fopen("mpu.txt", "a");
		fprintf(fMPU, "%d;%lu\n",auxCount,*it);
		fclose(fMPU);
	}


	auxCount=0;
	FILE *fLSM = fopen("lsm.txt", "w");
	fprintf(fLSM, "count;dtLSM\n");
	fclose(fLSM);
	for (std::vector<int>::iterator it = lsmData.begin() ; it != lsmData.end(); ++it){
		auxCount++;
		FILE *fLSM = fopen("lsm.txt", "a");
		fprintf(fLSM, "%d;%lu\n",auxCount,*it);
		fclose(fLSM);
	}


	auxCount=0;
	FILE *fLed = fopen("led.txt", "w");
	fprintf(fLed, "count;dtLed\n");
	fclose(fLed);
	for (std::vector<int>::iterator it = ledData.begin() ; it != ledData.end(); ++it){
		auxCount++;
		FILE *fLed = fopen("led.txt", "a");
		fprintf(fLed, "%d;%lu\n",auxCount,*it);
		fclose(fLed);
	}

	auxCount=0;
		FILE *fTot = fopen("dtTot.txt", "w");
		fprintf(fTot, "count;dtTot\n");
		fclose(fTot);
		for (std::vector<int>::iterator it = totData.begin() ; it != totData.end(); ++it){
			auxCount++;
			FILE *fTot = fopen("dtTot.txt", "a");
			fprintf(fTot, "%d;%lu\n",auxCount,*it);
			fclose(fTot);
		}

		time_t rawtime;
		struct tm * timeinfo;

		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		printf ( "Data e tempo local atual: %s", asctime (timeinfo) );
		printf("-----------------------------------Leitura da IMU MPU9250-----------------------------------------");
				printf("\n\nAcc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
				printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
				printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);
		printf("-----------------------------------Leitura da IMU LSM9DS1-----------------------------------------");
				printf("\n\nAcc: %+7.3f %+7.3f %+7.3f  ", ax2, ay2, az2);
				printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx2, gy2, gz2);
				printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx2, my2, mz2);
		printf("-----------------------------------Leitura do barometro-------------------------------------------");
		printf("\nTemperatura(C): %f Pressao (milibar): %f\n",
								temperatura, pressao);
		pthread_exit(NULL);
		pthread_exit(NULL);
		pthread_exit(NULL);
		pthread_exit(NULL);
           return 0;
       }

