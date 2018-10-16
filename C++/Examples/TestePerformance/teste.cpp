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
#include <Navio+/Led_Navio.h>
#include <Common/Ublox.h>
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

#define G_SI 9.80665
#define PI   3.14159

using namespace std;

std::unique_ptr <Led> get_led()
{
    if (get_navio_version() == NAVIO2)
    {
        auto ptr = std::unique_ptr <Led>{ new Led_Navio2() };
        return ptr;
    } else
    {
        auto ptr = std::unique_ptr <Led>{ new Led_Navio() };
        return ptr;
    }
}


void * acquireBarometerData(void * barom)
{
    MS5611* barometer = (MS5611*)barom;
    while (true) {
        barometer->refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer->readPressure();

        barometer->refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer->readTemperature();

        barometer->calculatePressureAndTemperature();

        //sleep(0.5);
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

	if (check_apm()) {
	        return 1;
	    }
	auto led = get_led();
	if (!led->initialize())
	        return EXIT_FAILURE;

	MS5611 baro;
	pthread_t baro_thread;
	baro.initialize();
	    if(pthread_create(&baro_thread, NULL, acquireBarometerData, (void *)&baro))
	    {
	        printf("Error: Failed to create barometer thread\n");
	        return 0;
	    }
	std::vector<double> pos_data;
	Ublox gps;

    /*auto sensor_name = get_sensor_name(argc, argv);
    if (sensor_name.empty())
        return EXIT_FAILURE;*/

    auto sensor = get_inertial_sensor("mpu");
    auto sensor2 = get_inertial_sensor("lsm");


    if (!sensor) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }

    if (!sensor->probe()) {
        printf("Sensor not enabled\n");
        return EXIT_FAILURE;
    }
    sensor->initialize();
    sensor2->initialize();

	struct timeval tv,tv2;
	float dt;
	static unsigned long previoustime=0, currenttime=0,dtlong=0,count=0,min=0,max=0,mem=0,media=0,sum=0;
	
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    float ax2, ay2, az2;
    float gx2, gy2, gz2;
    float mx2, my2, mz2;

    float temperatura,pressao;
//-------------------------------------------------------------------------


    if(gps.testConnection())
        {
            printf("Ublox test OK\n");
            if (!gps.configureSolutionRate(1000))
            {
                printf("Setting new rate: FAILED\n");
            }
    while(1) {
    	count++;
//----------------Obtencao do tempo antes da leitura dos sensores---------------------------------//
    	//gettimeofday(&tv,NULL);

//----------------Escrita no PWM  ---------------------------------//
    	 gettimeofday(&tv,NULL);
    	led->setColor(Colors::Green);

//----------------Leitura da IMU MPU ---------------------------------//

        sensor->update();
        sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
        sensor->read_magnetometer(&mx, &my, &mz);


//----------------Leitura da IMU LSM---------------------------------//

        sensor2->update();
        sensor2->read_accelerometer(&ax2, &ay2, &az2);
        sensor2->read_gyroscope(&gx2, &gy2, &gz2);
        sensor2->read_magnetometer(&mx2, &my2, &mz2);

//----------------Leitura do barometro ---------------------------------//


        temperatura=baro.getTemperature();

        pressao=baro.getPressure();
        gettimeofday(&tv2,NULL);
//----------------Obtencao do tempo apos leitura dos dados ---------------------------------//
        if(count!=1){
        	 mem=dtlong;
        }
        //gettimeofday(&tv2,NULL);
        previoustime = 1000000 * tv.tv_sec + tv.tv_usec;
    	currenttime = 1000000 * tv2.tv_sec + tv2.tv_usec;
    	dtlong=currenttime-previoustime;
    	if(count==1){
    	    		min=dtlong;
    	    		max=dtlong;
    	    		mem=dtlong;
    	    		media=dtlong;
    	    		sum=dtlong;
    	    	}
    	else{
    		sum=sum+dtlong;
    		media=sum/count;
    		if(dtlong<min){
    			min=dtlong;
    		}
    		if(dtlong>max){
    			max=dtlong;
    		}
    	}


        if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
                   {
                       // after desired message is successfully decoded, we can use the information stored in pos_data vector
                       // right here, or we can do something with it from inside decodeSingleMessage() function(see ublox.h).
                       // the way, data is stored in pos_data vector is specified in decodeMessage() function of class UBXParser(see ublox.h)
        	printf("--------------------------------------------------------------------------------------------------\n");
        	/*gettimeofday(&tv2,NULL);
        	currenttime = 1000000 * tv2.tv_sec + tv2.tv_usec;
        	dtlong=currenttime-previoustime;*/
        	printf("Numero da leitura: %lu \n", count);
        	printf("Duracao minima microsegundos da leitura dos sensores: %lu \n", min);
        	printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtlong);
        	printf("Duracao media em microsegundos da leitura dos sensores: %lu \n", media);
        	printf("Duracao maxima microsegundos da leitura dos sensores: %lu \n", max);
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
        	printf("-----------------------------------Leitura do GPS-------------------------------------------------");
        	        printf("\nGPS Millisecond Time of Week: %.0lf s\n", pos_data[0]/1000);
                    printf("Longitude: %lf\n", pos_data[1]/10000000);
                    printf("Latitude: %lf\n", pos_data[2]/10000000);
                    printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
                    printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
                    printf("Horizontal Accuracy Estateimate: %.3lf m\n", pos_data[5]/1000);
                    printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);
                    printf("--------------------------------------------------------------------------------------------------\n");
                   } else {
                	   printf("Numero da leitura: %lu \n", count);
                	   printf("Duracao minima microsegundos da leitura dos sensores: %lu \n", min);
                	   printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtlong);
                	   printf("Duracao media em microsegundos da leitura dos sensores: %lu \n", media);
                	   printf("Duracao maxima microsegundos da leitura dos sensores: %lu \n", max);
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
                       // printf("Message not captured\n");
                       // use this to see, how often you get the right messages
                       // to increase the frequency you can turn off the undesired messages or tweak ublox settings
                       // to increase internal receiver frequency
                   }

                   /*if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1)
                   {
                       printf("Current GPS status:\n");
                       printf("gpsFixOk: %d\n", ((int)pos_data[1] & 0x01));

                       printf("gps Fix status: ");
                       switch((int)pos_data[0]){
                           case 0x00:
                               printf("no fix\n");
                               break;

                           case 0x01:
                               printf("dead reckoning only\n");
                               break;

                           case 0x02:
                               printf("2D-fix\n");
                               break;

                           case 0x03:
                               printf("3D-fix\n");
                               break;

                           case 0x04:
                               printf("GPS + dead reckoning combined\n");
                               break;

                           case 0x05:
                               printf("Time only fix\n");
                               break;

                           default:
                               printf("Reserved value. Current state unknown\n");
                               break;

                       }

                       printf("\n");

                   } else {
                       // printf("Status Message not captured\n");
                   }*/


                   usleep(10000);
               }

           } else {

               printf("Ublox test not passed\nAbort program!\n");

           }
    pthread_exit(NULL);
           return 0;
       }

