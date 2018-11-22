 #include <stdio.h>
 #include <string.h>
 #include <time.h>
 #include <pthread.h>
 #include "spi.h"
 #include "gpio.h"
 
 #define SPI_SPEED 8312500
 #define SPI_BITS_PER_WORD 32
 #define BUFF_SIZE 256
 #define GPIO_PIN_NO 7
 #define SLEEP_NSECS 200000
 #define NSECS_PER_SEC 1000000000
 //#define PERIOD_NSECS 1000000 // 1ms
 #define PERIOD_NSECS 500000 // 500us
 
 #define THREAD_PRIORITY 50
 
 
 int add_nsecs(struct timespec *ts, long nsec)
 {
     ts->tv_nsec+=nsec;
     while(ts->tv_nsec > NSECS_PER_SEC)
     {
         ts->tv_nsec-=NSECS_PER_SEC;
         ts->tv_sec++;
     }
     return 0;
 }
 
 void *spi_thread(void *par)
 {
     int fd;
     char buff_tx[BUFF_SIZE];
     char buff_rx[BUFF_SIZE];
     GPIO_PIN_DAT gpio_pin;
     struct timespec req;
 
     fd=spi_open("/dev/spidev3.0", 0, SPI_BITS_PER_WORD, SPI_SPEED);
     if(fd < 0)
     {
         fprintf(stderr,"failed to open device\n");
         return (void *)-1;
     }
     gpio_pin.gpio_pin_no=GPIO_PIN_NO;
     gpio_pin.gpio_set_dir=GPIO_DIR_OUT;
     gpio_setup(&gpio_pin);
 
     memset(buff_tx, 0xaa, BUFF_SIZE);
     gpio_write_pin(&gpio_pin, LOW);
     clock_gettime(CLOCK_MONOTONIC, &req);
 
     while(1)
     {
         gpio_write_pin(&gpio_pin, HIGH);
         if(spi_transfer(fd, buff_tx, buff_rx, BUFF_SIZE) < 0)
             fprintf(stderr,"failed to transfer\n");
         gpio_write_pin(&gpio_pin, LOW);
         add_nsecs(&req, PERIOD_NSECS);
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &req, NULL);
     }
 
     gpio_close(&gpio_pin);
     spi_close(fd);
     return NULL;
 }
 
 int main(int argc, char **argv)
 {
     pthread_attr_t attr;
     struct sched_param sched;
     pthread_t thread;
 
     // set somethread attributes
     pthread_attr_init(&attr);
     pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
     sched.sched_priority=THREAD_PRIORITY;
     pthread_attr_setschedparam(&attr, &sched);
     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
 
     // start the thread
     pthread_create(&thread, &attr, spi_thread, NULL);
     // wait for thread to terminate
     pthread_join(thread, NULL);
 
     return 0;
 }