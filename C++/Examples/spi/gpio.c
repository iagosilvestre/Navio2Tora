#include <stdio.h>
 #include <limits.h>
 #include <fcntl.h>
 #include <unistd.h>
 
 #include "gpio.h"
 
 static const char *gpio_file_templ="/sys/class/gpio/gpio%d/%s";
 
 //*----------------------------------------------------------------------------
 //* Function Name       : gpio_setup
 //* Object              : define a direction for a pin and open the value file
 //* Input Parameters    : p_dat: pointer to pin data
 //*                       
 //*                     : 
 //* Output Parameters   : FALSE on error
 //+ Date                : 25.07.2018  Dietmar Muscholik
 //*----------------------------------------------------------------------------
 uint32_t gpio_setup( GPIO_PIN_DAT *p_dat)
 {
     uint32_t ret = FALSE;
     char path[PATH_MAX];
     int fd;
 
     snprintf(path, sizeof(path), gpio_file_templ, p_dat->gpio_pin_no, "direction");
     fd=open(path, O_WRONLY);
     if(fd > 0)
     {
         switch(p_dat->gpio_set_dir)
         {
         case GPIO_DIR_IN:
             if(write(fd, "in", 2) > 0)
                 ret = TRUE;
             break;
         case GPIO_DIR_OUT:
             if(write(fd, "out", 3) > 0)
                 ret = TRUE;
             break;
         }
         close(fd);
         if(ret)
         {
             snprintf(path, sizeof(path), gpio_file_templ, p_dat->gpio_pin_no, "value");
             p_dat->gpio_pin_fd=open(path, O_RDWR);
             if(p_dat->gpio_pin_fd < 0)
                 ret=FALSE;
         }
     }
 
     return ret;
 }
 
 
 //*----------------------------------------------------------------------------
 //* Function Name       : gpio_write
 //* Object              : backward compatibility, better use gpio_write_pin
 //* Input Parameters    : p_dat: pointer to pin data
 //*                       
 //*                     : 
 //* Output Parameters   : none
 //+ Date                : 25.07.2018  Dietmar Muscholik
 //*----------------------------------------------------------------------------
 void gpio_write( GPIO_PIN_DAT *p_dat)
 {
     if(p_dat->gpio_value)
         write(p_dat->gpio_pin_fd, "1", 1);
     else
         write(p_dat->gpio_pin_fd, "0", 1);
 }
 
 //*----------------------------------------------------------------------------
 //* Function Name       : gpio_read
 //* Object              : read the state of a pin
 //* Input Parameters    : p_dat: pointer to pin data
 //*                       
 //*                     : 
 //* Output Parameters   : state of the pin (HIGH or LOW)
 //+ Date                : 25.07.2018  Dietmar Muscholik
 //*----------------------------------------------------------------------------
 int32_t gpio_read( GPIO_PIN_DAT *p_dat)
 {
     char buff[2];
 
     lseek(p_dat->gpio_pin_fd,0, SEEK_SET);
     if(read(p_dat->gpio_pin_fd, buff, sizeof(buff)) > 0)
         return *buff=='1' ? HIGH : LOW;
     return -1;
 }
 
 
 //*----------------------------------------------------------------------------
 //* Function Name       : gpio_write
 //* Object              : write to a pin
 //* Input Parameters    : p_dat: pointer to pin data
 //*                       state: either HIGH or LOW
 //*                     : 
 //* Output Parameters   : none
 //+ Date                : 25.07.2018  Dietmar Muscholik
 //*----------------------------------------------------------------------------
 void gpio_write_pin( GPIO_PIN_DAT *p_dat, uint32_t state)
 {
     if(state)   // everything but LOW is HIGH
         write(p_dat->gpio_pin_fd, "1", 1);
     else
         write(p_dat->gpio_pin_fd, "0", 1);
 }
 //*----------------------------------------------------------------------------
 //* Function Name       : gpio_close
 //* Object              : close the value file
 //* Input Parameters    : none
 //*                       
 //*                     : 
 //* Output Parameters   : none
 //+ Date                : 25.07.2018  Dietmar Muscholik
 //*----------------------------------------------------------------------------
 void gpio_close( GPIO_PIN_DAT *p_dat)
 {
     close(p_dat->gpio_pin_fd);
 }
 