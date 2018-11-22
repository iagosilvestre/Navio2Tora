 #ifndef GPIO_H
 #define GPIO_H
 
 #include <stdint.h>
 
 // maybe enum
 #define GPIO_DIR_OUT 1
 #define GPIO_DIR_IN  0
 
 // enum
 #ifndef HIGH
 #define HIGH (1)
 #endif
 
 #ifndef LOW
 #define LOW (0)
 #endif
 
 #ifndef BOOL
 typedef int BOOL;
 #endif
 
 #ifndef TRUE
 #define FALSE 0
 #define TRUE (!FALSE)
 #endif
 
 typedef struct
 {
     uint32_t    gpio_value;
     uint32_t    gpio_pin_no;    // number of the pin
     int         gpio_pin_fd;    // filedescriptor of the value file of the pin
     uint32_t    gpio_pin_mask;  // only for backward compatibility
                                 // because it is frequently used in the code
     uint32_t    gpio_set_dir;
 } GPIO_PIN_DAT;
 
 uint32_t gpio_setup( GPIO_PIN_DAT *); // better use BOOL
 void gpio_write( GPIO_PIN_DAT *);
 int32_t gpio_read( GPIO_PIN_DAT *); // negative return indicates an error
 void gpio_write_pin( GPIO_PIN_DAT *, uint32_t);
 // additional functions
 void gpio_close( GPIO_PIN_DAT *);
 
 #endif