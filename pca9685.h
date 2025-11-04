#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>

// Adresse I2C par défaut du PCA9685
#define PCA9685_ADDR 0x40

// Registres principaux
#define MODE1       0x00
#define PRESCALE    0xFE
#define LED0_ON_L   0x06

// Prototypes
int pca9685_init(int fd);
void set_servo_angle(int fd, int channel, int angle);

#endif
