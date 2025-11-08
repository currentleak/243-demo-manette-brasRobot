#ifndef PCA9685_H
#define PCA9685_H

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdlib.h>

// Adresse I2C par défaut du PCA9685
#define PCA9685_ADDR 0x40

// Registres principaux
#define MODE1       0x00
#define PRESCALE    0xFE
#define LED0_ON_L   0x06

#define SERVO_CHANNEL 0

#define SERVO_BASE      4
#define SERVO_EPAULE    3
#define SERVO_COUDE     2
#define SERVO_POIGNET   1
#define SERVO_MAIN      0


#define MAX_ANGLE 1260
//#define MAX_ANGLE 180


// Prototypes
int pca9685_init(int fd);
void set_servo_angle(int fd, int channel, int angle);
int CheckAngleMax(int angle);

#endif