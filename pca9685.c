#include "pca9685.h"

// ---------------------------------------------------------------------------
// Écriture d’un octet (registre + donnée)
// ---------------------------------------------------------------------------
int i2c_write(int fd, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    if (write(fd, buffer, 2) != 2) {
        perror("Erreur écriture I2C");
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Conversion d’un angle 0–180° en ticks PWM (1 ms → 205, 2 ms → 410)
// ---------------------------------------------------------------------------
static int angle_to_ticks(int angle)
{
    // servo DF9GMS
    //if (angle < 0) angle = 0;
    //if (angle > 180) angle = 180;
    //return 205 + (int)((angle / 180.0) * (410 - 205));

    // servo HS-785HB
    if (angle < 0) angle = 0;
    if (angle > 1260) angle = 1260;

    int min_ticks = 184;  // correspond à ~0.9 ms
    int max_ticks = 430;  // correspond à ~2.1 ms
    return min_ticks + (int)((angle / 1260.0) * (max_ticks - min_ticks));
}

// ---------------------------------------------------------------------------
// Initialisation du PCA9685 à 50 Hz (pour servomoteurs)
// ---------------------------------------------------------------------------
int pca9685_init(int fd)
{
    // Mettre le PCA9685 en mode Sleep
    if(i2c_write(fd, MODE1, 0x10) != 0 )
    {
        return -1;
    }
    usleep(1000);

    // Régler la fréquence à 50 Hz (prescale = 0x7F)
    if(i2c_write(fd, PRESCALE, 0x7F) !=0 )
    {
        return -1;
    }
    usleep(1000);

    // Réactiver avec Auto-Increment + Restart
    if(i2c_write(fd, MODE1, 0xA0) !=0)
    {
        return -1;
    }
    usleep(1000);

    return 0;
}

// ---------------------------------------------------------------------------
// Définir un angle de servo sur un canal donné
// ---------------------------------------------------------------------------
void set_servo_angle(int fd, int channel, int angle)
{
    int ticks = angle_to_ticks(angle);
    int led_base = LED0_ON_L + 4 * channel;

    i2c_write(fd, led_base + 0, 0x00);          // ON_L
    i2c_write(fd, led_base + 1, 0x00);          // ON_H
    i2c_write(fd, led_base + 2, ticks & 0xFF);  // OFF_L
    i2c_write(fd, led_base + 3, ticks >> 8);    // OFF_H
}

/// @brief 
/// @param angle 
/// @return 
int CheckAngleMax(int angle)
{
	if(angle > 800)
		angle = 800;
	else if(angle < 400)
		angle = 400;
    return angle;
}