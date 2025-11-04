#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "pca9685.h"

#define I2C_BUS "/dev/i2c-1"
#define JOY_DEV "/dev/input/js0"
#define SERVO_CHANNEL 0

int main(void)
{
    int js_fd, i2c_fd;
    struct js_event js;

    // Ouvrir le joystick
    js_fd = open(JOY_DEV, O_RDONLY);
    if (js_fd < 0) {
        perror("Erreur ouverture joystick");
        return 1;
    }
    fcntl(js_fd, F_SETFL, O_NONBLOCK); // Lecture non bloquante

    // Ouvrir le bus I2C
    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) {
        perror("Erreur ouverture I2C");
        return 2;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("Erreur accès PCA9685");
        close(i2c_fd);
        return 3;
    }

    // Initialiser le PCA9685
    pca9685_init(i2c_fd);

    printf("Contrôle du servo canal %d avec le Thumbstick Droit X\n", SERVO_CHANNEL);
    printf("Centre (0) -> 90°  |  Gauche -> 0°  |  Droite -> 180°\n");

    int16_t x_axis = 0;
    int angle = 90;

    while (1) {
        if (read(js_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {
            if ((js.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
                if (js.number == 3) { // Axe X du stick droit (souvent axe 3)
                    x_axis = js.value;

                    // Convertir [-32768, 32767] → [0, 180]
                    angle = (int)(((x_axis + 32768) / 65535.0) * 180.0);

                    set_servo_angle(i2c_fd, SERVO_CHANNEL, angle);

                    printf("\rAxe X = %6d → Angle = %3d°   ", x_axis, angle);
                    fflush(stdout);
                }
            }
        }
        usleep(20000); // 20 ms (50 Hz)
    }

    close(js_fd);
    close(i2c_fd);
    return 0;
}
