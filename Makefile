CC = gcc
CFLAGS = -Wall -O2
SRC = main_servo.c pca9685.c
OUT = servo_control

all:
	$(CC) $(SRC) -o $(OUT) $(CFLAGS) -lm

clean:
	rm -f $(OUT)
