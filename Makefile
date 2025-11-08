CC = gcc
CFLAGS = -Wall -O2
SRC = manette.c pca9685.c
OUT = manette

all:
	$(CC) $(SRC) -o $(OUT) $(CFLAGS) -lm

clean:
	rm -f $(OUT)
