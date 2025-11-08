/**
 * @file    243-336_TestManette1.c
 * 
 * @brief   Exemple de code C pour demo Manette bras robot
 *         
 * @author  Kevin Cotton
 * @date    2025-07-06
 *
 */

#define _GNU_SOURCE 

#include <stdlib.h> // Standard Library Definition
#include <stdio.h> // Standard Input/Output Definitions
#include <unistd.h> // UNIX Standard Definitions 
#include <sys/wait.h> // Wait for Process Termination
#include <sys/ioctl.h> // Input/Output Control Definitions
#include <string.h> // String Function Definitions
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions 
#include <errno.h>   // ERROR Number Definitions

#include <linux/input.h>
#include <linux/uinput.h>

#include "manette.h"
#include "pca9685.h"

#include "pca9685.c"

#define I2C_BUS "/dev/i2c-1"

char* portTTY = "/dev/ttyUSB0";

char read_buffer[32];   // Buffer to store the data received 

/// @brief processus: Config et ouverture du port série UART
/// @param Return: File Descriptor du port série
/// @param 
int OuvrirConfigurerPortSerie(void)
{
    int fd_UART;
    fd_UART = open(portTTY, O_RDWR | O_NOCTTY ); //| O_NDELAY);  // Opening the Serial Port, O_RDWR Read/Write access to serial port,           
								            // O_NOCTTY - No terminal will control the process, O_NDELAY - Non-blocking mode     
	if(fd_UART < 0) // Error Checking
	{
		perror("open_port: Unable to open serial port");
		return -1;
	}
	//printf("\n Ouverture de %s reussit, fd=%d ", portTTY, fd_UART);
	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd_UART, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B57600); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B57600); // Set Write Speed  
	//cfsetispeed(&SerialPortSettings, B19200); // Set Read Speed  
	//cfsetospeed(&SerialPortSettings, B19200); // Set Write Speed
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  

	SerialPortSettings.c_cflag &= ~CRTSCTS;  // No Hardware flow Control
	//SerialPortSettings.c_cflag |= CRTSCTS; //Enable RTS/CTS
	SerialPortSettings.c_cflag |= CLOCAL; // Ignore Modem Control lines
	SerialPortSettings.c_cflag |= CREAD;	// Enable receiver
	//SerialPortSettings.c_iflag &= ~(INLCR | IGNCR | ICRNL); // Disable CR to NL mapping, Ignore CR and NL

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG);  //Disable echo, Disable signal  
    SerialPortSettings.c_lflag &= ~ICANON;	// Non Cannonical mode (byte mode)
	//SerialPortSettings.c_lflag |= ICANON;	// Cannonical mode (terminal)

	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = tailleMessage; //1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 0; // Wait.. (30 for 3sec), (0 for indefinetly) 

	if((tcsetattr(fd_UART, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
	{
		printf("\n Erreur! configuration des attributs du port serie\n");
		close(fd_UART); // Fermer le port série en cas d'erreur
		fflush(NULL);
		perror("Erreur lors de la configuration du port série");
		fflush(stderr);
		return -1;
	}

	// la pin RTS peut être utilisé comme RESET pour le microcontrolleur
	int status; 
	ioctl(fd_UART, TIOCMGET, &status); // Get the current status of the port
    status &= ~TIOCM_RTS; // Clear RTS bit to set it low RS232 -- high on UART side
   	ioctl(fd_UART, TIOCMSET, &status); // write RTS pin
	///status |= TIOCM_RTS; // Set RTS bit to high RS232 -- low on UART side
	//ioctl(fd_UART, TIOCMSET, &status); // write RTS pin 

    return fd_UART;
}

void printValueToTerminal(void)
{
				printf("Thumbstick Droite X: %d", read_buffer[1]);
				printf(" Y: %d\n", read_buffer[2]);
				printf("Thumbstick Gauche X: %d", read_buffer[3]);
				printf(" Y: %d\n", read_buffer[4]);
				printf("Encoder: %d (0b", read_buffer[5]);
				printf(")\n");

				printf("Top L: %d\n", BOUTON_TOP_L & read_buffer[6]);
				printf("Haut: %d ", BOUTON_HAUT & read_buffer[6]);	
				printf("Gauche: %d ", BOUTON_GAUCHE & read_buffer[6]);
				printf("Droite: %d ", BOUTON_DROITE & read_buffer[6]);
				printf("Bas: %d\n", BOUTON_BAS & read_buffer[6]);
				printf("Bot1 L: %d\n", BOUTON_BOT1_L & read_buffer[6]);				
				printf("Bot2 L: %d\n", BOUTON_BOT2_L & read_buffer[7]);

				printf("Top R: %d\n", BOUTON_TOP_R & read_buffer[8]);
				printf("Y: %d ", BOUTON_Y & read_buffer[8]);
				printf("X: %d ", BOUTON_X & read_buffer[8]);
				printf("B: %d ", BOUTON_B & read_buffer[8]);
				printf("A: %d\n", BOUTON_A & read_buffer[8]);
	 			printf("Bot1 R: %d\n", BOUTON_BOT1_R & read_buffer[8]);
				printf("Bot2 R: %d\n", BOUTON_BOT2_R & read_buffer[9]);
				printf("HOME: %d\n", BOUTON_HOME & read_buffer[9]);

}


/// @brief Exemple de processus gestion du port série UART
/// @return 0 on success, -1 on error
int main(int argc, char *argv[])
{   
	if(argc == 1)
	{
		printf("Sélectionner le port de Communication\n");
		printf("1 : Port USB (ttyUSB0)\n");
		printf("2 : Bluetooth (rfcomm0)\n");
		printf("3 : raw UART (ttyAMA0)\n");
		printf("4 : Port USB (ttyUSB1)\n");
		printf("Exemple pour connexion USB0: %s 1\n", argv[0]);
		return 1;
	}
	else 
	{
		if (*argv[1]=='1')
		{
			portTTY = "/dev/ttyUSB0";
		}
		else if (*argv[1]=='2')
		{
			portTTY = "/dev/rfcomm0";
		}
		else if (*argv[1]=='3')
		{
			portTTY = "/dev/ttyAMA0";
		}
		else if (*argv[1]=='4')
		{
			portTTY = "/dev/ttyUSB1";
		}
		else
		{
			printf("default : ttyUSB0\n");
			portTTY = "/dev/ttyUSB0";
		}
	}

    // ouvrire le port uart
    int fd_portUART;
	fd_portUART = OuvrirConfigurerPortSerie();
    if (fd_portUART >0)
		printf("Ouverture du port série %s réussie, fd=%d\n", portTTY, fd_portUART);
	else {
		printf("Erreur lors de l'ouverture du port série %s\n", portTTY);
		exit(1);
	}  
    // Read data from serial port 
	tcflush(fd_portUART, TCIFLUSH);  // Discards old data in the rx buffer
    int  bytes_read = 0;    // Number of bytes read by the read() system call 
	u_int8_t checksum = 0;  // checksum sur 8bit comme le PIC16F88 8bit

    // Ouvrir le bus I2C
    int i2c_fd;
    i2c_fd = open(I2C_BUS, O_RDWR);
    if (i2c_fd < 0) {
        perror("Erreur ouverture I2C");
		close(fd_portUART);
        return 2;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("Erreur accès PCA9685");
        close(i2c_fd);
		close(fd_portUART);
        return 3;
    }
    // Initialiser le PCA9685
    if(pca9685_init(i2c_fd) != 0)
	{
		perror("Erreur init PCA9685");
		close(i2c_fd);
		close(fd_portUART);
		return 4;
	}

	int counter = 0;
    uint8_t prev_state = 0;
    uint8_t curr_state;

    int16_t x_axis_L = 0;
	int16_t y_axis_L = 0;
	int16_t x_axis_R = 0;
	int16_t y_axis_R = 0;

    int angle = (int)MAX_ANGLE/2;
	int angle_main = (int)MAX_ANGLE/2;
	int angle_epaule = (int)MAX_ANGLE/2;
	int angle_coude = (int)MAX_ANGLE/2;

	static uint8_t prev = 0;

	// set all servo to default
	set_servo_angle(i2c_fd, SERVO_MAIN, angle_main);
	set_servo_angle(i2c_fd, SERVO_POIGNET, angle);
	set_servo_angle(i2c_fd, SERVO_COUDE, angle_coude);
	set_servo_angle(i2c_fd, SERVO_EPAULE, angle_epaule);
	set_servo_angle(i2c_fd, SERVO_BASE, angle);

    int i = 0;
	while(1) 
	{
		bytes_read = read(fd_portUART, &read_buffer, 32); // Lecture des données sur le port série 		
		//printf(" octets recus : %d --> ", bytes_read); // Print the number of bytes read

		if (bytes_read == tailleMessage) // Si 11 octets lus, c'est un message de commande
		{
 			for(i=0; i<tailleMessage-1; i++)
			{
				// if(read_buffer[i] == caractereFermeUART)
				// 	return;
				// Calculer le checksum
				checksum += read_buffer[i];
			} 
			if(checksum != read_buffer[tailleMessage-1]) // Vérifier le checksum
			{
				printf("Erreur de checksum: %d != %d\n", checksum, read_buffer[bytes_read-1]);
				tcflush(fd_portUART, TCIFLUSH);
			}
			// todo : verifier que le premier octet est le bon : 'S' pour Start

			else
			{
                //printValueToTerminal();

				/// Joystick -- Base et Poignet
				x_axis_L = read_buffer[3];
                angle = (int)(((x_axis_L) / 255.0) * MAX_ANGLE);
				if(angle > 800)
					angle = 800;
				else if(angle < 400)
					angle = 400;
                set_servo_angle(i2c_fd, SERVO_BASE, angle);
                printf("\nAxe X gauche = %6d → Angle = %4d°   ", x_axis_L, angle);

				y_axis_L = read_buffer[4];
                angle = (int)(((y_axis_L) / 255.0) * MAX_ANGLE);
                set_servo_angle(i2c_fd, SERVO_POIGNET, angle);
                printf("\nAxe Y gauche = %6d → Angle = %4d°   ", y_axis_L, angle);
				printf("\n");

				/// Encodeur -- Main
    			uint8_t A = (read_buffer[5] >> 7) & 1;
   				uint8_t B = (read_buffer[5] >> 3) & 1;
    			uint8_t curr = (A << 1) | B;
    			if ((prev == 0 && curr == 1) || (prev == 1 && curr == 3) || (prev == 3 && curr == 2) || (prev == 2 && curr == 0))
        			angle_main = angle_main +100;
   				else if ((prev == 0 && curr == 2) || (prev == 2 && curr == 3) || (prev == 3 && curr == 1) || (prev == 1 && curr == 0))
        			angle_main = angle_main -100;	
				angle_main = CheckAngleMax(angle_main);
	
				set_servo_angle(i2c_fd, SERVO_MAIN, angle_main);
 				printf("angle_main = %4d°   ", angle_main);
    			prev = curr;

				/// 4 Bouttons flèche de droite -- Epaule et Coude
				if( (BOUTON_Y & read_buffer[8]) == 0)
				{
					angle_epaule = angle_epaule + 3;
				}
				if( (BOUTON_A & read_buffer[8]) == 0)
				{
					angle_epaule = angle_epaule - 3;
				}
				angle_epaule = CheckAngleMax(angle_epaule);
				set_servo_angle(i2c_fd, SERVO_EPAULE, angle_epaule);
				printf("angle_epaule: %d ", angle_epaule);

				if( (BOUTON_X & read_buffer[8]) == 0)
				{
					angle_coude = angle_coude + 3;
				}
				if( (BOUTON_B & read_buffer[8]) == 0)
				{
					angle_coude = angle_coude - 3;
				}
				angle_coude = CheckAngleMax(angle_coude);
				set_servo_angle(i2c_fd, SERVO_COUDE, angle_coude);
				printf("angle_coude: %d ", angle_coude);

                fflush(stdout);
			}
			checksum = 0; // Réinitialiser le checksum pour la prochaine lecture
	
		}
		else if (bytes_read < 0) // Si erreur de lecture
		{
			printf("\n Erreur de lecture du port série %s\n", portTTY);
			perror("Erreur de lecture du port série");
			exit(-1);
		}
		else if (bytes_read == 0) // Si aucune donnée lue
		{
			printf("\n Aucune donnée lue sur le port série %s\n", portTTY);
		}
		else if (bytes_read > 0) // Si des données ont été lues
		{
			printf("Données lues: nombres incompatibles avec la commande\n");
		}

	}


	close(i2c_fd);
    close(fd_portUART);
    return 0;
}