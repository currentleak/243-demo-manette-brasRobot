/**
 * @file    243-336_TestManette1.c
 * 
 * @brief   Exemple de code C pour test de la Manette
 * 			processus (thread) avec la commande fork, utilisation de pipe
 *          Lecture/Ecriture sur port série UART
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

char* portTTY = "/dev/ttyUSB0";

/// @brief processus: Lecture sur le port Série, écriture dans le terminal
/// @param fd_UART: descripteur de fichier pour le port serie UART
void LirePortSerie(int fd_UART)
{
    // Read data from serial port 
	tcflush(fd_UART, TCIFLUSH);  // Discards old data in the rx buffer
	u_char read_buffer[32];   // Buffer to store the data received 
	int  bytes_read = 0;    // Number of bytes read by the read() system call 
	int i = 0;
	u_int8_t checksum = 0;  // checksum sur 8bit comme le PIC16F88 8bit

	while(1) 
	{
		bytes_read = read(fd_UART, &read_buffer, 32); // Lecture des données sur le port série 		
		//printf(" octets recus : %d --> ", bytes_read); // Print the number of bytes read

		if (bytes_read == tailleMessage) // Si 11 octets lus, c'est un message de commande
		{
			//printf("Commande reçue: \n");
			
			// if(read_buffer[bytes_read-1] == caractereFermeUART) // Vérifier le caractère de fermeture
			// 	return;

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
				tcflush(fd_UART, TCIFLUSH);
			}
			// todo : verifier que le premier octet est le bon : 'S' pour Start

			else
			{
				printf("\033[2J"); // Effacer l'écran
				printf("\033[H");  // Revenir au début de l'écran
				// Afficher les données lues -- ADC Thumbsticks, boutons, etc.
				printf("Thumbstick Droite X: %d", read_buffer[1]);
				printf(" Y: %d\n", read_buffer[2]);
				printf("Thumbstick Gauche X: %d", read_buffer[3]);
				printf(" Y: %d\n", read_buffer[4]);

				printf("Encoder: %d (0b", read_buffer[5]);
				printf(")\n");

				printf("Sel.1: %d ", BOUTON_SELECT_1 & read_buffer[6]);
				printf("Sel.2: %d\n", BOUTON_SELECT_2 & read_buffer[6]);
				printf("Top L: %d\n", BOUTON_TOP_L & read_buffer[6]);
				printf("Haut: %d ", BOUTON_HAUT & read_buffer[6]);	
				printf("Gauche: %d ", BOUTON_GAUCHE & read_buffer[6]);
				printf("Droite: %d ", BOUTON_DROITE & read_buffer[6]);
				printf("Bas: %d\n", BOUTON_BAS & read_buffer[6]);
				printf("Bot1 L: %d\n", BOUTON_BOT1_L & read_buffer[6]);				

				printf("Bot2 L: %d\n", BOUTON_BOT2_L & read_buffer[7]);
				printf("TS L: %d\n", BOUTON_TS_L & read_buffer[7]);

				printf("Sel.3: %d ", BOUTON_SELECT_3 & read_buffer[8]);
				printf("Sel.4: %d\n", BOUTON_SELECT_4 & read_buffer[8]);
				printf("Top R: %d\n", BOUTON_TOP_R & read_buffer[8]);
				printf("Y: %d ", BOUTON_Y & read_buffer[8]);
				printf("X: %d ", BOUTON_X & read_buffer[8]);
				printf("B: %d ", BOUTON_B & read_buffer[8]);
				printf("A: %d\n", BOUTON_A & read_buffer[8]);
	 			printf("Bot1 R: %d\n", BOUTON_BOT1_R & read_buffer[8]);
				printf("Bot2 R: %d\n", BOUTON_BOT2_R & read_buffer[9]);
				printf("TS R: %d\n", BOUTON_TS_R & read_buffer[9]);
				printf("HOME: %d\n", BOUTON_HOME & read_buffer[9]);

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
}

/// @brief processus: Lecture dans le terminal, écriture sur le port série
/// @param fd_UART: descripteur de fichier pour le port serie UART
/// @param 
void EcrirePortSerie(int fd_UART)
{
	while(1)
	{
    char caractere; 	// character to write into port
	caractere = getchar(); // Lecture des données dans le terminal
    write(fd_UART, &caractere, sizeof(caractere)); // Écriture des données sur le port série
	//printf(".%c.", caractere);			
	}
	
}

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
	printf("\n Ouverture de %s reussit, fd=%d ", portTTY, fd_UART);
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
		printf("Exemple pour connexion USB: %s 1\n", argv[0]);
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
		else
		{
			printf("default : ttyUSB0\n");
			portTTY = "/dev/ttyUSB0";
		}
	}

    int fd_portUART; /// descripteur de fichier pour le port UART (TTY)

	fd_portUART = OuvrirConfigurerPortSerie();
    if (fd_portUART >0)
		printf("Ouverture du port série %s réussie, fd=%d\n", portTTY, fd_portUART);
	else {
		printf("Erreur lors de l'ouverture du port série %s\n", portTTY);
		exit(1);
	}  


    pid_t pidEcrire, pidLire;
    pidEcrire = fork(); // processus enfant 1
    if (pidEcrire == 0) {
		EcrirePortSerie(fd_portUART);
        printf("Fin du processus Ecriture PortSerie\n"); // enfant 1
    	}
    else {
		pidLire = fork(); // processus enfant 2
		if (pidLire == 0) {
			LirePortSerie(fd_portUART);
        	printf("Fin du processus Lecture PortSerie\n"); 
		}
		else {
			// faire processus principal
			int n=1;

			while(n < 5) {
				printf("%d faire quelques trucs...\n", n);
				// TODO: 
				
				n++;
				sleep(3);
			}
			
			wait(NULL);	// attendre la fin de enfant 1
			wait(NULL); // attendre la fin de enfant 2
			close(fd_portUART);	// Fermer le port série
			printf("Fin du processus Principal\n");
		}

        fflush(NULL);
    }
    return 0;
}
