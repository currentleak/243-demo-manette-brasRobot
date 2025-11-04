//const char* portTTY = "/dev/ttyUSB0";
//const char* portTTY = "/dev/ttyAMA0";
const char caractereFermeUART = '!';

const int tailleMessage = 11; // Taille du message attendu

const u_int8_t BOUTON_SELECT_1 = 0x01; // Bouton Select 1
const u_int8_t BOUTON_SELECT_2 = 0x02; // Bouton Select 2 
const u_int8_t BOUTON_TOP_L = 0x04; // Bouton Top LEFT
const u_int8_t BOUTON_HAUT = 0x08; 
const u_int8_t BOUTON_GAUCHE = 0x10; 
const u_int8_t BOUTON_DROITE = 0x20;
const u_int8_t BOUTON_BAS = 0x40;
const u_int8_t BOUTON_BOT1_L = 0x80;

const u_int8_t BOUTON_BOT2_L = 0x01;
const u_int8_t BOUTON_TS_L = 0x02;

//const u_int8_t DEL_VERTE = 0x04; 
//const u_int8_t DEL_JAUNE = 0x08;

const u_int8_t BOUTON_SELECT_3 = 0x01; 
const u_int8_t BOUTON_SELECT_4 = 0x02;
const u_int8_t BOUTON_TOP_R = 0x04;  
const u_int8_t BOUTON_Y = 0x08; 
const u_int8_t BOUTON_X = 0x10; 
const u_int8_t BOUTON_B = 0x20; 
const u_int8_t BOUTON_A = 0x40; 
const u_int8_t BOUTON_BOT1_R = 0x80;

const u_int8_t BOUTON_BOT2_R = 0x01;
const u_int8_t BOUTON_TS_R = 0x02;

//const u_int8_t DEL_ROUGE = 0x04; 
//const u_int8_t DEL_ORANGE = 0x08; 
 
const u_int8_t BOUTON_HOME = 0x80; 
