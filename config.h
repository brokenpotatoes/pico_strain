//Pin defs
//Excitation control
#define EX_EN_PIN 15;
#define PGOOD 16;
//ADC Pins
#define SPI_FREQ            25e6 // 25mhz -> max SPI freq of the ADC
#define nDRDY_PIN           7
#define nCS_PIN             9
#define nSYNC_nRESET_PIN    12
#define CLK_PIN             21 //Only for RP2040 CLK not availible on pin 13
#define MISO_PIN            8
#define MOSI_PIN            11
#define SCK_PIN             10