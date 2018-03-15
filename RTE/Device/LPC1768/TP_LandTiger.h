#include "lpc17xx.h"
#include <stdio.h>

#define   CHX            0x90
#define   CHY            0xd0

#define SSP_SR_TFE        1      //Transmit FIFO Empty
#define SSP_SR_TNF       (1 << 1)//Transmit FIFO Not Full
#define SSP_SR_RNE       (1 << 2)//Receive FIFO Not Empty
#define SSP_SR_RFF       (1 << 3)//Receive FIFO Full
#define SSP_SR_BSY       (1 << 4)//Busy

#define SSPSR_RNE       2
#define SSPSR_BSY       4

#define SPI_SPEED_4MHz     18     /* 4MHz */
#define SPI_SPEED_2MHz     36     /* 2MHz */
#define SPI_SPEED_1MHz     72     /* 1MHz */
#define SPI_SPEED_500kHz  144     /* 500kHz */
#define SPI_SPEED_400kHz  180     /* 400kHz */

//GPIO0
#define PIN_TP_CS  (1 << 6)
//GPIO2
#define PIN_TP_INT (1 << 13)

#define TP_CS(a)   if (a)   \
      LPC_GPIO0->FIOSET = PIN_TP_CS;\
   else \
      LPC_GPIO0->FIOCLR = PIN_TP_CS;

#define TP_INT_IN   ( LPC_GPIO2->FIOPIN & PIN_TP_INT )
#define TOUCHPANEL_ERROR 2
#define TOUCHPANEL_MEAN_SAMPLES 50

void LPC17xx_SPI_SetSpeed(uint8_t speed);
int touchpanelReadX(void);

int touchpanelReadY(void);
void touchpanelGetXY(int *x, int *y);

void touchpanelInit(void);
