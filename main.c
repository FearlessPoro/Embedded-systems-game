#include "Board_LED.h"
#include "LPC17xx.h"                    // Device header
#include "RTE_Components.h"             // Component selection
#include "GPIO_LPC17xx.h"               // Keil::Device:GPIO
#include "PIN_LPC17xx.h"                // Keil::Device:PIN
#include "Board_Joystick.h"             // ::Board Support:Joystick
#include "Open1768_LCD.h"
#include "LCD_ILI9325.h"
#include <stdio.h>
#include <math.h>
#include "asciiLib.h"
#include "TP_Open1768.h"


struct Ball{
    uint16_t x;
    uint16_t y;
    uint16_t prevX;
    uint16_t prevY;
};

struct Ball kulka;

void BallInit();

int diff =5;

bool startFlag =0;


void delay(int cnt)
{
	volatile uint32_t i;
	for (i = 0; i < cnt; i++) {
		volatile uint8_t us = 12; /*  */
		while (us--) /*  */
		{
			;
		}
	}
}


static uint16_t SSP0_write(uint16_t cmd) {
	uint8_t byte_r;

	while (LPC_SSP0->SR & (1 << SSPSR_BSY))
		; /* Wait for transfer to finish */
	LPC_SSP0->DR = cmd;
	while (LPC_SSP0->SR & (1 << SSPSR_BSY))
		; /* Wait for transfer to finish */
	while (!(LPC_SSP0->SR & (1 << SSPSR_RNE)))
		; /* Wait until the Rx FIFO is not empty */
	byte_r = LPC_SSP0->DR;

	return byte_r; /* Return received value */
}




void LCD_ascii_write(int font, unsigned char * c, int x, int y, int color)
{
    int i, j;
    long duzo = LCDRed;
    unsigned char buffer[16];  
    int k=0;
    while(c[k] != '\0')
    {
        GetASCIICode(font ,  buffer, c[k]);
  
        for(i=15; i>=0; i--)
        {
            lcdSetCursor(x+k*9, y+i );
            lcdWriteReg(DATA_RAM, duzo);
            for(j = 7; j>=0; j--)
            {
                if( (buffer[i] & (1<<j)) )
                    lcdWriteData(color);
                else
                    lcdWriteData(duzo);
            }

        }
        k++;
    }        
}

void acelerometr_setup()
{


    
    ///////////////////////////////
    LPC_SC->PCONP |= 1 << 21;//ssp0 power
    LPC_SC->PCLKSEL1 |= 1<<10;//clock selection dla ssp0
    LPC_SSP0->CPSR |= 254;//dzielnik
    
    LPC_PINCON->PINSEL0 |= 1 << 31;//funkcja SCK0, pin p0.15
    //funkcje sse0, miso0, mosi0, piny p0.16, p.0.17, p0.18
    //1 << 2 | 1 << 4 | 1 << 6
    LPC_PINCON->PINSEL1 |= 0x02 << 0 | 0x02 << 2 | 0x02 << 4;
    PIN_Configure (0, 16, 2, PIN_PINMODE_PULLUP , 0); // funkcja SSEL0 na p0.16     
    
	
	LPC_SSP0->CR0 |= 3<<6;//clock
    LPC_SSP0->CR0 |= 200 << 8;
    LPC_SSP0->CR0 |= 15 << 0;//16 bitow
    LPC_SSP0->CR1 |= 1 << 1;//ssp enable
    
    

    SSP0_write((0<<15) + 0x2401); //threshhold na minimum chyba
	SSP0_write((0<<15)+ (1<<3)+ 0x2D00); //wlaczenie measure mode


	SSP0_write(0x2008); //ustawianie w 0x20(CTRL_REG1) na 15, wybudzenie
    SSP0_write(0x1D01); // ustawienie threshold na 40 czegos

}



void LCD_Reset()
{
int i,j;
for(i=0; i<240; i++)
    {
         for(j=0; j<320; j++)
         {
             lcdSetCursor(i, j );
             lcdWriteReg(DATA_RAM, LCDRed);
			lcdWriteData(LCDRed);   
         }
    }
}

void setup_LCD(void)
{
    
    LPC_GPIO0 ->FIODIR &= (0<<19);
    LPC_GPIOINT ->IO0IntEnR  |= (1<<19);
    PIN_Configure (0, 19, 0, PIN_PINMODE_PULLUP , 0);
    //NVIC_EnableIRQ(EINT3_IRQn);
    lcdConfiguration();
    init_ILI9325();
	LCD_Reset();
    
}





void drawBall()
{
	int i,j;
    int r=5;
	int size = 2*r;
    int xCenter = kulka.x;
    int yCenter = kulka.y;
    int x,y;
    
    for(i=0; i<size-1; i++)
    {
         for(j=1 ; j<size; j++)
         {
					   
            lcdSetCursor(i+kulka.prevX-size/2, j+kulka.prevY-size/2 );
			lcdWriteReg(DATA_RAM, LCDGreen);
			lcdWriteData(LCDGreen);   
						 
		}
    }
    
	for(i=0; i<size; i++)
    {
         for(j=1 ; j<size; j++)
         {
             x = i- size/2;
             y = j- size/2;
             if(round(x*x) + round(y*y) < round(r*r) )
             {
				lcdSetCursor(i+xCenter-size/2-1, j+yCenter-size/2 );
				lcdWriteReg(DATA_RAM, LCDBlack);
                lcdWriteData(LCDBlack);   
			 }
		 }
    }

		
}



void rysujPlansze()
{

    int i,j;
    int x,y;
    int r1 = 80  + diff/2;
    int r2 = 118 - diff/2;
    for(i=0; i<240; i++)
    {
         for(j=30; j<290; j++)
         {
            x = i - 120 ;
            y = j- 160;
            if( (x*x + y*y > r1 * r1) && (x*x + y*y < r2*r2 ))
            {
             lcdSetCursor(i, j );
             lcdWriteReg(DATA_RAM, LCDGreen);
			 lcdWriteData(LCDGreen);  
            }
              
         }
    }
    
    
    for(i=200; i<240; i++)
    {
         for(j=158; j<162; j++)
         {
             lcdSetCursor(i, j );
             lcdWriteReg(DATA_RAM, LCDRed);
			 lcdWriteData(LCDRed);  
         }
    }
    
    for(i=200+ diff/2; i<240-diff/2; i++)
    {
         for(j=162; j<166; j++)
         {
             lcdSetCursor(i, j );
             lcdWriteReg(DATA_RAM, LCDYellow);
			 lcdWriteData(LCDYellow);  
         }
    }
    unsigned char napis2[20] = "Poziom trudnosci:\0";
    unsigned char * poziom_trudnosci;
    if(diff == 0)
    {
        poziom_trudnosci = "  Przedszkolak\0";
    }
    else if(diff == 5)
    {
        poziom_trudnosci = "     Latwy    \0";
    
    }
    else if(diff == 10)
    {
        poziom_trudnosci = "    Normalny  \0";
    }
    else if(diff == 15)
    {
        poziom_trudnosci = "   Dark Souls \0";
    }
    else 
    {
        poziom_trudnosci = "       ???    \0";
    }
    
    LCD_ascii_write(0, napis2, 45, 150, LCDBlack);
    LCD_ascii_write(0, poziom_trudnosci, 50, 170, LCDBlack);
    
}

void wygrales()
{
    int x,y,i,j;
    int r1 = 80  + diff/2;
    int r2 = 118 - diff/2;
    unsigned char napis2[9] = "Wygrales";
    if(diff < 15)
        diff+=5;

    LCD_ascii_write(0, napis2, 80, 120, LCDYellow);
    for(i=0; i<240; i++)
    {
         for(j=30; j<290; j++)
         {
            x = i - 120 ;
            y = j- 160;
            if( (x*x + y*y > r1 * r1) && (x*x + y*y < r2*r2 ))
            {
             lcdSetCursor(i, j );
             lcdWriteReg(DATA_RAM, LCDYellow);
			 lcdWriteData(LCDYellow);  
            }
              
         }
    }
    
    
    delay(1000000);
    LCD_Reset();
    rysujPlansze();
    BallInit();
}


void moveBall(int Vx, int Vy)
{
    kulka.prevX = kulka.x;
    kulka.prevY = kulka.y;
    kulka.x += Vx;
    kulka.y += Vy;
    int x, y;
    int r1 = 84 + diff/2;
    int r2 = 115 - diff/2;
    x = kulka.x - 120;
    y = kulka.y - 160;
    unsigned char napis[11]  = "Przegrales"; 

    //wygrana bo meta
    if( (kulka.x >=195 && kulka.x < 245) && (kulka.y >= 165 && kulka.y <=167) )
    {
           wygrales();
    }
    
    //przegrana przy pierscieniu
    if(! ( (x*x + y*y > r1 * r1) && (x*x + y*y < r2*r2 )) )
    {
           if(diff >0)
            diff-=5;
           LCD_ascii_write(0, napis, 80, 120, LCDBlack);
           delay(1000000);
           LCD_Reset();
           rysujPlansze();
           BallInit();
    }
    
    //przegrana bo zla strona mety
    if( (kulka.x >=195 && kulka.x < 245) && (kulka.y >=154 && kulka.y <=167) )
    {
           if(diff > 0)
            diff-=5;

           LCD_ascii_write(0, napis, 80, 120, LCDBlack);
           delay(1000000);
           LCD_Reset();
           rysujPlansze();
           BallInit();
    }
    

    
        
}

void BallInit(void)
{
    kulka.x=220;
    kulka.y=150;
    kulka.prevX =220;
    kulka.prevY =150;
}


void EINT0_IRQHandler(void)
{
    delay(200);
    if(diff< 15)
        diff+=5;
    else diff=0;
    LCD_Reset();
    rysujPlansze();
    startFlag=0;
    BallInit();
    LPC_SC ->EXTINT |= (1<<0);
    
}

void EINT1_IRQHandler(void)
{
    unsigned char* napis;
    int i,j;
    startFlag = 1;
    for(i=100; i<150; i++)
    {
         for(j=120; j<140; j++)
         {
             lcdSetCursor(i, j );
             lcdWriteReg(DATA_RAM, LCDRed);
			 lcdWriteData(LCDRed);  
         }
    }
    rysujPlansze();
    BallInit();
    
    LPC_SC ->EXTINT |= (1<<0);
}


int main(void){
    

    setup_LCD();

    acelerometr_setup();
    BallInit();


    rysujPlansze();
    
    PIN_Configure (2, 10, 1, PIN_PINMODE_PULLUP , 0); 
    LPC_SC ->EXTINT |= (1<<0);
    LPC_SC ->EXTMODE |= (1<<0);
    LPC_SC -> EXTMODE |= (1<<0);
    NVIC_EnableIRQ(EINT0_IRQn);
    
    PIN_Configure (2, 11, 1, PIN_PINMODE_PULLUP , 0); 
    LPC_SC ->EXTINT |= (1<<1);
    LPC_SC ->EXTMODE |= (0<<1);
    LPC_SC -> EXTMODE |= (1<<1);
    NVIC_EnableIRQ(EINT1_IRQn);
    
    
	while(1)
    {
        if(startFlag)
        {
            uint16_t x1 = SSP0_write( (1<<15) + 0x3200); //os x niskie bity
            delay(10);
            uint16_t y1 = SSP0_write( (1<<15) + 0x3400);
            delay(10);
            uint16_t x2 = SSP0_write( (1<<15) + 0x3312); //os x wysokie bity
            delay(10);
            uint16_t y2 = SSP0_write( (1<<15) + 0x3512);
            x1 &= 0xFF; // zerowanie pierwszych 8 bitow
            y1 &= 0xFF; //jw
            x2 &= 0xFF;
            y2 &= 0xFF;
        
            short int wynikX = x1 + (x2 << 8);
            short int wynikY = y1 + (y2 << 8);
      
            drawBall();
            moveBall( round(-wynikX/100) , round(wynikY/100) );
        }
    }

    
    return 0;
}

