#include "N76E003.h"
#include "Common.h"
#include "Delay.h"
#include "SFR_Macro.h"
#include "Function_define.h"

#define PROJECTOR_ON            V15V_PWR_ON
#define PROJECTOR_OFF           V15V_PWR_OFF

#define BOARD_ON                CORE_PWR_ON
#define BOARD_OFF               CORE_PWR_OFF

#define I2C_SLAVE_ADDR          (0x2C << 1)
#define I2C_IDLE                0xf8
#define I2C_BYTE_CMD            0

#define I2C_CMD_RESET_PROJECTOR 0xE0
#define I2C_CMD_RESET_NANO      0xE1

#define I2C_RX_LEN              32
#define I2C_TX_LEN              32

UINT8 rxdata[I2C_RX_LEN];
UINT8 rxcnt = 0;

UINT8 txdata[I2C_TX_LEN];
UINT8 txcnt = 0;
/************************************************************************************************************
*    I2C interrupt sub-routine
************************************************************************************************************/
void I2C_ISR(void) interrupt 6
{
    switch (I2STAT)
    {
        case 0x00:
            STO = 1;
            break;

        case 0x60:
            AA = 1;
            break;
				
        case 0x68:
			P02 = 0;
            while(1);
            break;

        case 0x80:
            rxdata[rxcnt] = I2DAT;
            rxcnt++;

            if (rxcnt == I2C_RX_LEN)
                AA = 0;
            else
                AA = 1;
            break;

        case 0x88:
            rxdata[rxcnt] = I2DAT;
            rxcnt = 0;
            AA = 1;
            break;

        case 0xA0:
            AA = 1;
            break;

        case 0xA8:
            I2DAT = txdata[txcnt];
            txcnt++;
            AA = 1;
            break;
        
        case 0xB8:
            I2DAT = txdata[txcnt];
            txcnt++;
            AA = 1;
            break;

        case 0xC0:
            AA = 1;
            break; 

        case 0xC8:
            AA = 1;
            break;        
    }

    SI = 0;
    while(STO);
}
/************************************************************************************************************
*    WDT interrupt sub-routine
************************************************************************************************************/
void WDT_ISR (void)   interrupt 10
{
		clr_WDTF;
		set_WDCLR;
}

//========================================================================================================
void Init_I2C()
{
    P13_Quasi_Mode;                         //set SCL (P13) is Quasi mode
    P14_Quasi_Mode;                         //set SDA (P14) is Quasi mode
    
    SDA = 1;                                //set SDA and SCL pins high
    SCL = 1;
    
    set_P0SR_6;                             //set SCL (P06) is  Schmitt triggered input select.
    
    set_EI2C;                               //enable I2C interrupt by setting IE1 bit 0

    I2ADDR = I2C_SLAVE_ADDR;                //define own slave address
    set_I2CEN;                              //enable I2C circuit
    set_AA;
}

//========================================================================================================
void Init_IO()
{
    Set_All_GPIO_Quasi_Mode;
	P03_PushPull_Mode;
	P04_PushPull_Mode;
    CORE_PWR_ON;
    V15V_PWR_ON;
}

//========================================================================================================
void Init_WDT()
{
    TA=0xAA; TA=0x55; WDCON=0x06;  	// Setting WDT prescale 
    set_WDTR;                       // WDT run
    set_WDCLR;						// Clear WDT timer
    set_EWDT;
}

//========================================================================================================
void mymemset(UINT8 *dst, UINT8 val, UINT8 len)
{
    UINT8 i;

    for (i = 0; i < len; i++) {
        *dst++ = val;
    }
}

//========================================================================================================
void Rx_Reset()
{
    mymemset(rxdata, 0, I2C_RX_LEN);
    rxcnt = 0;
}

//========================================================================================================
void Tx_Reset()
{
    mymemset(txdata, 0, I2C_TX_LEN);
    txcnt = 0;
}

//========================================================================================================
void main(void)
{
    Init_IO();
    Init_I2C();
    Init_WDT();
    set_EA;

    while (1)
    {
        if (I2STAT == I2C_IDLE) 
        {
            switch (rxdata[I2C_BYTE_CMD])
            {
                case I2C_CMD_RESET_PROJECTOR:
                    PROJECTOR_OFF;
                    Rx_Reset();
                    Timer0_Delay1ms(5000);
                    PROJECTOR_ON;
                    break;

                case I2C_CMD_RESET_NANO:
                    BOARD_OFF;
                    Rx_Reset();
                    Timer0_Delay1ms(5000);
                    BOARD_ON;
                    break;

                default:
                    Rx_Reset();
                    break;
            }
        }
		Timer0_Delay1ms(100);
    }
}

