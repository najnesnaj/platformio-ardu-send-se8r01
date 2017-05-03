/*********************************************************************
Based of nrf24l01 code from http://www.elecfreaks.com
And Se8r01 code supplied by supplier,
And the final piece from this post:http://bbs.ai-thinker.com/forum.php?mod=viewthread&tid=407 
Cant read the language but if you can say thank you from me!!
All the pieces put together by me, Stefan (f2k) 151215                          
*********************************************************************/
  
#include "API.h"


//This code uses simple software spi so you can change these
//to whatever suits you best.
#define CEq       8 
// CE_BIT:   Digital Input     Chip Enable Activates RX or TX mode
#define CSNq      9 
// CSN BIT:  Digital Input     SPI Chip Select
#define SCKq      10 
// SCK BIT:  Digital Input     SPI Clock
#define MOSIq     11 
// MOSI BIT: Digital Input     SPI Slave Data Input
#define MISOq     12
// MISO BIT: Digital Output    SPI Slave Data Output, with tri-state option


byte SE8R01_DR_2M=0;      //choose 1 of these to set the speed
byte SE8R01_DR_1M=0;
byte SE8R01_DR_500K=1;
byte gtemp[5];

struct dataStruct{
  unsigned long counter;
  byte rt;
}myData;




#define ADR_WIDTH    4   // 4 unsigned chars TX(RX) address width
#define PLOAD_WIDTH  sizeof(myData)  // sizeof(myData) unsigned chars TX payload


unsigned char TX_ADDRESS[ADR_WIDTH]  = 
{
  0xb3,0x43,0x10,0x10
}; // Define a static TX address

unsigned char tx_buf[PLOAD_WIDTH] = {0};




  //**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{
 
  digitalWrite(CEq, 0);			// chip enable
  digitalWrite(CSNq, 1);                 // Spi disable	
}

/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char SPI_RW(unsigned char Byte)
{
  unsigned char i;
  for(i=0;i<8;i++)                      // output 8-bit
  {
    if(Byte&0x80)
    {
      digitalWrite(MOSIq, 1);    // output 'unsigned char', MSB to MOSI
    }
    else
    {
      digitalWrite(MOSIq, 0);
    }
    digitalWrite(SCKq, 1);                      // Set SCK high..
    Byte <<= 1;                         // shift next bit into MSB..
    if(digitalRead(MISOq) == 1)
    {
      Byte |= 1;       	                // capture current MISO bit
    }
    digitalWrite(SCKq, 0);         	// ..then set SCK low again
  }
  return(Byte);           	        // return read unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  digitalWrite(CSNq, 0);                   // CSN low, init SPI transaction
  status = SPI_RW(reg);                   // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(CSNq, 1);                   // CSN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  digitalWrite(CSNq, 0);           // CSN low, initialize SPI communication...
  SPI_RW(reg);                   // Select register to read from..
  reg_val = SPI_RW(0);           // ..then read register value
  digitalWrite(CSNq, 1);          // CSN high, terminate SPI communication

  return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSNq, 0);                  // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(CSNq, 1);                   // Set CSN high again

  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSNq, 0);                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSNq, 1);                  // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/


void rf_switch_bank(unsigned char bankindex)
{
    unsigned char temp0,temp1;
    temp1 = bankindex;

    temp0 = SPI_RW(iRF_BANK0_STATUS);

    if((temp0&0x80)!=temp1)
    {
        SPI_RW_Reg(iRF_CMD_ACTIVATE,0x53);
    }
}




void SE8R01_Calibration()
{
        unsigned char temp[5];
        rf_switch_bank(iBANK0);
        temp[0]=0x03;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_CONFIG,temp, 1);

        temp[0]=0x32;

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_CH, temp,1);



if (SE8R01_DR_2M==1)
  {temp[0]=0x48;}
else if (SE8R01_DR_1M==1)
  {temp[0]=0x40;}
else  
  {temp[0]=0x68;}   
  
  SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,temp,1);
  temp[0]=0x77;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_PRE_GURD, temp,1);
  
        rf_switch_bank(iBANK1);
        temp[0]=0x40;
        temp[1]=0x00;
        temp[2]=0x10;
if (SE8R01_DR_2M==1)
          {temp[3]=0xE6;}
else
    {temp[3]=0xE4;}

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp, 4);

        temp[0]=0x20;
        temp[1]=0x08;
        temp[2]=0x50;
        temp[3]=0x40;
        temp[4]=0x50;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp, 5);

        temp[0]=0x00;
        temp[1]=0x00;
if (SE8R01_DR_2M==1)
       { temp[2]=0x1E;}
else
   { temp[2]=0x1F;}

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_IF_FREQ, temp, 3);
       
if (SE8R01_DR_2M==1)
       { temp[0]=0x29;}
else
 { temp[0]=0x14;}

        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp, 1);

        temp[0]=0x00;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_LOW,temp,1);

        temp[0]=0x7F;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_DAC_CAL_HI,temp,1);

        temp[0]=0x02;
        temp[1]=0xC1;
        temp[2]=0xEB;            
        temp[3]=0x1C;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);
//--
        temp[0]=0x97;
        temp[1]=0x64;
        temp[2]=0x00;
        temp[3]=0x81;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp, 4);
        rf_switch_bank(iBANK0);
        
        digitalWrite(CEq, 1); 
        delayMicroseconds(30);
        digitalWrite(CEq, 0);  

        delay(50);                            // delay 50ms waitting for calibaration.

        digitalWrite(CEq, 1); 
        delayMicroseconds(30);
        digitalWrite(CEq, 0); 

        delay(50);                            // delay 50ms waitting for calibaration.
}


void SE8R01_Analog_Init()           //SE8R01 初始化
{    


        gtemp[0]=0x28;
        gtemp[1]=0x32;
        gtemp[2]=0x80;
        gtemp[3]=0x90;
        gtemp[4]=0x00;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_SETUP_VALUE, gtemp, 5);
        delay(2);
  
  unsigned char temp[5];   
 
  rf_switch_bank(iBANK1);
   
        temp[0]=0x40;
        temp[1]=0x01;               
        temp[2]=0x30;               
if (SE8R01_DR_2M==1)
       { temp[3]=0xE2; }              
else
 { temp[3]=0xE0;}
   
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_PLL_CTL0, temp,4);

        temp[0]=0x29;
        temp[1]=0x89;
        temp[2]=0x55;                     
        temp[3]=0x40;
        temp[4]=0x50;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_CAL_CTL, temp,5);

if (SE8R01_DR_2M==1)
       { temp[0]=0x29;}
else
 { temp[0]=0x14;}
         
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FDEV, temp,1);

    temp[0]=0x55;
    temp[1]=0xC2;
    temp[2]=0x09;
    temp[3]=0xAC;  
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RX_CTRL,temp,4);

    temp[0]=0x00;
    temp[1]=0x14;
    temp[2]=0x08;   
    temp[3]=0x29;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_FAGC_CTRL_1, temp,4);

    temp[0]=0x02;
    temp[1]=0xC1;
    temp[2]=0xCB;  
    temp[3]=0x1C;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_AGC_GAIN, temp,4);

    temp[0]=0x97;
    temp[1]=0x64;
    temp[2]=0x00;
    temp[3]=0x01;
    SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_RF_IVGEN, temp,4);
    
        gtemp[0]=0x2A;
        gtemp[1]=0x04;
        gtemp[2]=0x00;
        gtemp[3]=0x7D;
        SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK1_TEST_PKDET, gtemp, 4);

         rf_switch_bank(iBANK0);
}

void SE8R01_Init()  //1M 2M 和NRF24L01通用
{
  
        SE8R01_Calibration();   //校准
        SE8R01_Analog_Init();   //模拟部分电路初始化
         
if (SE8R01_DR_2M==1)
{  gtemp[0]=B01001111; }     //2MHz,+5dbm
else if  (SE8R01_DR_1M==1)
{  gtemp[0]=B01000111;  }     //1MHz,+5dbm
else
  {gtemp[0]=B01101111;  }     //500K,+5dbm

SPI_Write_Buf(iRF_CMD_WRITE_REG|iRF_BANK0_RF_SETUP,gtemp,1);

        SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_AA, 0x01);          //enable auto acc on pip 1
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_EN_RXADDR, 0x01);      //enable pip 1
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_SETUP_AW, 0x02);        //4 byte adress
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_SETUP_RETR, 0x08);        //lowest 4 bits 0-15 rt transmisston higest 4 bits 256-4096us Auto Retransmit Delay
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_RF_CH, 40);
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_DYNPD, 0x01);          //pipe0 pipe1 enable dynamic payload length data
        SPI_RW_Reg(WRITE_REG|iRF_BANK0_FEATURE, 0x07);        // enable dynamic paload lenght; enbale payload with ack enable w_tx_payload_noack
        SPI_RW_Reg(WRITE_REG + CONFIG, 0x3E);
 SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH);  //from tx
 SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, ADR_WIDTH); // Use the same address on the RX device as the TX device SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width       

digitalWrite(CEq, 1);  


}

void setup() 
{
  
  pinMode(CEq,  OUTPUT);
  pinMode(SCKq, OUTPUT);
  pinMode(CSNq, OUTPUT);
  pinMode(MOSIq,  OUTPUT);
  pinMode(MISOq, INPUT);
 

Serial.begin(115200);
  init_io();                        // Initialize IO port
  SPI_RW_Reg(FLUSH_TX,0); 
  unsigned char status=SPI_Read(STATUS);
  Serial.print("status = ");    
  Serial.println(status,HEX);     
  Serial.println("*******************Radio starting*****************");

 SE8R01_Init();

}

void loop() 
{

  memcpy(tx_buf, &myData,sizeof(myData));
  unsigned char status = SPI_Read(STATUS);   
  SPI_Write_Buf(iRF_CMD_WR_TX_PLOAD,tx_buf,PLOAD_WIDTH);
  SPI_RW_Reg(WRITE_REG+STATUS,0xff); // clear RX_DR or TX_DS or MAX_RT interrupt flag
  delay(10);              //sending data
  myData.rt=SPI_Read(OBSERVE_TX);        //count resends
  Serial.print("status ");
  Serial.print(status,HEX);
  Serial.print(" rt packets: ");
  Serial.println(myData.rt&B00001111);
  myData.counter++;


delay(500);   
}
