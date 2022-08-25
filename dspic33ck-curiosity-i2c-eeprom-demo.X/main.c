/*
© [2022] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/i2c_host/i2c1.h"
#include "mcc_generated_files/uart/uart1.h"
#include "mcc_generated_files/system/clock.h"

//EEPROM Specific definitions
#define CLIENT_NODE_ADDRESS 0x50U
#define BLOCK_ADDRESS 0x1U // 0 - 3 for 24C08 and 0 - 7 for 24C016
#define ADDRESS_SIZE 1U
#define PAGE_SIZE 16U
#define BLOCK_SIZE 256U


//EEPROM Macros
#define CLIENT_BLOCK_ADDRESS (CLIENT_NODE_ADDRESS | ((BLOCK_ADDRESS)<<1))
#define MAX_PAGES ((BLOCK_SIZE)/(PAGE_SIZE))


#define FCY             CLOCK_SystemFrequencyGet()
#include <libpic30.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

void I2C_Callback(void);
uint8_t EEPROMPageWrite(uint8_t pageAddress);
void EEPROMReadPages(uint8_t pageAddress, uint8_t numPages);


static void PrintWelcomeMessage(void);
static void PrintFeaturesMessage(void);
static void resetErrorStatus();
static bool ErrorHandler(void);

/*
    Main application
*/
bool i2cHostErrorNoneFlag = false;
bool i2cHostNackFlag = false;
bool i2cHostBusCollisionFlag = false;



int main(void)
{
    bool isExit = false;
    uint8_t i = 0, address = 0, numPagesWritten;
    SYSTEM_Initialize();
    
    I2C_Host.Initialize();
    I2C_Host.HostCallbackRegister(&I2C_Callback);
    
    PrintWelcomeMessage();
    PrintFeaturesMessage();
    
    printf("* Type the characters that has to be stored in EEPROM\r\n");
    printf("* Press enter key once to jump to next page and press it twice to exit recording\r\n\r\n");
    
    //Write the UART data to EEPROM, page wise
    while((i < MAX_PAGES) && (isExit != true))
    {
        address = i*PAGE_SIZE;
        if(EEPROMPageWrite(address)==0)
        {
            isExit = true;
            break;
        }
        printf("\r\n");
        i++; 

    }

    numPagesWritten = i;


    
    printf("\r\n\r\n");
    
    // Read page from EEPROM
    if(numPagesWritten != 0)
    {
        printf("Reading the stored data from EEPROM\r\n\r\n");
        EEPROMReadPages(0,numPagesWritten);
    }
    
    while(1)
    {
    }    
}

void I2C_Callback(void)
{
    enum I2C_HOST_ERROR status = I2C1_ErrorGet();
    
    if(status == I2C_HOST_ERROR_NONE)
    {
        i2cHostErrorNoneFlag = true;
    }
    else if(status == I2C_HOST_ERROR_NACK)
    {
        i2cHostNackFlag = true;
    }
    else if(status == I2C_HOST_ERROR_BUS_COLLISION)
    {
        i2cHostBusCollisionFlag = true;
    }
}

static void PrintWelcomeMessage(void)
{
    printf("\r\n");
    printf("*******************************************************\r\n");
    printf("dsPIC33CK256MP508 Curiosity I2C Demo\r\n");
    printf("*******************************************************\r\n");
}

static void PrintFeaturesMessage(void)
{
    
    printf("DEMO KEY FEATURES:\r\n");
    printf("1. I2C communication using dsPIC33CK256MP508\r\n");
    printf("2. 16 byte page write to I2C EEPROM and page read of same data\r\n");
    printf("3. Upto 256 bytes (block size of 24C08) of data can be recorded\r\n\r\n");
}

static void resetErrorStatus()
{
    i2cHostErrorNoneFlag = false;
    i2cHostNackFlag = false;
    i2cHostBusCollisionFlag = false;
}

static bool ErrorHandler(void)
{
    if((i2cHostErrorNoneFlag != true) && (i2cHostNackFlag == false) && (i2cHostBusCollisionFlag == false))
    {
        resetErrorStatus();
        return true;
    }
    
    return false;
}

uint8_t EEPROMPageWrite(uint8_t pageAddress)
{
    uint8_t j = 0,readData = 0;
    uint8_t clientWriteData[ADDRESS_SIZE + PAGE_SIZE] = {};
    clientWriteData[0] = pageAddress;
    printf("* Address: 0x%x\tData: ",pageAddress);
    readData = UART_Serial.Read();
    UART_Serial.Write(readData);
    while(readData != '\r')
    {
        if(readData == '\b')
        {
            j--;
        }
        else
        {
            clientWriteData[ADDRESS_SIZE + j] = readData;
            j++;
        }
        if(j == PAGE_SIZE)
        {
            break;
        }
        readData = UART_Serial.Read();
        UART_Serial.Write(readData);
    }
    
    //Write page to EEPROM
    I2C_Host.Write(CLIENT_BLOCK_ADDRESS, clientWriteData, (ADDRESS_SIZE + PAGE_SIZE));
    
    //EEPROM internal page write delay 
    __delay_ms(1);
    if(ErrorHandler() == true)
    {
        j = 0;
        printf("\r\n***ERROR WRITING EEPROM***\r\n");
    }
    return j;
}

void EEPROMReadPages(uint8_t pageAddress, uint8_t numPages)
{
    uint8_t i=0, j = 0;
    uint8_t clientWriteData[ADDRESS_SIZE + PAGE_SIZE] = {};     
    uint8_t clientReadData[128] = {0};
    
    clientWriteData[0] = pageAddress;
    
    for(i=0;i<numPages;i++)
    {
        clientWriteData[0] = PAGE_SIZE * i;
        
        //Read page from EEPROM
        I2C_Host.WriteRead(CLIENT_BLOCK_ADDRESS, clientWriteData, 1, clientReadData, PAGE_SIZE);
        
        printf("* Address: 0x%x\tData: ",clientWriteData[0]);
        for(j=0;j<PAGE_SIZE;j++)
        {
            UART_Serial.Write(clientReadData[j]);
        }
        printf("\r\n");    
    }
}