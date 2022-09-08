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

#ifndef EEPROM_H
#define	EEPROM_H

#include <stdint.h>
#include "eeprom_config.h"
#include "eeprom_types.h"


//EEPROM macros
#define EEPROM_CLIENT_BLOCK_ADDRESS (EEPROM_NODE_ADDRESS | ((EEPROM_BLOCK_ADDRESS)<<1))
#define EEPROM_MAX_PAGES ((EEPROM_BLOCK_SIZE)/(EEPROM_PAGE_SIZE))


void EEPROM_Initialize(void);
enum EEPROM_WRITE_STATUS EEPROM_PageWrite(union EEPROM_WRITE_BUFFER *writeBuffer);
enum EEPROM_READ_STATUS EEPROM_ReadPages(uint8_t pageAddress, uint8_t *readBuffer, uint8_t numPages);

#endif	/* EEPROM_H */

