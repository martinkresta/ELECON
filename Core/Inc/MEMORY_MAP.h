/*    File: MEMORY_MAP.h
    Author: MartinKresta
      Date: 17.11.2025
     Brief: Default definitions of memory layout. Defines a location of BSW, ASW header parts, And ASW executable start address
 */

#ifndef INC_MEMORY_MAP_H_
#define INC_MEMORY_MAP_H_

#include "main.h"
#include <stdint.h>

#define FLASH_ADDRESS_OFFSET                 0x8000000
#define EXEC_CODE_OFFSET                     0x200        // Executable code starts 256B after a Sw header struct / VTOR table alignment


// Bootloader addresses
//#define ADDRESS_OF_BOOTLOADER_HEADER                0x08000000
#define ADDRESS_OF_DEFAULT_BOOTLOADER_CODE_FIRST    0x08000000                    // Bootloader first code address
#define ADDRESS_OF_DEFAULT_BOOTLOADER_CODE_LAST     0x08003FFF                    // Bootloader last code address  (Max value)
#define ADDRESS_OF_DEFAULT_BOOTLOADER_START         0x08000000                    // Bootloader executable code start address
#define ADDRESS_OF_DEFAULT_BOOTLOADER_CRC_FIRST     0x08000000                     // CRC start address computing (with parameters without CRC value)
#define ADDRESS_OF_DEFAULT_BOOTLOADER_CRC_LAST      0x08004FFF                          // CRC stop address computing (Max value)


// Application addresses
#define ADDRESS_OF_APP_HEADER                       0x08004000
#define ADDRESS_OF_DEFAULT_APP_CODE_FIRST           0x08004000
#define ADDRESS_OF_DEFAULT_APP_CODE_LAST            0x0803FFFF
#define ADDRESS_OF_DEFAULT_APP_START                0x08004000 + EXEC_CODE_OFFSET
#define ADDRESS_OF_DEFAULT_APP_CRC_FIRST            0x08004000 + EXEC_CODE_OFFSET   // CRC start address computing (with parameters without CRC value)
#define ADDRESS_OF_DEFAULT_APP_CRC_LAST             0x0803FFFF                          // CRC stop address computing (Max value)


// Structure of SW header located at the start of application memory region  (256B)
typedef struct __attribute__ ((packed))
{
  const uint32_t _CRC;
  const uint32_t HW_ID;
  const uint32_t SW_VERSION;
  const uint32_t DATE;
  const uint8_t  VERSTR[6];     // human readable ASCII string Major.Minor =>  MMM.mm
  const uint8_t  RESERVED[10];
  const uint8_t  DEVICE[20];    // human readable ASCII string
  const uint32_t CODE_FIRST_ADDRESS;   // location of entire image in flash
  const uint32_t CODE_LAST_ADDRESS;
  const uint32_t START_ADDRESS;       // BTL will jump to this address
  const uint32_t CRC_FIRST_ADDRESS;   // start of part covered by CRC
  const uint32_t CRC_LAST_ADDRESS;    // end of part covered by CRC
  const uint32_t CRC_START_VALUE;
  const uint8_t  DUMMY[200];     // placeholder to 256bytes
}sSwInfoHeader;


#endif /* INC_MEMORY_MAP_H_ */
