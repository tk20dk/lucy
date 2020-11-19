#ifndef __DS18B20_H__
#define __DS18B20_H__

#include <cstdint>
#include "main.h"

struct TDs18b20
{
  static uint8_t const CMD_READ_TIME_SLOT     = 0xff;
  static uint8_t const CMD_SEARCH_ROM         = 0xf0;
  static uint8_t const CMD_READ_ROM           = 0x33;
  static uint8_t const CMD_MATCH_ROM          = 0x55;
  static uint8_t const CMD_SKIP_ROM           = 0xcc;
  static uint8_t const CMD_ALARM_SEARCH       = 0xec;
  static uint8_t const CMD_CONVERT            = 0x44;
  static uint8_t const CMD_WRITE_SCRATCHPAD   = 0x4e;
  static uint8_t const CMD_READ_SCRATCHPAD    = 0xbe;
  static uint8_t const CMD_COPY_SCRATCHPAD    = 0x48;
  static uint8_t const CMD_RECALL_EEPROM      = 0xb8;
  static uint8_t const CMD_READ_POWER_SUPPPLY = 0xb4;

  static void Init();
  static int16_t GetTemperature( uint32_t const Index );

private:
  static void Power( bool const Mode );
  static bool Reset( uint32_t const Index );
  static void Enable( uint32_t const Index );
  static bool CalcCRC( uint8_t const *const Data, uint32_t const Count );
  static bool Transmit( uint32_t const Index, uint8_t *const Data, uint32_t const Count );
  static void PowerDelay( uint32_t const Interval );
  static bool CopyScratchpad( uint32_t const Index );
  static bool ReadScratchpad( uint32_t const Index, uint8_t *const Data );
  static bool WriteScratchpad( uint32_t const Index, uint8_t const Data );
  static bool ReadPowerSupply( uint32_t const Index );
  static uint8_t TxRx( uint8_t const Data );
};

#endif // __DS18B20_H__
