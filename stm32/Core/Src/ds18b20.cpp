#include "main.h"
#include "system.h"
#include "ds18b20.h"
#include <gpio.h>
#include <usart.h>


auto const USARTx = reinterpret_cast< intptr_t >( USART2 );
typedef TDriverUsart< USARTx > TUsart;

auto const MUX_A0_Port = reinterpret_cast< intptr_t >( MUX_A0_GPIO_Port );
typedef TGpio< MUX_A0_Port, MUX_A0_Pin > TGpioMUX_A0;
auto const MUX_A1_Port = reinterpret_cast< intptr_t >( MUX_A1_GPIO_Port );
typedef TGpio< MUX_A1_Port, MUX_A1_Pin > TGpioMUX_A1;
auto const MUX_A2_Port = reinterpret_cast< intptr_t >( MUX_A2_GPIO_Port );
typedef TGpio< MUX_A2_Port, MUX_A2_Pin > TGpioMUX_A2;
auto const MUX_EN_Port = reinterpret_cast< intptr_t >( MUX_EN_GPIO_Port );
typedef TGpio< MUX_EN_Port, MUX_EN_Pin > TGpioMUX_EN;
auto const MUX_DATA_Port = reinterpret_cast< intptr_t >( MUX_DATA_GPIO_Port );
typedef TGpio< MUX_DATA_Port, MUX_DATA_Pin > TGpioMUX_DATA;


void TDs18b20::Init()
{
  TGpioMUX_DATA::Set();

  Power( true );

  for( auto Index = 0; Index < 8; Index++ )
  {
    Reset( Index );
    ReadPowerSupply( Index );
    WriteScratchpad( Index, 0x00 );
    CopyScratchpad( Index );
  }

  Power( false );
}

int16_t TDs18b20::GetTemperature( uint32_t const Index )
{
  Power( true );

  uint8_t Data0[ 2 ] = { CMD_SKIP_ROM, CMD_CONVERT };
  bool Status = Transmit( Index, Data0, sizeof( Data0 ));

  PowerDelay( 95 );

  uint8_t Data[ 11 ] =
  {
    CMD_SKIP_ROM,
    CMD_READ_SCRATCHPAD,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT
  };
  Status = Status && Transmit( Index, Data, sizeof( Data ));

  Power( false );

  Status = Status && CalcCRC( &Data[ 2 ], 9 );
  if( Status == false )
  {
    return 0;
  }

  int16_t const Temp = ( Data[ 3 ] << 8 ) + Data[ 2 ];
  return ( Temp * 10 ) / 16;
}

void TDs18b20::PowerDelay( uint32_t const Interval )
{
  PinModeOutput( MUX_DATA_GPIO_Port, MUX_DATA_Pin );
  PinModePushPull( MUX_DATA_GPIO_Port, MUX_DATA_Pin );
  TSystem::StopDelay( Interval );
  PinModeOpenDrain( MUX_DATA_GPIO_Port, MUX_DATA_Pin );
  PinModeAlternate( MUX_DATA_GPIO_Port, MUX_DATA_Pin );
}

bool TDs18b20::ReadScratchpad( uint32_t const Index, uint8_t *const Data )
{
  uint8_t TxData[ 11 ] =
  {
    CMD_SKIP_ROM,
    CMD_READ_SCRATCHPAD,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT,
    CMD_READ_TIME_SLOT
  };

  bool Status = Transmit( Index, TxData, sizeof( TxData ));
  Status = Status && CalcCRC( &TxData[ 2 ], 9 );
  return Status;
}

//  9  93.75ms 0x00
// 10 187.50ms 0x20
// 11 375.00ms 0x40
// 12 750.00ms 0x60
bool TDs18b20::WriteScratchpad( uint32_t const Index, uint8_t const Mode )
{
  uint8_t Data[ 5 ] = { CMD_SKIP_ROM, CMD_WRITE_SCRATCHPAD, 0, 0, Mode };
  return Transmit( Index, Data, sizeof( Data ));
}

bool TDs18b20::CopyScratchpad( uint32_t const Index )
{
  uint8_t Data[ 2 ] = { CMD_SKIP_ROM, CMD_COPY_SCRATCHPAD };
  bool const Status = Transmit( Index, Data, sizeof( Data ));

  PowerDelay( 10 );

  return Status;
}

bool TDs18b20::ReadPowerSupply( uint32_t const Index )
{
  uint8_t Data[ 3 ] = { CMD_SKIP_ROM, CMD_READ_POWER_SUPPPLY, CMD_READ_TIME_SLOT };

  return Transmit( Index, Data, sizeof( Data ));
}

void TDs18b20::Power( bool const Mode )
{
  if( Mode )
  {
    PinModeAlternate( MUX_DATA_GPIO_Port, MUX_DATA_Pin );
    TGpioMUX_EN::Set();
  }
  else
  {
    PinModeAnalog( MUX_DATA_GPIO_Port, MUX_DATA_Pin );
    TGpioMUX_EN::Reset();
  }
}

void TDs18b20::Enable( uint32_t const Index )
{
  static uint8_t const Map[ 8 ] = { 0, 1, 2, 3, 7, 6, 5, 4 };

  Map[Index] & 0x01 ? TGpioMUX_A0::Set() : TGpioMUX_A0::Reset();
  Map[Index] & 0x02 ? TGpioMUX_A1::Set() : TGpioMUX_A1::Reset();
  Map[Index] & 0x04 ? TGpioMUX_A2::Set() : TGpioMUX_A2::Reset();
  TSystem::StopDelay( 2 );
}

bool TDs18b20::Transmit( uint32_t const Index, uint8_t *const Data, uint32_t const Count )
{
  bool Status = Reset( Index );

  for( auto Index = 0U; Status && ( Index < Count ); Index++ )
  {
    auto InByte = 0x00;
    auto const OutByte = Data[ Index ];
    for( auto BitIndex = 0; Status && ( BitIndex < 8 ); BitIndex++ )
    {
      uint8_t const Bit = OutByte & ( 1 << BitIndex ) ? 0xff : 0x00;
      InByte |= (( TxRx( Bit ) == 0xff ) << BitIndex );
    }
    Data[ Index ] = InByte;
  }

  return Status;
}

bool TDs18b20::CalcCRC( uint8_t const *const Data, uint32_t const Count )
{
  auto Crc = 0;
  for( auto Index = 0U; Index < Count; Index++ )
  {
    auto Byte = Data[ Index ];
    for( auto Bit = 0; Bit < 8; Bit++ )
    {
      auto const Tmp =( Byte ^ Crc ) & 1;
      Crc >>= 1;
      Byte >>= 1;
      if( Tmp )
      {
        Crc ^= 0x8c;
      }
    }
  }

  return Crc == 0;
}

bool TDs18b20::Reset( uint32_t const Index )
{
  Enable( Index );

  TUsart::SetBaudrate( 9600 );
  uint8_t const Data = TxRx( 0xf0 );
  TUsart::SetBaudrate( 115200 );

  return Data != 0xf0;
}

uint8_t TDs18b20::TxRx( uint8_t const Data )
{
  TUsart::Transmit( Data );
  return TUsart::Receive();
}
