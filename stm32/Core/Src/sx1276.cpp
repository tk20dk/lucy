#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "sx1276.h"
#include "sx1276-def.h"


auto const RADIO_CS_Port = reinterpret_cast< intptr_t >( RADIO_CS_GPIO_Port );
typedef TGpio< RADIO_CS_Port, RADIO_CS_Pin > TGpioRADIO_CS;
auto const RADIO_RESET_Port = reinterpret_cast< intptr_t >( RADIO_RESET_GPIO_Port );
typedef TGpio< RADIO_RESET_Port, RADIO_RESET_Pin > TGpioRADIO_RESET;

auto const SPIx = reinterpret_cast< intptr_t >( SPI1 );
typedef TDriverSpi<SPIx> TSpi;

TSx1276::TSx1276( uint32_t const BaseFreq, TCallback const Callback ) :
  Frequency( 0 ),
  FrequencyOffset( 0 ),
  BaseFreq( BaseFreq ),
  Callback( Callback )
{
}

bool TSx1276::Init()
{
  TSpi::Init();

  Reset();
  // Switch to LoRa mode.
  Sleep();

  auto const Version = ReadRegister( REG_VERSION );
  if( Version != 0x12 )
  {
    return false;
  }

  SetCRC( true );
  SetSyncWord( 0x12 );
  SetPreambleLength( 12 );

  SetTxPower( System.Config.TxPower );
  SetCodingRate( System.Config.CodingRate );
  SetSpreadingFactor( System.Config.SpreadingFactor );
  SetSignalBandwidth( System.Config.SignalBandwidth );
  SetChannel( System.Config.Channel );

  // Set overload current protection to 240mA
  WriteRegister( REG_OCP, 0x7f );

  // set base addresses
  WriteRegister( REG_FIFO_TX_BASE_ADDR, 0 );
  WriteRegister( REG_FIFO_RX_BASE_ADDR, 0 );

  // set LNA boost
  WriteRegister( REG_LNA, ReadRegister( REG_LNA ) | 0x03 );

  // Enable LowDataRateOptimize
//  WriteRegister( REG_MODEM_CONFIG3, 0x08 );

  // Explicit header mode
  WriteRegister( REG_MODEM_CONFIG_1, ReadRegister( REG_MODEM_CONFIG_1 ) & 0xfe );

  return true;
}

void TSx1276::Interrupt()
{
  auto const IrqFlags = ReadRegister( REG_IRQ_FLAGS );
  WriteRegister( REG_IRQ_FLAGS, IrqFlags );

  if( IrqFlags & IRQ_RX_DONE_MASK )
  {
    auto const HopChannel = ReadRegister( REG_HOP_CHANNEL );
    if( HopChannel & 0x40 )
    {
      if( IrqFlags & IRQ_CRC_ERROR_MASK )
      {
        Callback( TRadioEvent::CrcError );
      }
      else
      {
        Callback( TRadioEvent::RxDone );
      }
    }
    else
    {
      Callback( TRadioEvent::NoCrc );
    }
  }

  if( IrqFlags & IRQ_TX_DONE_MASK )
  {
    Callback( TRadioEvent::TxDone );
  }

  if( IrqFlags & IRQ_RX_TIMEOUT_MASK )
  {
    Callback( TRadioEvent::Timeout );
  }
}

void TSx1276::Reset()
{
  TGpioRADIO_RESET::Reset();
  LL_mDelay( 1 );
  TGpioRADIO_RESET::Set();
  LL_mDelay( 1 );
}

void TSx1276::Receive()
{
  WriteRegister( REG_PAYLOAD_LENGTH, 255 );
  WriteRegister( REG_DIO_MAPPING_1, 0x00 );
  WriteRegister( REG_IRQ_FLAGS_MASK, ~( IRQ_RX_DONE_MASK | IRQ_CRC_ERROR_MASK ));
  WriteRegister( REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS );
}

void TSx1276::Transmit( void const *const Buffer, uint16_t const Length )
{
  Idle();

  // reset FIFO address and payload length
  WriteRegister( REG_FIFO_ADDR_PTR, 0 );
  WriteRegister( REG_PAYLOAD_LENGTH, 0 );

#if 0
  ResetPin( PortCS, PinCS );
  Spi.Write( REG_FIFO );
  Spi.Write( Buffer, Length );
  SetPin( PortCS, PinCS );
#else
  uint8_t const* const Ptr = static_cast<uint8_t const*>( Buffer );
  for( auto Index = 0; Index < Length; Index++ )
  {
    WriteRegister( REG_FIFO, Ptr[ Index ] );
  }
#endif

  WriteRegister(REG_PAYLOAD_LENGTH, Length );

  WriteRegister( REG_DIO_MAPPING_1, 0x40 );
  WriteRegister( REG_IRQ_FLAGS_MASK, ~IRQ_TX_DONE_MASK );
  WriteRegister( REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX );
}

uint32_t TSx1276::ReadPacket( void *const Buffer, uint32_t const MaxLength )
{
  auto const Length = ReadRegister( REG_RX_NB_BYTES );
  if( Length > MaxLength )
  {
    return 0;
  }

  WriteRegister( REG_FIFO_ADDR_PTR, ReadRegister( REG_FIFO_RX_CURRENT_ADDR ));

#if 0
  ResetPin( PortCS, PinCS );
  Spi.Write( REG_FIFO );
  Spi.Read( Buffer, Length );
  SetPin( PortCS, PinCS );
#else
  uint8_t *const Ptr = static_cast<uint8_t*>( Buffer );
  for( auto Index = 0; Index < Length; Index++ )
  {
    Ptr[ Index ] = ReadRegister( REG_FIFO );
  }
#endif

  WriteRegister( REG_FIFO_ADDR_PTR, 0 );
  return Length;
}

int32_t TSx1276::GetRssi()
{
  int32_t const Snr = static_cast<int8_t>( ReadRegister( REG_PKT_SNR_VALUE ));
  uint32_t const Rssi = static_cast<uint8_t>( ReadRegister( REG_PKT_RSSI_VALUE ));

  if( Snr < 0 )
  {
    return Frequency > 868e6 ? -157 + Rssi + Snr / 4 : -164 + Rssi + Snr / 4;
  }
  else
  {
    return Frequency > 868e6 ? -157 + (Rssi * 16) / 15 : -164 + (Rssi * 16) / 15;
  }
}

int32_t TSx1276::GetSnr()
{
  int32_t const Snr = static_cast<int8_t>( ReadRegister( REG_PKT_SNR_VALUE ));
  return ( Snr * 10 ) / 4;
}

void TSx1276::Idle( void )
{
  WriteRegister( REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY );
}

void TSx1276::Sleep( void )
{
  WriteRegister( REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP );
}

void TSx1276::SetTxPower( int32_t const Level )
{
  auto const OldMode = ReadRegister( REG_OP_MODE );
  Sleep();

  int32_t Level0 = Level;
  if( Level0 < 2 )
  {
    Level0 = 2;
  }
  else if( Level0 > 17 )
  {
    Level0 = 17;
  }

  Level0 -= 2;
  WriteRegister( REG_PA_CONFIG, PA_BOOST | 0x70 | Level0 );

  WriteRegister( REG_OP_MODE, OldMode );
}

void TSx1276::SetChannel( uint32_t const Channel )
{
  SetFrequency(	BaseFreq + 25000U * Channel );
}

void TSx1276::SetFrequency( uint32_t const Frequency )
{
  uint8_t const OldMode = ReadRegister( REG_OP_MODE );
  Sleep();

  this->Frequency = Frequency;

  uint64_t const frf = (uint64_t(Frequency + FrequencyOffset) << 19 ) / 32000000;

  WriteRegister( REG_FRF_MSB, (uint8_t)( frf >> 16 ));
  WriteRegister( REG_FRF_MID, (uint8_t)( frf >> 8 ));
  WriteRegister( REG_FRF_LSB, (uint8_t)( frf >> 0 ));

  WriteRegister( REG_OP_MODE, OldMode );
}

void TSx1276::SetSpreadingFactor( uint32_t SpreadingFactor )
{
  uint8_t const OldMode = ReadRegister( REG_OP_MODE );
  Sleep();

  if( SpreadingFactor < 6 )
  {
    SpreadingFactor = 6;
  }
  else if( SpreadingFactor > 12 )
  {
    SpreadingFactor = 12;
  }

  if( SpreadingFactor == 6 )
  {
    WriteRegister( REG_DETECTION_OPTIMIZE, 0xc5 );
    WriteRegister( REG_DETECTION_THRESHOLD, 0x0c );
  }
  else
  {
    WriteRegister( REG_DETECTION_OPTIMIZE, 0xc3 );
    WriteRegister( REG_DETECTION_THRESHOLD, 0x0a );
  }

  WriteRegister( REG_MODEM_CONFIG_2, ( ReadRegister( REG_MODEM_CONFIG_2 ) & 0x0f ) | (( SpreadingFactor << 4 ) & 0xf0 ));

  WriteRegister(REG_OP_MODE, OldMode );
}

void TSx1276::SetSignalBandwidth( uint32_t const SignalBandwidth )
{
  uint8_t const OldMode = ReadRegister(REG_OP_MODE );
  Sleep();

  WriteRegister( REG_MODEM_CONFIG_1, ( ReadRegister( REG_MODEM_CONFIG_1) & 0x0f ) | ( SignalBandwidth << 4 ));

  // Receiver Spurious Reception of a LoRa Signal
  WriteRegister( 0x31, ReadRegister( 0x31 ) & ~0x80 );
  switch( SignalBandwidth )
  {
    case 0:
      WriteRegister( 0x2f, 0x48 );
      WriteRegister( 0x30, 0x00 );
      FrequencyOffset = 7810;
      break;
    case 1:
      WriteRegister( 0x2f, 0x44 );
      WriteRegister( 0x30, 0x00 );
      FrequencyOffset = 10420;
      break;
    case 2:
      WriteRegister( 0x2f, 0x44 );
      WriteRegister( 0x30, 0x00 );
      FrequencyOffset = 15620;
      break;
    case 3:
      WriteRegister( 0x2f, 0x44 );
      WriteRegister( 0x30, 0x00 );
      FrequencyOffset = 20830;
      break;
    case 4:
      WriteRegister( 0x2f, 0x44 );
      WriteRegister( 0x30, 0x00 );
      FrequencyOffset = 31250;
      break;
    case 5:
      WriteRegister( 0x2f, 0x44 );
      WriteRegister( 0x30, 0x00 );
      FrequencyOffset = 41670;
      break;
    case 6:
    case 7:
    case 8:
      WriteRegister( 0x2f, 0x40 );
      WriteRegister( 0x30, 0x00 );
      FrequencyOffset = 0;
      break;
    case 9:
//      WriteRegister( 0x2f, 0x );
//      WriteRegister( 0x30, 0x );
      FrequencyOffset = 0;
      break;
  }

  // Sensitivity Optimization with a 500 kHz Bandwidth
  if( SignalBandwidth == 9 )
  {
    if( BaseFreq >= 410e6 && BaseFreq <= 525e6 )
    {
      WriteRegister( 0x36, 0x02 );
      WriteRegister( 0x3a, 0x7F );
    }
    else if( BaseFreq >= 862e6 && BaseFreq <= 1020e6 )
    {
      WriteRegister( 0x36, 0x02 );
      WriteRegister( 0x3a, 0x64 );
    }
    else
    {
      WriteRegister( 0x36, 0x03 );
      WriteRegister( 0x3a, 0x65 );
    }
  }

  WriteRegister(REG_OP_MODE, OldMode );
}

void TSx1276::SetCodingRate( uint32_t Denominator )
{
  uint8_t const OldMode = ReadRegister( REG_OP_MODE );
  Sleep();

  if( Denominator < 5 )
  {
    Denominator = 5;
  }
  else if( Denominator > 8 )
  {
    Denominator = 8;
  }

  uint32_t const cr = Denominator - 4;
  WriteRegister( REG_MODEM_CONFIG_1, ( ReadRegister( REG_MODEM_CONFIG_1 ) & 0xf1) | ( cr << 1 ));

  WriteRegister( REG_OP_MODE, OldMode );
}

void TSx1276::SetPreambleLength( uint32_t const Length )
{
  WriteRegister( REG_PREAMBLE_MSB, Length >> 8 );
  WriteRegister( REG_PREAMBLE_LSB, Length );
}

void TSx1276::SetSyncWord( uint32_t const SyncWord )
{
  WriteRegister( REG_SYNC_WORD, SyncWord );
}

void TSx1276::SetCRC( bool const Mode )
{
  uint8_t const Reg = ReadRegister( REG_MODEM_CONFIG_2 );
  WriteRegister( REG_MODEM_CONFIG_2, Mode ? Reg | 0x04 : Reg & ~0x04 );
}

uint8_t TSx1276::ReadRegister( uint8_t const Address )
{
  TGpioScopeLow< TGpioRADIO_CS > RADIO_CS;

  TSpi::Transmit( Address & 0x7f );
  return TSpi::Receive();
}

void TSx1276::WriteRegister( uint8_t const Address, uint8_t const Value )
{
  TGpioScopeLow< TGpioRADIO_CS > RADIO_CS;

  TSpi::Transmit( Address | 0x80 );
  TSpi::Transmit( Value );
}
