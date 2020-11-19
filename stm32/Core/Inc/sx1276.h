#ifndef SX1276_H_
#define SX1276_H_

#include "fast-delegate.h"
#include "system.h"
#include "spi.h"


enum class TRadioEvent
{
  RxDone,
  TxDone,
  Timeout,
  NoCrc,
  CrcError
};

typedef fastdelegate::FastDelegate< void ( TRadioEvent const Event ) > TCallback;

class TSx1276
{
public:
  TSx1276( uint32_t const BaseFreq, TCallback const Callback );

  bool Init();
  void Receive();
  void Transmit( void const *const Buffer, uint16_t const Length );
  void Interrupt();
  int32_t GetSnr();
  int32_t GetRssi();
  uint32_t ReadPacket( void *const Buffer, uint32_t const MaxLength );

  void Sleep();
private:
  void Idle();
  void Reset();

  void SetCRC( bool const Mode );
  void SetTxPower( int32_t const Level );
  void SetChannel( uint32_t const Channel );
  void SetFrequency( uint32_t const Frequency );
  void SetCodingRate( uint32_t const Denominator );
  void SetSpreadingFactor( uint32_t const SpreadingFactor );
  void SetSignalBandwidth( uint32_t const SignalBandwidth );
  void SetSyncWord( uint32_t const SyncWord );
  void SetPreambleLength( uint32_t const Length );

  void WriteRegister( uint8_t const Address, uint8_t const Value );
  uint8_t ReadRegister( uint8_t const Address );

private:
  uint32_t Frequency;
  uint32_t FrequencyOffset;
  uint32_t const BaseFreq;
  TCallback const Callback;
};

#endif // SX1276_H_
