#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <cstdio>
#include <cstdint>
#include <cstdarg>
#include "main.h"
#include "gpio.h"


auto const HMI_RED_Port = reinterpret_cast< intptr_t >( HMI_RED_GPIO_Port );
typedef TGpio< HMI_RED_Port, HMI_RED_Pin > TGpioHMI_RED;
auto const HMI_GREEN_Port = reinterpret_cast< intptr_t >( HMI_GREEN_GPIO_Port );
typedef TGpio< HMI_GREEN_Port, HMI_GREEN_Pin > TGpioHMI_GREEN;


inline void PinModeInput( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  auto const Index = (31 - __builtin_clz( Pin )) * 2;
  Port->MODER = ( Port->MODER & ~( 3 << Index )) | ( 0 << Index );
}
inline void PinModeOutput( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  auto const Index = (31 - __builtin_clz( Pin )) * 2;
  Port->MODER = ( Port->MODER & ~( 3 << Index )) | ( 1 << Index );
}
inline void PinModeAlternate( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  auto const Index = (31 - __builtin_clz( Pin )) * 2;
  Port->MODER = ( Port->MODER & ~( 3 << Index )) | ( 2 << Index );
}
inline void PinModeAnalog( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  auto const Index = (31 - __builtin_clz( Pin )) * 2;
  Port->MODER = ( Port->MODER & ~( 3 << Index )) | ( 3 << Index );
}
inline void PinModePushPull( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  Port->OTYPER &= ~Pin;
}
inline void PinModeOpenDrain( GPIO_TypeDef *const Port, uint32_t const Pin )
{
  Port->OTYPER |= Pin;
}

uint8_t const FLAG_CONFIG = 0x12;

class TSystem
{
public:
  struct TConfig
  {
    TConfig() :
      Flag( FLAG_CONFIG ),
      Mode( 0 ),
      UnitId( 0 ),
      SensorDelay( 60 ),
      TxPower( 2 ),
      Channel( 38 ),
      CodingRate( 5 ),      // 5 to 8  => 4/5 to 4/8
      SpreadingFactor( 7 ), // 6 to 12 => 85 to 4096 chips/symbols
      SignalBandwidth( 9 )  // 0 to 9  => 7,8kHz to 500kHz
    {
    }

    uint8_t Flag;
    uint8_t Mode;
    uint8_t UnitId;
    uint8_t SensorDelay;
    int8_t  TxPower;
    uint8_t Channel;
    uint8_t CodingRate;
    uint8_t SpreadingFactor;
    uint8_t SignalBandwidth;
  };

  void UpdateConfig( void );

  static void Init()
  {
    LL_PWR_SetRegulModeLP( LL_PWR_REGU_LPMODES_LOW_POWER );
    LL_LPM_EnableDeepSleep();
    LL_PWR_EnableFastWakeUp();
    LL_PWR_EnableUltraLowPower();

    LL_LPUART_Enable( LPUART1 );
    LL_LPUART_ReceiveData8( LPUART1 );
    LL_LPUART_ClearFlag_FE( LPUART1 );
    LL_LPUART_ClearFlag_NE( LPUART1 );
    LL_LPUART_ClearFlag_ORE( LPUART1 );
  }

  static void Red( uint32_t const Interval )
  {
    TGpioHMI_RED::Set();
    TSystem::MsDelay( 100 );
    TGpioHMI_RED::Reset();
  }

  static void Green( uint32_t const Interval )
  {
    TGpioHMI_GREEN::Set();
    TSystem::MsDelay( 100 );
    TGpioHMI_GREEN::Reset();
  }

  static void MsDelay( uint32_t const Interval )
  {
    LL_mDelay( Interval );
  }

  static void DeepSleep()
  {
    LL_PWR_ClearFlag_SB();
    LL_PWR_ClearFlag_WU();
    __WFI();
  }

  static void Printf( char const *const Format, ... )
  {
    static char Buffer[ 128 ];

    va_list Args;
    va_start( Args, Format );
    int const Result = vsnprintf( Buffer, sizeof( Buffer ), Format, Args );
    va_end( Args );

    for( auto Index = 0; Index < Result; Index++ )
    {
      while( !LL_LPUART_IsActiveFlag_TXE( LPUART1 ))
      {
      }

      LL_LPUART_TransmitData8( LPUART1, Buffer[ Index ] );

      while( !LL_LPUART_IsActiveFlag_TC( LPUART1 ))
      {
      }
    }
  }

  static void DisableSWD()
  {
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    LL_GPIO_Init( GPIOB, &GPIO_InitStruct );
  }

  static void StopDelay( uint32_t const Interval )
  {
    MsDelay( Interval );
  }

  public:
  TConfig Config;
};

extern TSystem System;

#endif // SYSTEM_H_
