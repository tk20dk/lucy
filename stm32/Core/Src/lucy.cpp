#include "lucy.h"
#include "adc.h"
#include "rtc.h"
#include "ds18b20.h"


TLucy Lucy;
TSystem System;

void TLucy::Init()
{
  TSystem::Init();
  TAdc::Init();
  TRtc::Init( 60000 );
  TDs18b20::Init();
  Radio.Init();

//  TSystem::DisableSWD();
}

void TLucy::Loop()
{
  if( FlagRadio )
  {
    FlagRadio = false;
    Radio.Interrupt();
  }

  if( FlagWakeUp )
  {
    FlagWakeUp = false;
    WakeUpEvent();
  }

  TSystem::DeepSleep();
}

void TLucy::WakeUpEvent()
{
  TSystem::Green( 100 );

  TAdc::Kick();
  auto const Vbat = TAdc::GetVbat();
  auto const Vref = TAdc::GetVref();
  TSystem::Printf( "Vbat: %u.%03u\n", Vbat / 1000, Vbat % 1000 );
  TSystem::Printf( "Vref: %u.%03u\n", Vref / 1000, Vref % 1000 );

  union
  {
    int16_t Temp[ 8 ];
    uint8_t Buffer[ 1 ];
  } Data;

  for( auto Index = 0; Index < 8; Index++ )
  {
    auto const Temperature = TDs18b20::GetTemperature( Index );
    TSystem::Printf( "%u: %d.%d\n", Index, Temperature / 10, Temperature % 10 );
    Data.Temp[ Index ] = Temperature;
  }
  Radio.Transmit( Data.Buffer, sizeof( Data ));
}

void TLucy::RadioEvent( TRadioEvent const Event )
{
  if( Event == TRadioEvent::TxDone )
  {
    Radio.Sleep();
    TSystem::Printf( "TxDone\n" );
  }
  else if( Event == TRadioEvent::RxDone )
  {
    uint8_t Buffer[ 128 ];
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));
    if( Length > 0 )
    {
    }
    Radio.Sleep();
    TSystem::Printf( "RxDone\n" );
  }
  else if( Event == TRadioEvent::Timeout )
  {
    TSystem::Printf( "Timeout\n" );
  }
  else if( Event == TRadioEvent::NoCrc )
  {
    TSystem::Printf( "NoCrc\n" );
  }
  else if( Event == TRadioEvent::CrcError )
  {
    TSystem::Printf( "CrcError\n" );
  }
}

void TLucy::RTC_IRQHandler()
{
  if( LL_RTC_IsActiveFlag_WUT( RTC ) == 1 )
  {
    LL_RTC_ClearFlag_WUT( RTC );
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_20 );
    FlagWakeUp = true;
  }
  else
  {
    NVIC_DisableIRQ( RTC_IRQn );
  }
}

void TLucy::EXTI4_15_IRQHandler()
{
  if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_8 ) != RESET )
  {
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_8 );
    FlagRadio = true;
  }

  if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_9 ) != RESET )
  {
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_9 );
    FlagRadio = true;
  }

  if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_10 ) != RESET )
  {
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_10 );
    FlagRadio = true;
  }
}

void TLucy::ADC1_COMP_IRQHandler()
{
  if( LL_ADC_IsActiveFlag_EOS( ADC1 ) != 0 )
  {
    LL_ADC_ClearFlag_EOS( ADC1 );
    TAdc::AdcGrpRegularSequenceConvComplete_Callback();
  }

  if( LL_ADC_IsActiveFlag_OVR( ADC1 ) != 0 )
  {
    LL_ADC_ClearFlag_OVR( ADC1 );
    TAdc::AdcGrpRegularOverrunError_Callback();
  }
}

void TLucy::DMA1_Channel1_IRQHandler()
{
  if( LL_DMA_IsActiveFlag_TC1( DMA1 ) == 1 )
  {
    LL_DMA_ClearFlag_GI1( DMA1 );
    TAdc::AdcDmaTransferComplete_Callback();
  }

  if( LL_DMA_IsActiveFlag_TE1( DMA1 ) == 1 )
  {
    LL_DMA_ClearFlag_TE1( DMA1 );
    TAdc::AdcDmaTransferError_Callback();
  }
}

TLucy::TLucy() :
  FlagADC( false ),
  FlagRadio( false ),
  FlagWakeUp( true ),
  Radio( 433050000, TCallback( this, &TLucy::RadioEvent ))
{
}

extern "C" void LucyInit()
{
  Lucy.Init();
}

extern "C" void LucyLoop()
{
  Lucy.Loop();
}

extern "C" void RTC_IRQHandler()
{
  Lucy.RTC_IRQHandler();
}

extern "C" void EXTI4_15_IRQHandler()
{
  Lucy.EXTI4_15_IRQHandler();
}

extern "C" void ADC1_COMP_IRQHandler()
{
  Lucy.ADC1_COMP_IRQHandler();
}

extern "C" void DMA1_Channel1_IRQHandler()
{
  Lucy.DMA1_Channel1_IRQHandler();
}
