#ifndef LUCY_H__
#define LUCY_H__

#include "sx1276.h"


class TLucy
{
public:
  TLucy();

  void Init();
  void Loop();

  void RTC_IRQHandler();
  void EXTI4_15_IRQHandler();
  void ADC1_COMP_IRQHandler();
  void DMA1_Channel1_IRQHandler();

private:
  void RadioEvent( TRadioEvent const Event );
  void WakeUpEvent();

private:
  bool FlagADC;
  bool FlagRadio;
  bool FlagWakeUp;
  TSx1276 Radio;
};

#endif // LUCY_H__
