#ifndef RTC_H__
#define RTC_H__


struct TRtc
{
  static void Init( uint32_t const Interval )
  {
    SetAlarmInterval( Interval );
    LL_EXTI_EnableIT_0_31( LL_EXTI_LINE_20 );
    LL_EXTI_EnableRisingTrig_0_31( LL_EXTI_LINE_20 );
  }

  static void SetAlarmInterval( uint32_t const Interval )
  {
    LL_RTC_DisableWriteProtection( RTC );
    LL_RTC_WAKEUP_Disable( RTC );
    LL_RTC_DisableIT_WUT( RTC );

    LL_RTC_WAKEUP_SetAutoReload( RTC, Interval );
    LL_RTC_WAKEUP_SetClock( RTC, LL_RTC_WAKEUPCLOCK_CKSPRE );

    LL_RTC_WAKEUP_Enable( RTC );
    LL_RTC_EnableIT_WUT( RTC );
    LL_RTC_EnableWriteProtection( RTC );
  }
};

#endif // RTC_H__
