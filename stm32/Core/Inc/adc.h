#include "system.h"


auto const PWR_SENSOR_Port = reinterpret_cast< intptr_t >( PWR_SENSOR_GPIO_Port );
typedef TGpio< PWR_SENSOR_Port, PWR_SENSOR_Pin > TGpioPWR_SENSOR;

#define ADC_CONVERTED_DATA_BUFFER_SIZE 3
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES ( LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32 )

static uint16_t Vbat;
static uint16_t Vref;
static uint16_t AdcData[ ADC_CONVERTED_DATA_BUFFER_SIZE ];
static volatile uint8_t ubDmaTransferStatus = 2;
static volatile uint8_t ubAdcGrpRegularSequenceConvStatus;

struct TAdc
{
  static void Init()
  {
    Configure_DMA();
    Configure_ADC();
    Activate_ADC();
  }

  static void Kick()
  {
    TGpioPWR_SENSOR::SetPinMode( LL_GPIO_MODE_OUTPUT );

    UserButton_Callback();
    while( ubDmaTransferStatus != 1 )
    {
    }

    Vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE( AdcData[ 1 ], LL_ADC_RESOLUTION_12B );
    Vbat = __LL_ADC_CALC_DATA_TO_VOLTAGE( Vref, AdcData[ 0 ], LL_ADC_RESOLUTION_12B );

    TGpioPWR_SENSOR::SetPinMode( LL_GPIO_MODE_ANALOG );
  }

  static uint32_t GetVbat()
  {
    return ( Vbat * 3 ) / 2;
  }

  static uint32_t GetVref()
  {
    return Vref;
  }

  static void Configure_DMA()
  {
    NVIC_SetPriority( DMA1_Channel1_IRQn, 1 );
    NVIC_EnableIRQ( DMA1_Channel1_IRQn );

    LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_DMA1 );

    /*  - DMA transfer in circular mode to match with ADC configuration:        */
    /*    DMA unlimited requests.                                               */
    /*  - DMA transfer from ADC without address increment.                      */
    /*  - DMA transfer to memory with address increment.                        */
    /*  - DMA transfer from ADC by half-word to match with ADC configuration:   */
    /*    ADC resolution 12 bits.                                               */
    /*  - DMA transfer to memory by half-word to match with ADC conversion data */
    /*    buffer variable type: half-word.                                      */
    LL_DMA_ConfigTransfer( DMA1,
                           LL_DMA_CHANNEL_1,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                           LL_DMA_MODE_CIRCULAR              |
                           LL_DMA_PERIPH_NOINCREMENT         |
                           LL_DMA_MEMORY_INCREMENT           |
                           LL_DMA_PDATAALIGN_HALFWORD        |
                           LL_DMA_MDATAALIGN_HALFWORD        |
                           LL_DMA_PRIORITY_HIGH              );

    LL_DMA_SetPeriphRequest( DMA1,
                             LL_DMA_CHANNEL_1,
                             LL_DMA_REQUEST_0 );

    LL_DMA_ConfigAddresses( DMA1,
                            LL_DMA_CHANNEL_1,
                            LL_ADC_DMA_GetRegAddr( ADC1, LL_ADC_DMA_REG_REGULAR_DATA ),
                            (uint32_t)&AdcData,
                            LL_DMA_DIRECTION_PERIPH_TO_MEMORY );

    LL_DMA_SetDataLength( DMA1,
                          LL_DMA_CHANNEL_1,
                          ADC_CONVERTED_DATA_BUFFER_SIZE );

    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_1 );
    LL_DMA_EnableIT_TE( DMA1, LL_DMA_CHANNEL_1 );
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_1 );
  }

  static void Configure_ADC()
  {
    NVIC_SetPriority( ADC1_COMP_IRQn, 0 );
    NVIC_EnableIRQ( ADC1_COMP_IRQn );

    LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_ADC1 );

    if( __LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0 )
    {
      LL_ADC_SetCommonPathInternalCh( __LL_ADC_COMMON_INSTANCE( ADC1 ), ( LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR ));

      uint32_t wait_loop_index = (( LL_ADC_DELAY_TEMPSENSOR_STAB_US * ( SystemCoreClock / ( 100000 * 2 ))) / 10 );
      while( wait_loop_index != 0 )
      {
        wait_loop_index--;
      }
    }

    if( LL_ADC_IsEnabled( ADC1 ) == 0 )
    {
      LL_ADC_SetClock( ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2 );
      LL_ADC_SetSamplingTimeCommonChannels( ADC1, LL_ADC_SAMPLINGTIME_160CYCLES_5 );
    }

    if(( LL_ADC_IsEnabled( ADC1 ) == 0 ) || ( LL_ADC_REG_IsConversionOngoing( ADC1 ) == 0 ))
    {
      LL_ADC_REG_SetTriggerSource( ADC1, LL_ADC_REG_TRIG_SOFTWARE );
      LL_ADC_REG_SetContinuousMode( ADC1, LL_ADC_REG_CONV_SINGLE );
      LL_ADC_REG_SetDMATransfer( ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED );
      LL_ADC_REG_SetOverrun( ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN );
      LL_ADC_REG_SetSequencerChannels( ADC1, LL_ADC_CHANNEL_8 | LL_ADC_CHANNEL_VREFINT | LL_ADC_CHANNEL_TEMPSENSOR );
    }

    if(( LL_ADC_IsEnabled( ADC1 ) == 0 ) || ( LL_ADC_REG_IsConversionOngoing( ADC1 ) == 0 ))
    {
    }

    LL_ADC_EnableIT_EOS( ADC1 );
    LL_ADC_EnableIT_OVR( ADC1 );
  }

  static void Activate_ADC()
  {
    if( LL_ADC_IsEnabled( ADC1 ) == 0 )
    {
      uint32_t const backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer( ADC1 );
      LL_ADC_REG_SetDMATransfer( ADC1, LL_ADC_REG_DMA_TRANSFER_NONE );

      LL_ADC_StartCalibration( ADC1 );

      while( LL_ADC_IsCalibrationOnGoing( ADC1 ) != 0 )
      {
      }

      uint32_t wait_loop_index = ( ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1 );
      while( wait_loop_index != 0 )
      {
        wait_loop_index--;
      }

      LL_ADC_REG_SetDMATransfer( ADC1, backup_setting_adc_dma_transfer );
      LL_ADC_Enable( ADC1 );

      while( LL_ADC_IsActiveFlag_ADRDY( ADC1 ) == 0 )
      {
      }
    }
  }

  static void UserButton_Callback()
  {
    if( ubDmaTransferStatus != 0 )
    {
      ubDmaTransferStatus = 0;
    }
    else
    {
       TSystem::Red( 1000000 );
    }

    if(( LL_ADC_IsEnabled( ADC1 ) == 1 ) && ( LL_ADC_IsDisableOngoing( ADC1 ) == 0 ) && ( LL_ADC_REG_IsConversionOngoing( ADC1 ) == 0 ))
    {
      LL_ADC_REG_StartConversion( ADC1 );
    }
    else
    {
       TSystem::Red( 1000000 );
    }
  }

  static void AdcDmaTransferComplete_Callback()
  {
    ubDmaTransferStatus = 1;

    if( ubAdcGrpRegularSequenceConvStatus != 1 )
    {
      AdcDmaTransferError_Callback();
    }

    ubAdcGrpRegularSequenceConvStatus = 0;
  }

  static void AdcDmaTransferError_Callback()
  {
    TSystem::Red( 1000000 );
  }

  static void AdcGrpRegularSequenceConvComplete_Callback()
  {
    ubAdcGrpRegularSequenceConvStatus = 1;
  }

  static void AdcGrpRegularOverrunError_Callback()
  {
    LL_ADC_DisableIT_OVR( ADC1 );
    TSystem::Red( 1000000 );
  }
};
