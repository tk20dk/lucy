#ifndef GPIO_H__
#define GPIO_H__

#include <cstdint>


template < intptr_t const GPIOx, uint32_t const PinMask >
struct TGpio
{
  static void SetPinMode( uint32_t const Mode )
  {
    auto const Gpio = reinterpret_cast< GPIO_TypeDef* >( GPIOx );
    LL_GPIO_SetPinMode( Gpio, PinMask, Mode );
  }

  static bool Input()
  {
    auto const Gpio = reinterpret_cast< GPIO_TypeDef* >( GPIOx );
    return LL_GPIO_IsInputPinSet( Gpio, PinMask ) != 0;
  }

  static void Set()
  {
    auto const Gpio = reinterpret_cast< GPIO_TypeDef* >( GPIOx );
    LL_GPIO_SetOutputPin( Gpio, PinMask );
  }

  static void Reset()
  {
    auto const Gpio = reinterpret_cast< GPIO_TypeDef* >( GPIOx );
    LL_GPIO_ResetOutputPin( Gpio , PinMask );
  }

  static void High()
  {
    Set();
  }

  static void Low()
  {
    Reset();
  }
};

template < typename TGpio >
struct TGpioScopeLow
{
  TGpioScopeLow()
  {
    TGpio::Reset();
  }

  ~TGpioScopeLow()
  {
    TGpio::Set();
  }
};

template < typename TGpio >
struct TGpioScopeHigh
{
  TGpioScopeHigh()
  {
    TGpio::Set();
  }

  ~TGpioScopeHigh()
  {
    TGpio::Reset();
  }
};

#endif // GPIO_H__
