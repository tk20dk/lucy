#ifndef LPUART_H__
#define LPUART_H__


template< intptr_t const LPUARTx >
struct TDriverLpuart
{
  static void Init()
  {
    auto const Lpuart = reinterpret_cast< USART_TypeDef* >( LPUARTx );

    LL_LPUART_Enable( Lpuart );
    LL_LPUART_ReceiveData8( Lpuart );
    LL_LPUART_ClearFlag_FE( Lpuart );
    LL_LPUART_ClearFlag_NE( Lpuart );
    LL_LPUART_ClearFlag_ORE( Lpuart );
  }
};

#endif //LPUART_H__
