#ifndef USART_H__
#define USART_H__




template< intptr_t const USARTx >
struct TDriverUsart
{
  static void Init()
  {
    auto const Usart = reinterpret_cast< USART_TypeDef* >( USARTx );

    LL_USART_Enable( Usart );

    LL_USART_ReceiveData8( Usart );

    LL_USART_ClearFlag_FE( Usart );
    LL_USART_ClearFlag_NE( Usart );
    LL_USART_ClearFlag_ORE( Usart );
  }

  template< typename T >
  static uint8_t TransmitReceive( T const Data )
  {
    Transmit( Data );
    return Receive();
  }

  static char Receive()
  {
    auto const Usart = reinterpret_cast< USART_TypeDef* >( USARTx );

    if( LL_USART_IsActiveFlag_FE( Usart ))
    {
      LL_USART_ClearFlag_FE( Usart );
    }

    if( LL_USART_IsActiveFlag_NE( Usart ))
    {
      LL_USART_ClearFlag_NE( Usart );
    }

    if( LL_USART_IsActiveFlag_ORE( Usart ))
    {
      LL_USART_ClearFlag_ORE( Usart );
    }

    while( !LL_USART_IsActiveFlag_RXNE( Usart ))
    {
    }
    return LL_USART_ReceiveData8( Usart );
  }

  template< typename T, typename TT >
  static void Transmit( T const* const Data, TT const Length )
  {
    for( TT Index = 0; Index < Length; Index++ )
    {
      Transmit( Data[ Index ] );
    }
  }

  template< typename T >
  static void Transmit( T const Data )
  {
    auto const Usart = reinterpret_cast< USART_TypeDef* >( USARTx );
    auto const Tmp = static_cast< uint8_t >( Data );

    while ( !LL_LPUART_IsActiveFlag_TXE( Usart ))
    {
    }

    LL_LPUART_TransmitData8( Usart, Tmp );

    while( !LL_LPUART_IsActiveFlag_TC( Usart ))
    {
    }
  }

  static void SetBaudrate( uint32_t const Baudrate )
  {
    auto const Usart = reinterpret_cast< USART_TypeDef* >( USARTx );

    LL_USART_InitTypeDef USART_InitStruct = {0};

    USART_InitStruct.BaudRate = Baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Disable( Usart );
    LL_USART_Init( Usart, &USART_InitStruct);
    LL_USART_Enable( Usart );
  }
};


#endif // USART_H__
