#ifndef SPI_H__
#define SPI_H__


template< intptr_t const SPIx >
struct TDriverSpi
{
  static void Init()
  {
    auto const Spi = reinterpret_cast< SPI_TypeDef* >( SPIx );

    LL_SPI_Enable( Spi );
  }

  static uint8_t Receive()
  {
    return TransmitReceive( 0x00 );
  }

  template< typename T >
  static void Transmit( T const Data )
  {
    (void)TransmitReceive( Data );
  }

  template< typename T >
  static uint8_t TransmitReceive( T const Data )
  {
    auto const Spi = reinterpret_cast< SPI_TypeDef* >( SPIx );

    while( !LL_SPI_IsActiveFlag_TXE( Spi ))
    {
    }
    LL_SPI_TransmitData8( Spi, static_cast< uint8_t >( Data ));

    while( !LL_SPI_IsActiveFlag_RXNE( Spi ))
    {
    }
    return LL_SPI_ReceiveData8( Spi );
  }

  static void WaitIdle()
  {
    auto const Spi = reinterpret_cast< SPI_TypeDef* >( SPIx );

    while( LL_SPI_IsActiveFlag_BSY( Spi ))
    {
    }
  }

  static void SetTransferBitOrderLSB()
  {
    auto const Spi = reinterpret_cast< SPI_TypeDef* >( SPIx );
    LL_SPI_SetTransferBitOrder( Spi, LL_SPI_LSB_FIRST );
  }

  static void SetTransferBitOrderMSB()
  {
    auto const Spi = reinterpret_cast< SPI_TypeDef* >( SPIx );
    LL_SPI_SetTransferBitOrder( Spi, LL_SPI_MSB_FIRST );
  }
};

#endif // SPI_H__
