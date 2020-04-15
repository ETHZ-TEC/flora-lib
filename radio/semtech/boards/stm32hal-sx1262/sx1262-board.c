/*!
 * \file      sx1262dvk1cas-board.c
 *
 * \brief     Target board SX1262DVK1CAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Markus Wegmann ( Technokrat )
 *
 */

#include <stdlib.h>

#include "main.h"

#include "arch/stm32hal/platform.h"
#include "radio/semtech/boards/board.h"
#include "radio/semtech/boards/sx126x-board.h"
#include "radio/semtech/boards/utilities.h"
#include "radio/radio_platform.h"
#include "time/rtc.h"
#include "radio/semtech/radio.h"
#include "stm32l4xx_hal.h"


void (*RadioOnDioIrqPtr)() = NULL;


static uint8_t tmp;
static uint8_t zero = 0;

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
#ifdef USE_TCXO
    return 5;
#else
    return 0;
#endif
}

void SX126xReset( void )
{
    rtc_delay(10);
    HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_RESET);
    rtc_delay(20);
    HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_SET); // internal pull-up
    rtc_delay(10);
    RADIO_TX_STOP_IND();
}

void SX126xWaitOnBusy( void )
{
    while( HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin) == GPIO_PIN_SET );
}

void SX126xWakeup( void )
{
    RADIO_TX_STOP_IND();
    RADIO_RX_STOP_IND();

    CRITICAL_SECTION_BEGIN( );

    tmp = RADIO_GET_STATUS;

    do
    {
    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, &zero, 1, 100);
    NSS_HIGH();
    }
    while (HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin) == GPIO_PIN_SET);

    CRITICAL_SECTION_END( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    NSS_HIGH();

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

void SX126xWriteCommandWithoutExecute( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    // no NSS_HIGH(); as it will be timed precisely
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, 100);
    for( uint16_t i = 0; i < size; i++ )
    {
      HAL_SPI_TransmitReceive(&RADIO_SPI, (uint8_t*) (buffer + i), &tmp, 1, 100);
        buffer[i] = tmp;
    }
    NSS_HIGH();

    SX126xWaitOnBusy( );
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    tmp = RADIO_WRITE_REGISTER;

    SX126xCheckDeviceReady( );

    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address) + 1, 1, 100); // MSB first!
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address), 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    NSS_HIGH();

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    tmp = RADIO_READ_REGISTER;

    SX126xCheckDeviceReady( );

    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address) + 1, 1, 100); // MSB first!
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address), 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, 100);
    HAL_SPI_Receive(&RADIO_SPI, buffer, size, 1000);
    NSS_HIGH();

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
  tmp = RADIO_WRITE_BUFFER;

    SX126xCheckDeviceReady( );

    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &offset, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    NSS_HIGH();

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
  tmp = RADIO_READ_BUFFER;

    SX126xCheckDeviceReady( );

    NSS_LOW();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &offset, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, 100);
    HAL_SPI_Receive(&RADIO_SPI, buffer, 255, 1000);
    NSS_HIGH();

    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetPaSelect( uint32_t channel )
{
#ifndef DEVKIT
  return SX1262;
#else
    if( HAL_GPIO_ReadPin(DEVKIT_DEVICE_SEL_GPIO_Port, DEVKIT_DEVICE_SEL_Pin) == 1 )
    {
        return SX1261;
    }
    else
    {
        return SX1262;
    }
#endif
}

void SX126xAntSwOn( void )
{
#ifndef DEVKIT
  HAL_GPIO_WritePin(RADIO_ANT_SW_GPIO_Port, RADIO_ANT_SW_Pin, GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(RADIO_ANT_SW_GPIO_Port, RADIO_ANT_SW_Pin, GPIO_PIN_SET);
#endif
}

void SX126xAntSwOff( void )
{
#ifndef DEVKIT
  // DEBUG BEGIN: ANT_SWITCH
  HAL_GPIO_WritePin(RADIO_ANT_SW_GPIO_Port, RADIO_ANT_SW_Pin, GPIO_PIN_SET);
  // DEBUG END: ANT_SWITCH
#else
  HAL_GPIO_WritePin(RADIO_ANT_SW_GPIO_Port, RADIO_ANT_SW_Pin, GPIO_PIN_RESET);
#endif
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
#ifndef DEVKIT
  if (frequency > 863E6 || frequency < 870E6)
    return true;
  else
    return false;
#else
    return true;
#endif
}
