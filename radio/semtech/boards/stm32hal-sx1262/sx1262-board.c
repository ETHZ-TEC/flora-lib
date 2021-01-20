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

#include "flora_lib.h"


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
    delay_us(100);
    RADIO_CLR_NRESET_PIN();
    delay_us(200);
    RADIO_SET_NRESET_PIN();
    delay_us(100);
}

void SX126xWaitOnBusy( void )
{
    if (RADIO_READ_NSS_PIN() == 0)
    {
        RADIO_SET_NSS_PIN();    // make sure the pin is high
        delay_us(1);            // 600ns max required between two NSS edges
    }
    while( RADIO_READ_BUSY_PIN( ) );
}

void SX126xWakeup( void )
{
    ENTER_CRITICAL_SECTION( );

    tmp = RADIO_GET_STATUS;

    do
    {
        RADIO_CLR_NSS_PIN();
        HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
        HAL_SPI_Transmit(&RADIO_SPI, &zero, 1, 100);
        RADIO_SET_NSS_PIN();
    }
    while ( RADIO_READ_BUSY_PIN ( ) );

    // make sure the antenna switch is turned on
    SX126xAntSwOn( );

    LEAVE_CRITICAL_SECTION( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    RADIO_SET_NSS_PIN();

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

void SX126xWriteCommandWithoutExecute( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    // no RADIO_SET_NSS_PIN(); as it will be timed precisely
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, 100);
    for( uint16_t i = 0; i < size; i++ )
    {
        HAL_SPI_TransmitReceive(&RADIO_SPI, (uint8_t*) (buffer + i), &tmp, 1, 100);
        buffer[i] = tmp;
    }
    RADIO_SET_NSS_PIN();

    SX126xWaitOnBusy( );
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    tmp = RADIO_WRITE_REGISTER;

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address) + 1, 1, 100); // MSB first!
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address), 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    RADIO_SET_NSS_PIN();

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

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address) + 1, 1, 100); // MSB first!
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address), 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, 100);
    HAL_SPI_Receive(&RADIO_SPI, buffer, size, 1000);
    RADIO_SET_NSS_PIN();

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

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &offset, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    RADIO_SET_NSS_PIN();

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    tmp = RADIO_READ_BUFFER;

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &tmp, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &offset, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, 100);
    HAL_SPI_Receive(&RADIO_SPI, buffer, 255, 1000);
    RADIO_SET_NSS_PIN();

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

