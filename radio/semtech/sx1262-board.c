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


#define SX126x_CMD_STATUS_VALID(status)     ((status & 0xe) < (0x3 << 1) || (status & 0xe) > (0x5 << 1))      // see datasheet p.95


static int8_t sx126x_lock = 0;     // radio SPI access semaphore


static void SX126xReleaseLock(void)
{
    sx126x_lock--;
    if (sx126x_lock < 0)
    {
        LOG_ERROR("invalid semaphore state");
        sx126x_lock = 0;
    }
}

static bool SX126xAcquireLock(void)
{
    if (sx126x_lock == 0)
    {
        sx126x_lock++;
        if (sx126x_lock == 1)
        {
            return true;
        }
        SX126xReleaseLock();
    }
    LOG_ERROR("failed to acquire radio lock");
    return false;
}

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

    RADIO_CLR_NSS_PIN();      // falling edge will trigger the radio wake-up
    delay_us(100);
    SX126xWaitOnBusy( );

    SX126xSetOperatingMode(MODE_STDBY_RC);

    // make sure the antenna switch is turned on
    SX126xAntSwOn( );

    LEAVE_CRITICAL_SECTION( );
}

static bool SX126xSPIWrite( RadioCommands_t command, uint8_t *buffer, uint8_t size )
{
#if SX126x_CHECK_CMD_RETVAL

    uint8_t cmd_status = 0;

    if (HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, SX126x_CMD_TIMEOUT)               != HAL_OK ||
        HAL_SPI_TransmitReceive(&RADIO_SPI, buffer, &cmd_status, 1, SX126x_CMD_TIMEOUT)        != HAL_OK ||
        !SX126x_CMD_STATUS_VALID(cmd_status)                                                             ||
        ((size > 1) && (HAL_SPI_Transmit(&RADIO_SPI, &buffer[1], size - 1, SX126x_CMD_TIMEOUT) != HAL_OK)))
    {
        LOG_WARNING("failed to send radio cmd (%x)", cmd_status);
        return false;
    }

#else /* SX126x_CHECK_CMD_RETVAL */

    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT);

#endif /* SX126x_CHECK_CMD_RETVAL */

    return true;
}

bool SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    bool success = true;

    if (!SX126xAcquireLock())
    {
        return false;
    }
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    success = SX126xSPIWrite( command, buffer, size );
    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }

    SX126xReleaseLock();

    return success;
}

bool SX126xWriteCommandWithoutExecute( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    bool success = true;

    if (!SX126xAcquireLock())
    {
        return false;
    }
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    success = SX126xSPIWrite( command, buffer, size );
    // no RADIO_SET_NSS_PIN(); as it will be timed precisely

    SX126xReleaseLock();

    return success;
}

bool SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    bool success = true;

    if (!SX126xAcquireLock())
    {
        return false;
    }
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();

#if SX126x_CHECK_CMD_RETVAL

    uint8_t cmd_status = 0;

    if (HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, SX126x_CMD_TIMEOUT)             != HAL_OK ||
        HAL_SPI_TransmitReceive(&RADIO_SPI, &cmd_status, &cmd_status, 1, SX126x_CMD_TIMEOUT) != HAL_OK ||
        !SX126x_CMD_STATUS_VALID(cmd_status)                                                           ||
        HAL_SPI_TransmitReceive(&RADIO_SPI, buffer, buffer, size, SX126x_CMD_TIMEOUT)        != HAL_OK)
    {
        LOG_WARNING("failed to execute read cmd");
        success = false;
    }

#else
    uint8_t zero = 0;

    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&RADIO_SPI, (uint8_t*) buffer, buffer, size, SX126x_CMD_TIMEOUT);

#endif /* SX126x_CHECK_CMD_RETVAL */

    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();

    return success;
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint8_t cmd = RADIO_WRITE_REGISTER;

    if (!SX126xAcquireLock())
    {
        return;
    }
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address) + 1, 1, SX126x_CMD_TIMEOUT); // MSB first!
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address), 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, 1000);
    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint8_t cmd  = RADIO_READ_REGISTER;
    uint8_t zero = 0;

    if (!SX126xAcquireLock())
    {
        return;
    }
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address) + 1, 1, SX126x_CMD_TIMEOUT); // MSB first!
    HAL_SPI_Transmit(&RADIO_SPI, ((uint8_t*) &address), 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Receive(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT);
    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint8_t cmd = RADIO_WRITE_BUFFER;

    if (!SX126xAcquireLock())
    {
        return;
    }
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &offset, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT);
    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint8_t cmd  = RADIO_READ_BUFFER;
    uint8_t zero = 0;

    if (!SX126xAcquireLock())
    {
        return;
    }
    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &offset, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &zero, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Receive(&RADIO_SPI, buffer, 255, SX126x_CMD_TIMEOUT);
    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();
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

