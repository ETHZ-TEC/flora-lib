/*
 * Copyright (c) 2018 - 2021, ETH Zurich, Computer Engineering Group (TEC)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * based on sx1262dvk1cas-board.c by Semtech
 */

#include "flora_lib.h"


#define SX126x_CMD_STATUS_VALID(status)     ((status & 0xe) < (0x4 << 1) || (status & 0xe) > (0x5 << 1))      // see datasheet p.95


#if SX126x_PRINT_ERRORS

#define SX126x_ERROR(...)         sx126x_error_cnt++; LOG_ERROR(__VA_ARGS__)

#else /* SX126x_PRINT_ERRORS */

#define SX126x_ERROR(...)         sx126x_error_cnt++

#endif /* SX126x_PRINT_ERRORS */


#if SX126x_USE_ACCESS_LOCK

#ifndef SX126xAcquireLock         // if not used-defined, use the default lock implementation

semaphore_t sx126x_lock = 1;      // initial value 1 means this is a binary semaphore

#define SX126xAcquireLock()       if (!semaphore_acquire(&sx126x_lock)) \
                                  { \
                                      SX126x_ERROR("radio access denied"); \
                                      return false; \
                                  }
#define SX126xReleaseLock()       semaphore_release(&sx126x_lock)

#endif /* SX126xAcquireLock */

#else /* SX126x_USE_ACCESS_LOCK */

// access lock not used -> define empty macros
#define SX126xAcquireLock()       1
#define SX126xReleaseLock()

#endif /* SX126x_USE_ACCESS_LOCK */


static uint32_t sx126x_error_cnt = 0;


uint32_t SX126xCheckCmdError( bool reset_counter )
{
    uint32_t cnt = sx126x_error_cnt;
    if (reset_counter)
    {
        sx126x_error_cnt = 0;
    }
    return cnt;
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

    // operating mode after reset is STDBY_RC
    SX126xSetOperatingMode(MODE_STDBY_RC);
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

    uint8_t status = 0;
    // in case the command does not have any arguments, make sure the pointer is still valid and size is at least 1 to be able to read the status
    if (size == 0 || buffer == 0)
    {
      buffer = &status;
      size   = 1;
    }
    if (HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, SX126x_CMD_TIMEOUT)               != HAL_OK ||
        HAL_SPI_TransmitReceive(&RADIO_SPI, buffer, &status, 1, SX126x_CMD_TIMEOUT)            != HAL_OK ||
        !SX126x_CMD_STATUS_VALID(status)                                                                 ||
        ((size > 1) && (HAL_SPI_Transmit(&RADIO_SPI, &buffer[1], size - 1, SX126x_CMD_TIMEOUT) != HAL_OK)))
    {
        SX126x_ERROR("failed to send radio cmd (%x)", status);
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

    SX126xAcquireLock( );

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

    SX126xAcquireLock( );

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();
    success = SX126xSPIWrite( command, buffer, size );
    // no RADIO_SET_NSS_PIN(); as it will be timed precisely

    SX126xReleaseLock();

    return success;
}

bool SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    bool    success = true;
    uint8_t status  = 0;

    SX126xAcquireLock( );

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();

#if SX126x_CHECK_CMD_RETVAL


    if (HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, SX126x_CMD_TIMEOUT)      != HAL_OK ||
        HAL_SPI_TransmitReceive(&RADIO_SPI, &status, &status, 1, SX126x_CMD_TIMEOUT)  != HAL_OK ||
        !SX126x_CMD_STATUS_VALID(status)                                                        ||
        HAL_SPI_TransmitReceive(&RADIO_SPI, buffer, buffer, size, SX126x_CMD_TIMEOUT) != HAL_OK)
    {
        SX126x_ERROR("failed to execute read cmd (%x)", status);
        success = false;
    }

#else

    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &command, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, (uint8_t*) &status, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&RADIO_SPI, (uint8_t*) buffer, buffer, size, SX126x_CMD_TIMEOUT);

#endif /* SX126x_CHECK_CMD_RETVAL */

    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();

    return success;
}

bool SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    bool    success = true;
    uint8_t cmd     = RADIO_WRITE_REGISTER;
    uint8_t addr[2];

    addr[0] = address >> 8;    // MSB first!
    addr[1] = address & 0xff;

    SX126xAcquireLock( );

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();

#if SX126x_CHECK_CMD_RETVAL

    if (HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, 100)                     != HAL_OK ||
        HAL_SPI_Transmit(&RADIO_SPI, addr, 2, SX126x_CMD_TIMEOUT)      != HAL_OK ||
        HAL_SPI_Transmit(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT) != HAL_OK)
    {
        SX126x_ERROR("failed to write registers");
        success = false;
    }

#else /* SX126x_CHECK_CMD_RETVAL */

    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(&RADIO_SPI, addr, 2, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT);

#endif /* SX126x_CHECK_CMD_RETVAL */

    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();

    return success;
}

bool SX126xWriteRegister( uint16_t address, uint8_t value )
{
    return SX126xWriteRegisters( address, &value, 1 );
}

bool SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    bool    success = true;
    uint8_t cmd     = RADIO_READ_REGISTER;
    uint8_t addr_ret[3];        // address and return value

    addr_ret[0] = address >> 8;     // MSB first!
    addr_ret[1] = address & 0xff;
    addr_ret[2] = 0;

    SX126xAcquireLock( );

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();

#if SX126x_CHECK_CMD_RETVAL

    if (HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT)     != HAL_OK ||
        HAL_SPI_Transmit(&RADIO_SPI, addr_ret, 3, SX126x_CMD_TIMEOUT) != HAL_OK ||
        !SX126x_CMD_STATUS_VALID(addr_ret[2])                                   ||
        HAL_SPI_Receive(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT) != HAL_OK)
    {
        SX126x_ERROR("failed to read registers");
        success = false;
    }

#else /* SX126x_CHECK_CMD_RETVAL */

    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, addr_ret, 3, SX126x_CMD_TIMEOUT);
    HAL_SPI_Receive(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT);

#endif /* SX126x_CHECK_CMD_RETVAL */

    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();

    return success;
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data = 0;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

bool SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    bool    success = true;
    uint8_t cmd     = RADIO_WRITE_BUFFER;

    SX126xAcquireLock( );

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();

#if SX126x_CHECK_CMD_RETVAL

    if (HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT)      != HAL_OK ||
        HAL_SPI_Transmit(&RADIO_SPI, &offset, 1, SX126x_CMD_TIMEOUT)   != HAL_OK ||
        HAL_SPI_Transmit(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT) != HAL_OK)
    {
        SX126x_ERROR("failed to write buffer");
        success = false;
    }

#else /* SX126x_CHECK_CMD_RETVAL */

    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, &offset, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, buffer, size, SX126x_CMD_TIMEOUT);

#endif /* SX126x_CHECK_CMD_RETVAL */

    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock();

    return success;
}

bool SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    bool    success = true;
    uint8_t cmd     = RADIO_READ_BUFFER;
    uint8_t status  = 0;

    SX126xAcquireLock( );

    SX126xCheckDeviceReady( );

    RADIO_CLR_NSS_PIN();

#if SX126x_CHECK_CMD_RETVAL

    if (HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT)                    != HAL_OK ||
        HAL_SPI_Transmit(&RADIO_SPI, &offset, 1, SX126x_CMD_TIMEOUT)                 != HAL_OK ||
        HAL_SPI_TransmitReceive(&RADIO_SPI, &status, &status, 1, SX126x_CMD_TIMEOUT) != HAL_OK ||
        !SX126x_CMD_STATUS_VALID(status)                                                       ||
        HAL_SPI_Receive(&RADIO_SPI, buffer, 255, SX126x_CMD_TIMEOUT)                 != HAL_OK)
    {
        SX126x_ERROR("failed to read buffer");
        success = false;
    }

#else /* SX126x_CHECK_CMD_RETVAL */

    HAL_SPI_Transmit(&RADIO_SPI, &cmd, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, &offset, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Transmit(&RADIO_SPI, &status, 1, SX126x_CMD_TIMEOUT);
    HAL_SPI_Receive(&RADIO_SPI, buffer, 255, SX126x_CMD_TIMEOUT);

#endif /* SX126x_CHECK_CMD_RETVAL */

    RADIO_SET_NSS_PIN();
    delay_us(1);          // wait at least 600ns before continuing

    SX126xWaitOnBusy( );

    SX126xReleaseLock( );

    return success;
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

