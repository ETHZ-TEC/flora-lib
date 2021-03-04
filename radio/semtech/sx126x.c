/*!
 * \file      sx126x.c
 *
 * \brief     SX126x driver implementation
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
 */

/*
 * reference implementation is available at https://github.com/Lora-net/LoRaMac-node/blob/master/src/radio/sx126x/sx126x.c
 */

#include "flora_lib.h"

/*!
 * \brief Internal frequency of the radio
 */
#define SX126X_XTAL_FREQ                            32000000UL

/*!
 * \brief Scaling factor used to perform fixed-point operations
 */
#define SX126X_PLL_STEP_SHIFT_AMOUNT                ( 14 )

/*!
 * \brief PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
 */
#define SX126X_PLL_STEP_SCALED                      ( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )

/*!
 * \brief Maximum value for parameter symbNum in \ref SX126xSetLoRaSymbNumTimeout
 */
#define SX126X_MAX_LORA_SYMB_NUM_TIMEOUT            248

/*!
 * \brief Radio registers definition
 */
typedef struct
{
    uint16_t      Addr;                             //!< The address of the register
    uint8_t       Value;                            //!< The value of the register
}RadioRegisters_t;

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode;

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t PacketType;

/*!
 * \brief Hold the status of the Image calibration
 */
static bool ImageCalibrated = false;

/*!
 * \brief Get the number of PLL steps for a given frequency in Hertz
 *
 * \param [in] freqInHz Frequency in Hertz
 *
 * \returns Number of PLL steps
 */
static uint32_t SX126xConvertFreqInHzToPllStep( uint32_t freqInHz );
/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xOnDioIrq( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xSetPollingMode( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xSetInterruptMode( void );

/*
 * \brief Process the IRQ if handled by the driver
 */
void SX126xProcessIrqs( void );

/*
 * External helper functions
 */

/*!
 * \brief Set the trim values for the XOSC (external XTAL)
 */
extern void RadioSetXoscTrim( void );

/*!
 * \brief Sets the whitening mode.
 *
 * \param [IN] whitening  Whitening mode [0: no whitening, 1: whitening enabled]
 */
extern void RadioSetGfskWhitening( uint8_t whitening );


/*
 * SX126x functions
 */

void SX126xInit( )
{
    SX126xReset( );
    ImageCalibrated = false;

    SX126xWakeup( );
    SX126xSetStandby( STDBY_XOSC );

#ifdef USE_TCXO
    CalibrationParams_t calibParam;

    SX126xSetDio3AsTcxoCtrl( TCXO_CTRL_1_7V, SX126xGetBoardTcxoWakeupTime( ) << 6 ); // convert from ms to SX126x time base
    calibParam.Value = 0x7F;
    SX126xCalibrate( calibParam );
#endif

    // adjust TX clamp config register to optimize the PA clamping threshold (workaround as described in the datasheet, p.103)
    uint8_t txclampcfg = SX126xReadRegister(0x08D8);
    SX126xWriteRegister(0x08D8, txclampcfg | 0x1E);

    SX126xSetDio2AsRfSwitchCtrl( true );
}

RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}

void SX126xSetOperatingMode( RadioOperatingModes_t mode )
{
    OperatingMode = mode;
}

void SX126xCheckDeviceReady( void )
{
    if( ( SX126xGetOperatingMode( ) == MODE_SLEEP ) || ( SX126xGetOperatingMode( ) == MODE_RX_DC ) )
    {
        SX126xWakeup( );
    }
    SX126xWaitOnBusy( );
}

void SX126xSetPayload( uint8_t *payload, uint8_t size )
{
    SX126xWriteBuffer( 0x00, payload, size );
}

uint8_t SX126xGetPayload( uint8_t *buffer, uint8_t *size,  uint8_t maxSize )
{
    uint8_t offset = 0;

    SX126xGetRxBufferStatus( size, &offset );
    if( *size > maxSize )
    {
        return 1;
    }
    SX126xReadBuffer( offset, buffer, *size );
    return 0;
}

void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout )
{
    SX126xSetPayload( payload, size );
    SX126xSetTx( timeout, true );
}

uint8_t SX126xSetSyncWord( uint8_t *syncWord )
{
    SX126xWriteRegisters( REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 );
    return 0;
}

void SX126xSetCrcSeed( uint16_t seed )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( seed >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( seed & 0xFF );

    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            SX126xWriteRegisters( REG_LR_CRCSEEDBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}

void SX126xSetCrcPolynomial( uint16_t polynomial )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( polynomial >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( polynomial & 0xFF );

    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            SX126xWriteRegisters( REG_LR_CRCPOLYBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}

void SX126xSetWhiteningSeed( uint16_t seed )
{
    uint8_t regValue = 0;

    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            regValue = SX126xReadRegister( REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE;
            regValue = ( ( seed >> 8 ) & 0x01 ) | regValue;
            SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_MSB, regValue ); // only 1 bit.
            SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_LSB, ( uint8_t )seed );
            break;

        default:
            break;
    }
}

uint32_t SX126xGetRandom( void )
{
    uint32_t number = 0;
    uint8_t regAnaLna = 0;
    uint8_t regAnaMixer = 0;

    regAnaLna = SX126xReadRegister( REG_ANA_LNA );
    SX126xWriteRegister( REG_ANA_LNA, regAnaLna & ~( 1 << 0 ) );

    regAnaMixer = SX126xReadRegister( REG_ANA_MIXER );
    SX126xWriteRegister( REG_ANA_MIXER, regAnaMixer & ~( 1 << 7 ) );

    // Set radio in continuous reception
    SX126xSetRx( 0xFFFFFF, true, false ); // Rx Continuous

    SX126xReadRegisters( RANDOM_NUMBER_GENERATORBASEADDR, ( uint8_t* )&number, 4 );

    SX126xSetStandby( STDBY_RC );

    SX126xWriteRegister( REG_ANA_LNA, regAnaLna );
    SX126xWriteRegister( REG_ANA_MIXER, regAnaMixer );

    return number;
}

void SX126xSetSleep( SleepParams_t sleepConfig )
{
    if( SX126xWriteCommand( RADIO_SET_SLEEP, &sleepConfig.Value, 1 ) )
    {
        SX126xAntSwOff( );

        if( sleepConfig.Fields.WarmStart == 0 )
        {
            ImageCalibrated = false;
        }

        OperatingMode = MODE_SLEEP;
    }
}

void SX126xSetStandby( RadioStandbyModes_t standbyConfig )
{
    // NOTE: Be aware when using MODE_STDBY_RC: XTAL trim values are overwritten by SX1262 state machine when switching to MODE_STDBY_RC, i.e. they need to be reapplied when switching to XOSC
    if( SX126xWriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 ) )
    {
        if( standbyConfig == STDBY_RC )
        {
            OperatingMode = MODE_STDBY_RC;
        }
        else
        {
            OperatingMode = MODE_STDBY_XOSC;
        }
    }
}

void SX126xSetFs( void )
{
    if( SX126xWriteCommand( RADIO_SET_FS, 0, 0 ) )
    {
        OperatingMode = MODE_FS;
    }
}

void SX126xSetTx( uint32_t timeout, bool execute )
{
    uint8_t buf[3];
    bool    success;

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );

    if( execute )
    {
        success = SX126xWriteCommand( RADIO_SET_TX, buf, 3 );
    }
    else
    {
        success = SX126xWriteCommandWithoutExecute( RADIO_SET_TX, buf, 3 );
    }
    if( success )
    {
        OperatingMode = MODE_TX;
    }
}

void SX126xSetRx( uint32_t timeout, bool execute, bool boosted )
{
    uint8_t buf[3];
    bool    success;

    if( boosted )
    {
        SX126xWriteRegister( REG_RX_GAIN, 0x96 ); // max LNA gain, increase current by ~2mA for around ~3dB in sensitivity
    }

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );

    if( execute )
    {
        success = SX126xWriteCommand( RADIO_SET_RX, buf, 3 );
    }
    else
    {
        success = SX126xWriteCommandWithoutExecute( RADIO_SET_RX, buf, 3 );
    }
    if( success )
    {
        if( timeout == 0xFFFFFF )
        {
            OperatingMode = MODE_RX_CONTINUOUS;
        }
        else
        {
            OperatingMode = MODE_RX;
        }
    }
}

void SX126xSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime, bool execute )
{
    uint8_t buf[6];
    bool    success;

    buf[0] = ( uint8_t )( ( rxTime >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( rxTime >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( rxTime & 0xFF );
    buf[3] = ( uint8_t )( ( sleepTime >> 16 ) & 0xFF );
    buf[4] = ( uint8_t )( ( sleepTime >> 8 ) & 0xFF );
    buf[5] = ( uint8_t )( sleepTime & 0xFF );

    if( execute )
    {
        success = SX126xWriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 6 );
    }
    else
    {
        success = SX126xWriteCommandWithoutExecute( RADIO_SET_RXDUTYCYCLE, buf, 6 );
    }
    if( success )
    {
        OperatingMode = MODE_RX_DC;
    }
}

void SX126xSetCad( void )
{
    if( SX126xWriteCommand( RADIO_SET_CAD, 0, 0 ) )
    {
        OperatingMode = MODE_CAD;
    }
}

void SX126xSetTxContinuousWave( void )
{
    if( SX126xWriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 ) )
    {
        OperatingMode = MODE_TX;
    }
}

void SX126xSetTxInfinitePreamble( void )
{
    if( SX126xWriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 ) )
    {
        OperatingMode = MODE_TX;
    }
}

void SX126xSetStopRxTimerOnPreambleDetect( bool enable )
{
    SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}

void SX126xSetLoRaSymbNumTimeout( uint8_t symbNum )
{
    uint8_t mant = ( ( ( symbNum > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT ) ?
                       SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : 
                       symbNum ) + 1 ) >> 1;
    uint8_t exp  = 0;
    uint8_t reg  = 0;

    while( mant > 31 )
    {
        mant = ( mant + 3 ) >> 2;
        exp++;
    }

    reg = mant << ( 2 * exp + 1 );
    SX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &reg, 1 );

    if( symbNum != 0 )
    {
        reg = exp + ( mant << 3 );
        SX126xWriteRegister( REG_LR_SYNCH_TIMEOUT, reg );
    }
}

void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xSetStandby( STDBY_RC ); // explicitly set to STDBY_RC since regulator mode should be set only in STDBY_RC mode
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
    SX126xSetXoscTrim(); // necessary since writing calibration values apparently resets standby mode to STDBY_RC which overwrites external XTAL trim calibration values
}

void SX126xCalibrate( CalibrationParams_t calibParam )
{
    SX126xWriteCommand( RADIO_CALIBRATE, &calibParam.Value, 1 );
}

void SX126xCalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    if( SX126xWriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 ) )
    {
        ImageCalibrated = true;
    }
}

void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    SX126xWriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}

void SX126xSetRxTxFallbackMode( uint8_t fallbackMode )
{
    SX126xWriteCommand( RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1 );
}

void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];

    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX126xWriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}

uint16_t SX126xGetIrqStatus( void )
{
    uint8_t irqStatus[2] = { 0 };

    SX126xReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    return ( (uint16_t)irqStatus[0] << 8 ) | irqStatus[1];
}

void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable )
{
    SX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}

void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout )
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( timeout & 0xFF );

    SX126xWriteCommand( RADIO_SET_TCXOMODE, buf, 4 );
}

void SX126xSetRfFrequency( uint32_t frequency )
{
    uint8_t buf[4];

    if( ImageCalibrated == false )
    {
        SX126xCalibrateImage( frequency );
    }

    uint32_t freqInPllSteps = SX126xConvertFreqInHzToPllStep( frequency );

    buf[0] = ( uint8_t )( ( freqInPllSteps >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freqInPllSteps >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freqInPllSteps >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freqInPllSteps & 0xFF );
    SX126xWriteCommand( RADIO_SET_RFFREQUENCY, buf, 4 );

    SX126xSetXoscTrim(); // sets radio chip to STDBY_XOSC and sets trim values for external XTAL
}

void SX126xSetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    if( SX126xWriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 ) )
    {
        PacketType = packetType;
    }
}

RadioPacketTypes_t SX126xGetPacketType( void )
{
    return PacketType;
}

void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];

    if( SX126xGetPaSelect( 0 ) == SX1261 )
    {
        if( power == 15 )
        {
            SX126xSetPaConfig( 0x06, 0x00, 0x01, 0x01 );
        }
        else
        {
            SX126xSetPaConfig( 0x04, 0x00, 0x01, 0x01 );
        }
        if( power >= 14 )
        {
            power = 14;
        }
        else if( power < -17 )
        {
            power = -17;
        }
        SX126xWriteRegister( REG_OCP, 0x18 ); // current max is 80 mA for the whole device
    }
    else // sx1262
    {
        SX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 );
        if( power > 22 )
        {
            power = 22;
        }
        else if( power < -9 )
        {
            power = -9;
        }
        SX126xWriteRegister( REG_OCP, 0x38 ); // current max 140mA for the whole device
    }
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

void SX126xSetModulationParams( ModulationParams_t *modulationParams )
{
    uint8_t n;
    uint32_t tempVal = 0;
    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != modulationParams->PacketType )
    {
        SX126xSetPacketType( modulationParams->PacketType );
    }

    switch( modulationParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        n = 8;
        tempVal = ( uint32_t )( 32 * SX126X_XTAL_FREQ / modulationParams->Params.Gfsk.BitRate );
        buf[0] = ( tempVal >> 16 ) & 0xFF;
        buf[1] = ( tempVal >> 8 ) & 0xFF;
        buf[2] = tempVal & 0xFF;
        buf[3] = modulationParams->Params.Gfsk.ModulationShaping;
        buf[4] = modulationParams->Params.Gfsk.Bandwidth;
        tempVal = SX126xConvertFreqInHzToPllStep( modulationParams->Params.Gfsk.Fdev );
        buf[5] = ( tempVal >> 16 ) & 0xFF;
        buf[6] = ( tempVal >> 8 ) & 0xFF;
        buf[7] = ( tempVal& 0xFF );
        SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
        break;
    case PACKET_TYPE_LORA:
        n = 4;
        buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
        buf[1] = modulationParams->Params.LoRa.Bandwidth;
        buf[2] = modulationParams->Params.LoRa.CodingRate;
        buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;

        SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );

        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
}

void SX126xSetPacketParams( PacketParams_t *packetParams )
{
    uint8_t n;
    uint8_t crcVal = 0;
    uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != packetParams->PacketType )
    {
        SX126xSetPacketType( packetParams->PacketType );
    }

    switch( packetParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM )
        {
            SX126xSetCrcSeed( CRC_IBM_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_IBM );
            crcVal = RADIO_CRC_2_BYTES;
        }
        else if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT )
        {
            SX126xSetCrcSeed( CRC_CCITT_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
            crcVal = RADIO_CRC_2_BYTES_INV;
        }
        else
        {
            crcVal = packetParams->Params.Gfsk.CrcLength;
        }
        n = 9;
        buf[0] = ( packetParams->Params.Gfsk.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.Gfsk.PreambleLength;
        buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
        buf[3] = ( packetParams->Params.Gfsk.SyncWordLength /*<< 3*/ ); // convert from byte to bit
        buf[4] = packetParams->Params.Gfsk.AddrComp;
        buf[5] = packetParams->Params.Gfsk.HeaderType;
        buf[6] = packetParams->Params.Gfsk.PayloadLength;
        buf[7] = crcVal;
        buf[8] = packetParams->Params.Gfsk.DcFree;
        break;
    case PACKET_TYPE_LORA:
        n = 6;
        buf[0] = ( packetParams->Params.LoRa.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.LoRa.PreambleLength;
        buf[2] = packetParams->Params.LoRa.HeaderType;
        buf[3] = packetParams->Params.LoRa.PayloadLength;
        buf[4] = packetParams->Params.LoRa.CrcMode;
        buf[5] = packetParams->Params.LoRa.InvertIQ;
        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }

    SX126xWriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
}

void SX126xSetCadParams( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout )
{
    uint8_t buf[7];

    buf[0] = ( uint8_t )cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = ( uint8_t )cadExitMode;
    buf[4] = ( uint8_t )( ( cadTimeout >> 16 ) & 0xFF );
    buf[5] = ( uint8_t )( ( cadTimeout >> 8 ) & 0xFF );
    buf[6] = ( uint8_t )( cadTimeout & 0xFF );
    if( SX126xWriteCommand( RADIO_SET_CADPARAMS, buf, 7 ) )
    {
        OperatingMode = MODE_CAD;
    }
}

void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

RadioStatus_t SX126xGetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status;

    SX126xReadCommand( RADIO_GET_STATUS, &stat, 1 );
    status.Value = stat;
    return status;
}

int8_t SX126xGetRssiInst( void )
{
    uint8_t buf[1];
    int8_t rssi = 0;

    SX126xReadCommand( RADIO_GET_RSSIINST, buf, 1 );
    rssi = -buf[0] >> 1;
    return rssi;
}

void SX126xGetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBufferPointer )
{
    uint8_t status[2];

    SX126xReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if( ( SX126xGetPacketType( ) == PACKET_TYPE_LORA ) && ( SX126xReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
    {
        *payloadLength = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
    }
    else
    {
        *payloadLength = status[0];
    }
    *rxStartBufferPointer = status[1];
}

void SX126xGetPacketStatus( PacketStatus_t *pktStatus )
{
    uint8_t status[3];

    SX126xReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );

    pktStatus->packetType = SX126xGetPacketType( );
    switch( pktStatus->packetType )
    {
        case PACKET_TYPE_GFSK:
            pktStatus->Params.Gfsk.RxStatus = status[0];
            pktStatus->Params.Gfsk.RssiSync = -status[1] >> 1;
            pktStatus->Params.Gfsk.RssiAvg = -status[2] >> 1;
            pktStatus->Params.Gfsk.FreqError = 0;
            break;

        case PACKET_TYPE_LORA:
            pktStatus->Params.LoRa.RssiPkt = -status[0] >> 1;
            // Returns SNR value [dB] rounded to the nearest integer value
            pktStatus->Params.LoRa.SnrPkt = ( ( ( int8_t )status[1] ) + 2 ) >> 2;
            pktStatus->Params.LoRa.SignalRssiPkt = -status[2] >> 1;
            pktStatus->Params.LoRa.FreqError = 0;
            break;

        default:
        case PACKET_TYPE_NONE:
            // In that specific case, we set everything in the pktStatus to zeros
            // and reset the packet type accordingly
            memset( pktStatus, 0, sizeof( PacketStatus_t ) );
            pktStatus->packetType = PACKET_TYPE_NONE;
            break;
    }
}

RadioError_t SX126xGetDeviceErrors( void )
{
    RadioError_t error;

    SX126xReadCommand( RADIO_GET_ERROR, ( uint8_t * )&error.Value, 2 );
    return error;
}

void SX126xClearDeviceErrors( void )
{
    uint8_t buf[2] = { 0x00, 0x00 };
    SX126xWriteCommand( RADIO_CLR_ERROR, buf, 2 );
}

void SX126xClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];

    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX126xWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}

static uint32_t SX126xConvertFreqInHzToPllStep( uint32_t freqInHz )
{
    uint32_t stepsInt;
    uint32_t stepsFrac;

    // pllSteps = freqInHz / (SX126X_XTAL_FREQ / 2^19 )
    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    stepsInt = freqInHz / SX126X_PLL_STEP_SCALED;
    stepsFrac = freqInHz - ( stepsInt * SX126X_PLL_STEP_SCALED );
    
    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( stepsInt << SX126X_PLL_STEP_SHIFT_AMOUNT ) + 
           ( ( ( stepsFrac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
             SX126X_PLL_STEP_SCALED );
}

void SX126xSetXoscTrim( void )
{
#ifndef USE_TCXO
    // Internal state machine of SX1262 overwrites trim values when switching to XOSC (happens whenever in Tx/Rx) -> set XOSC beforehand
    SX126xSetRxTxFallbackMode( 0x30 );  // always fallback to XOSC (not RC since state change overwrites trim values)
    SX126xSetStandby( STDBY_XOSC );     // set XOSC mode now
    // set values for XTAL trimming caps (calibration)
    SX126xWriteRegister( REG_XTA_TRIM, 0x0E );
    SX126xWriteRegister( REG_XTB_TRIM, 0x0F );
#endif
}
