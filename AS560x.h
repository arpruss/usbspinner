/**
 * AS560x Driver
 *
 * Arduino Library
 * for AS560x Magnet Rotary Sensor
 *
 * License: BSD 3-Clause
 *
 * Nick Schwarzenberg,
 * 06/2016, v0.1.0
 */


// include these dependencies in your top-level .ino!
#include <Arduino.h>
#include <Wire.h>


// prevent redefinitions
#ifndef AS560x_driver
#define AS560x_driver


class AS560x
{
    public:

        /* :: Configuration :: */


        // 7-bit device address
        unsigned char address = 0x36;

        // 8-bit register addresses
        static struct ByteRegister {
            static const unsigned char
            ZMCO = 0x00,
            ABN = 0x09,
            PUSHTHR = 0x0A,
            STATUS = 0x0B,
            AGC = 0x1A,
            BURN = 0xFF;
        } ByteRegister;

        // 16-bit register start (high byte) addresses
        static struct WordRegister {
            static const unsigned char
            ZPOS = 0x01,
            CONF = 0x03,
            RAWANGLE = 0x0C,
            ANGLE = 0x0E,
            MAGNITUDE = 0x1B;
        } WordRegister;

        // initialization with explicit I2C channel
        // must call begin() on the channel before using
        AS560x(WireBase *i2cChannel)
        {
            // host I2C bus as master in the passed channel
            wireChannel = i2cChannel;
        }

        /* :: Low-Level Access :: */


        // low-level: read one byte from an 8-bit register
        unsigned char readRaw8( unsigned char registerAddress )
        {
            // send START for I2C frame to the AS560x
            wireChannel->beginTransmission( this->address );

            // send register address
            wireChannel->write( registerAddress );

            // flush, but do not release bus (no STOP)
            wireChannel->endTransmission( false );

            // request one byte as response from the AS560x, release and wait
            // (casting is necessary due to similar declarations of .requestFrom)
            wireChannel->requestFrom( (uint8_t) this->address, (uint8_t) 1, (uint8_t) true );

            // return response
            return wireChannel->read();
        }

        uint32_t readRaw16Atomic( unsigned char registerAddress )
        {
            // send START for I2C frame to the AS560x
            wireChannel->beginTransmission( this->address );

            // send register address
            wireChannel->write( registerAddress );

            // flush, but do not release bus (no STOP)
            wireChannel->endTransmission( false );

            // request one byte as response from the AS560x, release and wait
            // (casting is necessary due to similar declarations of .requestFrom)
            wireChannel->requestFrom( (uint8_t) this->address, (uint8_t) 2, (uint8_t) true );

            // return response
            uint8_t high = wireChannel->read();
            uint8_t low = wireChannel->read();
            return word(high,low);
        }

        // low-level: read two bytes as 16-bit word from two 8-bit registers
        unsigned int readRaw16( unsigned char registerStartAddress )
        {
            // get high byte, then low byte
            unsigned char highByte = this->readRaw8( registerStartAddress );
            unsigned char lowByte = this->readRaw8( registerStartAddress + 1 );

            // combine to 16-bit word
            return word( highByte, lowByte );
        }

        // low-level: write one byte to an 8-bit register
        void writeRaw8( unsigned char registerAddress, unsigned char value )
        {
            // send START for I2C frame to the AS560x
            wireChannel->beginTransmission( this->address );

            // send register address and value
            wireChannel->write( registerAddress );
            wireChannel->write( value );

            // flush and release (STOP)
            wireChannel->endTransmission( true );
        }

        // low-level: write 16-bit word as two bytes to two 8-bit registers
        void writeRaw16( unsigned char registerStartAddress, unsigned int value )
        {
            // send high byte, then low byte
            this->writeRaw8( registerStartAddress, (unsigned char) highByte( value ) );
            this->writeRaw8( registerStartAddress + 1, (unsigned char) lowByte( value ) );
        }



        /* :: Higher-Level Methods :: */


        // query status to find out if magnet is detected
        bool magnetDetected()
        {
            // query status register
            unsigned char status = this->readRaw8( AS560x::ByteRegister::STATUS );

            // return true if bit 5 is set
            return bitRead( status, 5 ) == 1 ? true : false;
        }

        // get current magnetic magnitude (12 bit)
        unsigned int getMagnitude()
        {
            // read and return two-byte magnitude
            return this->readRaw16Atomic( AS560x::WordRegister::MAGNITUDE );
        }

        // get current gain of AGC (8 bit)
        unsigned char getGain()
        {
            // read and return one-byte gain
            return this->readRaw8( AS560x::ByteRegister::AGC );
        }

        // get raw angle (12 bit)
        unsigned int getRawAngle()
        {
            // read and return two-byte raw angle
            return this->readRaw16Atomic( AS560x::WordRegister::RAWANGLE );
        }

        // set zero-position to specified raw angle (12 bit)
        void setZeroPosition( unsigned int rawAngle )
        {
            // send position setting command
            this->writeRaw16( AS560x::WordRegister::ZPOS, rawAngle );
        }

        // convenience method: read current raw angle and pass it to .setZeroPosition(rawAngle)
        void setZeroPosition() { this->setZeroPosition( this->getRawAngle() ); }

        // set angle resolution (affecting output value range and update speed)
        void setResolution( unsigned int angleSteps )
        {
            char power = -1;

            // coerce angle steps to supported values (8, 16, 32, …, 2048)
            angleSteps = min( max( angleSteps, 8 ), 2048 );

            // find dual logarithm (2^power >= angleSteps)
            // (by comparing increasing powers of two with angleSteps)
            while ( ( 1 << ++power ) < angleSteps );

            // send ABN setting command (-3 (2^3 = 8) shifts the powers 3…11 (for 8…2048) to 0…8)
            this->writeRaw8( AS560x::ByteRegister::ABN, power-3 );
        }

        // get zero-adjusted and filtered angle (12 bit)
        unsigned int getAngle()
        {
            // read and return two-byte clean angle
            return this->readRaw16Atomic( AS560x::WordRegister::ANGLE );
        }

    private:
        WireBase *wireChannel;
};


// see ifndef above
#endif
