/*
 * Copyright (c) 2018 nitacku
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * @file        nI2C.h
 * @summary     Generic I2C Master/Slave Interface
 * @version     2.1
 * @author      nitacku
 * @data        15 July 2018
 */

#ifndef _I2C_H_
#define _I2C_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <inttypes.h>
#include "nTWI.h"

// Using singleton design pattern
class CI2C
{
    public:
    
    enum status_t : uint8_t
    {
        STATUS_OK = 0,
        STATUS_BUSY,
        STATUS_TIMEOUT,
        STATUS_ERROR_BUFFER_OVERFLOW,
        STATUS_ERROR_MEMORY_ALLOCATION,
        STATUS_ERROR_STATE_MISMATCH,
        STATUS_ERROR_NO_DEVICE_RESPONSE,
        STATUS_ERROR_DEVICE_REJECTED_DATA,
        STATUS_ERROR_ILLEGAL_START_STOP,
        STATUS_ERROR_LOST_ARBITRATION,
    };
    
    enum class Speed : uint8_t
    {
        SLOW = (uint8_t)CTWI::Speed::SLOW,
        FAST = (uint8_t)CTWI::Speed::FAST,
        INIT = (uint8_t)CTWI::Speed::INIT, // Value I2C is initialized with
    };
    
    struct Handle
    {
        Handle(void)
            : device_address{0}
            , address_size{0}
            , speed{Speed::INIT}
        {
            // empty
        }
        
        Handle(const uint8_t _device_address,
               const uint8_t _address_size,
               const Speed _speed)
            : device_address(_device_address)
            , address_size(_address_size)
            , speed(_speed)
        {
            // empty
        }
        
        uint8_t device_address;
        uint8_t address_size;
        Speed speed;
    };
    
    private:
    
    const uint32_t DISCARD_ADDRESS = -1;
    
    // Reduce clutter and allow type-casting
    typedef void(*Callback)(const CTWI::status_t status);
    
    enum class Mode : uint8_t
    {
        READ,
        WRITE,
    };
    
    enum limit_t : uint8_t
    {
        MAX_ADDRESS = 0x80,
    };

    Speed m_speed;
    bool m_comm_active;
    uint16_t m_timeout;
    CTWI m_twi; // Two-wire interface instance

    public:

    CI2C(CI2C const&)           = delete; // Forbid constructor 
    void operator=(CI2C const&) = delete; // Forbid constructor
    
    // Return reference to CI2C for accessing methods
    static CI2C& GetInstance()
    {
        static CI2C m_instance; // Guaranteed to be destroyed
        return m_instance;      // Instantiated on first use
    }
    
    // Return true when i2c peripheral is in use
    bool IsCommActive(void);
    // Set the timeout interval to wait for operation to complete
    void SetTimeoutMS(const uint16_t timeout);
    // Register new device with <device_address> and internal memory <address_size> at <speed>
    const Handle RegisterDevice(const uint8_t device_address, const uint8_t address_size, const Speed speed);
    // Write <bytes> amount of <data> to device's memory at <address> with <delay_ms> pause between transmits
    CI2C::status_t Write(const Handle& handle, const uint32_t address, const uint8_t data[], const uint32_t bytes, const uint8_t delay_ms = 0);
    // Read <bytes> amount from device's memory at <address> into <data>
    CI2C::status_t Read(const Handle& handle, const uint32_t address, uint8_t data[], const uint32_t bytes, void(*callback)(const uint8_t status) = nullptr);
    // Write <bytes> amount of <data> to device with <delay_ms> pause between transmits
    CI2C::status_t Write(const Handle& handle, const uint8_t data[], const uint32_t bytes, const uint8_t delay_ms = 0);
    // Read <bytes> amount from device into <data>
    CI2C::status_t Read(const Handle& handle, uint8_t data[], const uint32_t bytes, void(*callback)(const uint8_t status) = nullptr);
    
    private:
    
    CI2C(void); // Default constructor
    CI2C::status_t PrepareForTransfer(const Mode mode, const Handle& handle, const uint32_t address, const uint32_t bytes, const uint8_t capacity);
    CI2C::status_t BlockingTransfer(const Mode mode, const Handle& handle, const uint32_t address, uint8_t data[], const uint32_t bytes, const uint8_t delay_ms = 0);
    CI2C::status_t QueuedTransfer(const Mode mode, const Handle& handle, const uint32_t address, uint8_t data[], const uint32_t bytes, const uint8_t delay_ms = 0, Callback callback = nullptr);
    static void ReadCallback(const CTWI::status_t status);
    CI2C::status_t WaitForComplete(void);
};

extern CI2C* nI2C;

#endif
