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
 * @file        nI2C.cpp
 * @summary     Generic I2C Master/Slave Interface
 * @version     2.1
 * @author      nitacku
 * @data        15 July 2018
 */

#include "nI2C.h"

#define ASSERT_STATUS(X) do { CTWI::status_t status = X; if (status) { m_comm_active = false; return (CI2C::status_t)status; } } while (0)

// Assign global object pointer
CI2C* nI2C = &CI2C::GetInstance();

static volatile bool g_rx_complete = false;
static volatile CI2C::status_t g_status = CI2C::status_t::STATUS_OK;

CI2C::CI2C()
    : m_speed(CI2C::Speed::INIT)
    , m_comm_active(false)
    , m_timeout(100)
{
    // Empty
}


bool CI2C::IsCommActive(void)
{
    return m_comm_active;
};


void CI2C::SetTimeoutMS(const uint16_t timeout)
{
    m_timeout = timeout;
    m_twi.SetTimeoutMS(m_timeout);
}


const CI2C::Handle CI2C::RegisterDevice(const uint8_t device_address, const uint8_t address_size, const CI2C::Speed speed)
{
    if (device_address < MAX_ADDRESS)
    {
        if ((address_size > 0) && (address_size < 5))
        {
            return {device_address, address_size, speed};
        }
    }
    
    return {}; // Error
}


CI2C::status_t CI2C::Read(const CI2C::Handle& handle, const uint32_t address, uint8_t data[], const uint32_t bytes, void(*callback)(const uint8_t error))
{
#ifdef CTWI_USING_BLOCKING_ACCESS
    (void)(callback); // Silence compiler warning
    return BlockingTransfer(Mode::READ, handle, address, data, bytes, 0);
#else
    return QueuedTransfer(Mode::READ, handle, address, data, bytes, 0, (Callback)callback);
#endif
}


CI2C::status_t CI2C::Write(const Handle& handle, const uint32_t address, const uint8_t data[], const uint32_t bytes, const uint8_t delay_ms)
{
#ifdef CTWI_USING_BLOCKING_ACCESS
    return BlockingTransfer(Mode::WRITE, handle, address, const_cast<uint8_t*>(data), bytes, delay_ms);
#else
    return QueuedTransfer(Mode::WRITE, handle, address, const_cast<uint8_t*>(data), bytes, delay_ms);
#endif
}


CI2C::status_t CI2C::Read(const CI2C::Handle& handle, uint8_t data[], const uint32_t bytes, void(*callback)(const uint8_t error))
{
#ifdef CTWI_USING_BLOCKING_ACCESS
    (void)(callback); // Silence compiler warning
    return BlockingTransfer(Mode::READ, handle, DISCARD_ADDRESS, data, bytes);
#else
    return QueuedTransfer(Mode::READ, handle, DISCARD_ADDRESS, data, bytes, 0, (Callback)callback);
#endif
}


CI2C::status_t CI2C::Write(const Handle& handle, const uint8_t data[], const uint32_t bytes, const uint8_t delay_ms)
{
#ifdef CTWI_USING_BLOCKING_ACCESS
    return BlockingTransfer(Mode::WRITE, handle, DISCARD_ADDRESS, const_cast<uint8_t*>(data), bytes, delay_ms);
#else
    return QueuedTransfer(Mode::WRITE, handle, DISCARD_ADDRESS, const_cast<uint8_t*>(data), bytes, delay_ms);
#endif
}


CI2C::status_t CI2C::PrepareForTransfer(const Mode mode, const CI2C::Handle& handle, const uint32_t address, const uint32_t bytes, const uint8_t capacity)
{
    // Check if active
    if (!IsCommActive())
    {
        // Check if handle valid
        if (handle.address_size > 0)
        {
            // Check if interrupts are disabled
            if (!(SREG & 0x80))
            {
#ifndef CTWI_USING_BLOCKING_ACCESS
                uint8_t packet_count = 1;
                
                if (mode == Mode::WRITE)
                {
                    if (bytes > capacity)
                    {
                        return STATUS_BUSY;
                    }
                }
                else
                {
                    if (address != DISCARD_ADDRESS)
                    {
                        packet_count++;
                    }
                }
            
                // Since interrupts are disabled, there is no risk that the queue
                // is modified between checking it and starting transfer.
                // Check that required packets will not exceed queue availability
                if (packet_count > m_twi.GetQueueVacancy())
                {
                    return STATUS_BUSY;
                }
#else
                // Silence compiler warnings
                (void)mode;
                (void)address;
                (void)bytes;
                (void)capacity;
                
                // Blocking implementation locks with interrupts disabled
                return STATUS_BUSY;
#endif
            }
            
            // Flag as active
            m_comm_active = true;
            
            // TODO: Speed should be queued along with packets otherwise speed
            // may change before queued packets are completed
            // Check if speed different
            if (m_speed != handle.speed)
            {
                m_twi.SetSpeed((handle.speed == Speed::SLOW) ? CTWI::Speed::SLOW : CTWI::Speed::FAST);
                m_speed = handle.speed;
            }
            
            return STATUS_OK;
        }
        
        return STATUS_ERROR_MEMORY_ALLOCATION;
    }

    return STATUS_BUSY;
}


CI2C::status_t CI2C::BlockingTransfer(const CI2C::Mode mode, const CI2C::Handle& handle, const uint32_t address, uint8_t data[], const uint32_t bytes, const uint8_t delay_ms)
{
    // Calculate buffer remaining capacity
    uint8_t capacity = (CTWI::size_t::SIZE_BUFFER - handle.address_size);
    
    // Determine if transfer should occur
    status_t status = PrepareForTransfer(mode, handle, address, bytes, capacity);
    
    if (status == STATUS_OK)
    {
        if (mode == Mode::READ)
        {
            // Check if should skip
            if (address != DISCARD_ADDRESS)
            {
                // Calculate new address
                CTWI::Address register_address{address};
                
                // Add slave destination register to queue
                for (uint8_t index = handle.address_size; index > 0; index--)
                {
                    ASSERT_STATUS(m_twi.MasterQueueBlocking(const_cast<uint8_t*>(&register_address.byte[index - 1])));
                }
                
                // Write destination register to slave
                ASSERT_STATUS(m_twi.MasterWriteBlocking(handle.device_address));
            }

            // Request bytes from slave
            ASSERT_STATUS(m_twi.MasterReadBlocking(handle.device_address, data, bytes));
        }
        else
        {
            uint32_t bytes_transmitted = 0;
            
            while (bytes_transmitted < bytes)
            {
                // Calculate new address
                CTWI::Address register_address{address + bytes_transmitted};
                // Check if remaining byte count greater than buffer capacity
                uint32_t bytes_remaining = (bytes - bytes_transmitted);
                uint8_t bytes_to_transmit = (bytes_remaining > capacity) ? capacity : bytes_remaining;
                
                // Check if should skip
                if (address != DISCARD_ADDRESS)
                {
                    // Add slave destination register to queue
                    for (uint8_t index = handle.address_size; index > 0; index--)
                    {
                        ASSERT_STATUS(m_twi.MasterQueueBlocking(const_cast<uint8_t*>(&register_address.byte[index - 1])));
                    }
                }

                // Add data to queue
                ASSERT_STATUS(m_twi.MasterQueueBlocking(const_cast<uint8_t*>(&data[bytes_transmitted]), bytes_to_transmit));

                // Write bytes to slave
                ASSERT_STATUS(m_twi.MasterWriteBlocking(handle.device_address));
                
                // Optional delay between successful transfers
                delay(delay_ms);
                
                // Increment byte counter
                bytes_transmitted += bytes_to_transmit;
            }
        }

        // Relinquish access
        m_comm_active = false;

        return STATUS_OK;
    }
    
    return status; // BUSY or ERROR
}


CI2C::status_t CI2C::QueuedTransfer(const CI2C::Mode mode, const CI2C::Handle& handle, const uint32_t address, uint8_t data[], const uint32_t bytes, const uint8_t delay_ms, Callback callback)
{
    // Calculate buffer remaining capacity
    uint8_t capacity = (CTWI::size_t::SIZE_BUFFER - handle.address_size);
    
    // Determine if transfer should occur
    status_t status = PrepareForTransfer(mode, handle, address, bytes, capacity);
    
    if (status == STATUS_OK)
    {
        if (mode == Mode::READ)
        {
            if (address != DISCARD_ADDRESS)
            {
                // Create empty data packet
                CTWI::Packet packet{CTWI::Mode::WRITE, handle.device_address, nullptr, 0, 0, nullptr};
                CTWI::Register device_register{address, handle.address_size};
                
                // Queue destination register
                ASSERT_STATUS(m_twi.MasterQueueNonBlocking(packet, &device_register));
            }

            bool block = (callback == nullptr);
            g_rx_complete = false;
            
            // Create packet to read bytes
            CTWI::Packet packet{CTWI::Mode::READ, handle.device_address, data, bytes, 0, (block ? ReadCallback : callback)};

            // Queue read request
            ASSERT_STATUS(m_twi.MasterQueueNonBlocking(packet));
            
            if (block)
            {
                status_t wait_status = WaitForComplete();
                
                if (wait_status)
                {
                    m_comm_active = false;
                    return wait_status;
                }
            }
        }
        else
        {
            uint32_t bytes_transmitted = 0;
            
            while (bytes_transmitted < bytes)
            {
                // Create device destination register definition
                CTWI::Register device_register{address + bytes_transmitted, handle.address_size};
                // Check if remaining byte count greater than buffer capacity
                uint32_t bytes_remaining = (bytes - bytes_transmitted);
                uint8_t bytes_to_transmit = (bytes_remaining > capacity) ? capacity : bytes_remaining;
            
                // Create data packet
                CTWI::Packet packet{CTWI::Mode::WRITE, handle.device_address, &data[bytes_transmitted], bytes_to_transmit, delay_ms, nullptr};
                // Queue write request
                ASSERT_STATUS(m_twi.MasterQueueNonBlocking(packet, (address != DISCARD_ADDRESS) ? &device_register : nullptr));
                // Increment byte counter
                bytes_transmitted += bytes_to_transmit;
            }
        }

        // Relinquish access
        m_comm_active = false;

        return STATUS_OK;
    }
    
    return status; // BUSY or ERROR
}


void CI2C::ReadCallback(const CTWI::status_t status)
{
    g_status = (CI2C::status_t)status;
    g_rx_complete = true;
}


CI2C::status_t CI2C::WaitForComplete(void)
{
    unsigned long end = millis() + (unsigned long)m_timeout;

    // Wait while transmission incomplete
    while (!g_rx_complete)
    {
        // Check if timeout is set
        if (m_timeout > 0)
        {
            // Check current time
            if (millis() > end)
            {
                return STATUS_TIMEOUT; // Timeout
            }
        }
    }
    
    return g_status;
}
