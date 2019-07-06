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
 * @file        nTWI.cpp
 * @summary     Interrupt-safe Non-blocking I2C Master/Slave Interface
 * @version     1.2
 * @author      nitacku
 * @data        15 July 2018
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include "nTWI.h"
#include "avrlibdefs.h"

static CTWI* g_callback_object = nullptr; // Global pointer to class object

CTWI::CTWI(void)
    : m_state{State::IDLE}
    , m_status{STATUS_OK}
    , m_reset{true}
    , m_timeout{100}
#ifdef CTWI_ENABLE_SLAVE_MODE
    , m_slave_rx_callback{nullptr}
    , m_slave_tx_callback{nullptr}
#endif
{
    // Activate internal pullups for TWI
    // Required for 400KHz operation
    *portOutputRegister(digitalPinToPort(SDA)) |= digitalPinToBitMask(SDA);
    *portOutputRegister(digitalPinToPort(SCL)) |= digitalPinToBitMask(SCL);

    // set i2c bit rate
    SetSpeed(Speed::INIT);
    
    // enable TWI (two-wire interface)
    sbi(TWCR, TWEN);
    // enable TWI interrupt and slave address ACK
    sbi(TWCR, TWIE);
    sbi(TWCR, TWEA);
    
#ifdef CTWI_USING_BLOCKING_ACCESS
    m_buffer = m_tx_buffer; // Assign buffer pointer
#endif

    g_callback_object = this;
}


void CTWI::SetSpeed(const CTWI::Speed speed)
{
    uint8_t bitrate_div;
    // set i2c bitrate
    // SCL freq = F_CPU/(16+2*TWBR))
#ifdef TWPS0
    // for processors with additional bitrate division (mega128)
    // SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
    // set TWPS to zero
    cbi(TWSR, TWPS0);
    cbi(TWSR, TWPS1);
#endif
    uint16_t value = ((speed == Speed::SLOW) ? 100 : 400);

    // calculate bitrate division
    bitrate_div = ((F_CPU / 1000l) / value);

    if (bitrate_div >= 16)
    {
        bitrate_div = (bitrate_div - 16) / 2;
    }

    outb(TWBR, bitrate_div);
}


void CTWI::SetTimeoutMS(const uint16_t timeout)
{
    m_timeout = timeout;
}

#ifdef CTWI_ENABLE_SLAVE_MODE
void CTWI::SetLocalDeviceAddress(const uint8_t address)
{
#ifndef CTWI_USING_BLOCKING_ACCESS
    // Will not be allocated if this function is never used
    static uint8_t buffer[SIZE_BUFFER + 1];
    // Assign to allocated memory
    m_slave_buffer = buffer;
#else
    m_slave_buffer = m_buffer;
#endif // CTWI_USING_BLOCKING_ACCESS
    
    // set local device address (used in slave mode only)
    outb(TWAR, (address << 1));
}


void CTWI::SetSlaveReceiveHandler(void (*slave_rx_callback)(const uint8_t data[], const uint8_t length))
{
    m_slave_rx_callback = slave_rx_callback;
}


void CTWI::SetSlaveTransmitHandler(void (*slave_tx_callback)(void))
{
    m_slave_tx_callback = slave_tx_callback;
}
#endif // CTWI_ENABLE_SLAVE_MODE


#ifndef CTWI_USING_BLOCKING_ACCESS
uint8_t CTWI::GetQueueVacancy(void)
{
    // Return current number of vacant elements
    return m_queue.Vacancy();
}

CTWI::status_t CTWI::MasterQueueNonBlocking(CTWI::Packet& packet, const CTWI::Register* const register_address)
{
    // A locked state occurs if the queue is full and interrupts are disabled.
    // This condition is prevented by the upper nI2C class.
    
    if (m_queue.IsFull())
    {
        // Not sure how much time to wait, choose a large value
        // Could peer into front of queue and guess...
        uint32_t timeout = 100000;
        
        // Wait for queued transmit to complete
        while (m_queue.IsFull() && --timeout);
        
        if (timeout == 0)
        {
            Stop();
            ProcessQueue();
            return STATUS_BUSY;
        }
    }
    
    // Only writes have data to copy at this point
    if (packet.mode == Mode::WRITE)
    {
        uint8_t offset = 0;
        
        if (register_address != nullptr)
        {
            offset = register_address->address_size;
        }
        // Boundary check
        if ((packet.length + offset) > SIZE_BUFFER)
        {
            return STATUS_ERROR_BUFFER_OVERFLOW;
        }
        
        uint8_t sreg = SREG; // Save register
        cli(); // Memory operators are not reentrant - halt interrupts
        // Allocate memory for message copy
        uint8_t* message = new uint8_t[packet.length + offset];
        SREG = sreg; // Restore register

        // Verify allocation was successful
        if (message != nullptr)
        {
            // Copy data into message
            memcpy_fast(&message[offset], packet.data, packet.length);
            
            // Copy register (if defined) into message
            for (uint8_t index = 0; index < offset; index++)
            {
                message[index] = register_address->address.byte[offset - index - 1];
                packet.length++; // Add register byte
            }
            
            packet.data = message; // Re-assign data pointer
        }
        else
        {
            return STATUS_ERROR_MEMORY_ALLOCATION;
        }
    }

    // Interrupts are disabled to prevent reentry into the following section
    uint8_t sreg = SREG; // Save register
    cli();
    bool is_empty = m_queue.IsEmpty(); // Save status
    m_queue.Push(packet); // Copy packet to queue
    
    // Kick-start transmit if single packet in queue
    if (is_empty)
    {
        ProcessQueue();
    }
    
    SREG = sreg; // Restore register
    
    return m_status;
}

#ifdef CTWI_ENABLE_SLAVE_MODE
CTWI::status_t CTWI::SlaveQueueNonBlocking(const uint8_t data[], const uint8_t length)
{
    // Check that state is correct
    if (m_state != State::SLAVE_TX)
    {
        return STATUS_ERROR_STATE_MISMATCH;
    }
    
    uint8_t sreg = SREG; // Save Register
    cli(); // Prevent modification to state while appending data
    
    // Length must only be reset after transmitting buffer since this method
    // is allowed to be called many times to append data to buffer.
    // Check if length should be reset
    if (m_reset == true)
    {
        m_buffer_length = 0;
        m_reset = false;
    }
    
    // Boundary check
    if ((length + m_buffer_length) > SIZE_BUFFER)
    {
        SREG = sreg; // Restore register
        return STATUS_ERROR_BUFFER_OVERFLOW;
    }
    
    memcpy_fast(&m_slave_buffer[m_buffer_length], data, length);
    m_buffer_length += length;
    SREG = sreg; // Restore register
    
    return m_status;
}
#endif

// Entry into this method must only be on completion of packet with a non-empty
// queue or when kick-starting transmit with previously empty queue.
void CTWI::ProcessQueue(void)
{
    // Retrieve next packet
    Packet* packet = m_queue.Front();
    uint8_t RW;
    
    if (packet->mode == Mode::WRITE)
    {
        m_state = State::MASTER_TX;
        RW = TW_WRITE;
    }
    else
    {
        m_state = State::MASTER_RX;
        RW = TW_READ;
    }
    
    m_buffer = packet->data; // Assign pointer
    m_device_address = ((packet->device_address << 1) | RW);
    m_buffer_length = packet->length;
    m_buffer_index = 0;
    m_status = STATUS_OK;
    SendStart(); // Begin transmit
}

// Entry into this method is only via ISR
void CTWI::ProcessCallback(void)
{
    // Retrieve current packet
    Packet* packet = m_queue.Front();
    
    // Only writes need to deallocate memory
    if (packet->mode == Mode::WRITE)
    {
        // Deallocate memory
        // No need to disable interrupts since called within ISR
        delete[] packet->data;
    }
    
    if (packet->callback != nullptr)
    {
        // Invoke user callback
        packet->callback(m_status);
    }
    
    // Optional user requested delay on success
    if ((packet->delay > 0) && !m_status)
    {
        // Check if interrupts are enabled
        if (SREG & 0x80)
        {
            delay(packet->delay);
        }
        else
        {
            // Calculate cycles needed for delay
            uint32_t count = ((F_CPU / 4000) * packet->delay);
            
            while (--count)
            {
                __asm__ __volatile__ ("nop");
            }
        }
    }
    
    // Remove packet
    // No need to disable interrupts since called within ISR
    m_queue.Pop();
    
    // Are there more packets to process?
    if (!m_queue.IsEmpty())
    {
        ProcessQueue();
    }
}

#else // CTWI_USING_BLOCKING_ACCESS

CTWI::status_t CTWI::MasterWriteBlocking(const uint8_t address)
{
    // Must be IDLE to accept request
    if (m_state != State::IDLE)
    {
        return STATUS_BUSY;
    }

    m_state = State::MASTER_TX;
    m_reset = true; // Allow reset of m_tx_length
    m_status = STATUS_OK;
    m_device_address = ((address << 1) | TW_WRITE); // RW cleared: write operation
    m_buffer_index = 0;
    SendStart();

    // Wait for transmit to complete
    return WaitForComplete();
}


CTWI::status_t CTWI::MasterReadBlocking(const uint8_t address, uint8_t data[], const uint8_t length)
{
    // Must be IDLE to accept request
    if (m_state != State::IDLE)
    {
        return STATUS_BUSY;
    }

    m_state = State::MASTER_RX;
    m_status = STATUS_OK;
    m_device_address = ((address << 1) | TW_READ); // RW set: read operation
    m_buffer_index = 0;
    m_buffer_length = length;
    m_buffer = data; // Assign buffer to user buffer
    SendStart();
    
    // Wait for transmit to complete
    status_t status = WaitForComplete();
    
    if (status != STATUS_OK)
    {
        m_buffer = m_tx_buffer; // Restore buffer pointer
        return status;
    }
    
    // User buffer contains received data at this point
    m_buffer = m_tx_buffer; // Restore buffer pointer

    return STATUS_OK;
}


CTWI::status_t CTWI::MasterQueueBlocking(const uint8_t data[], const uint8_t length)
{
    // Check that state is correct
    if ((m_state != State::SLAVE_TX) && (m_state != State::IDLE))
    {
        return STATUS_ERROR_STATE_MISMATCH;
    }
    
    uint8_t sreg = SREG; // Save register
    cli(); // Prevent modification to state while appending data
    
    // Length must only be reset after transmitting buffer since this method
    // is allowed to be called many times to append data to buffer.
    // Check if length should be reset
    if (m_reset == true)
    {
        m_buffer_length = 0;
        m_reset = false;
    }
    
    // Boundary check
    if ((length + m_buffer_length) > SIZE_BUFFER)
    {
        SREG = sreg; // Restore register
        return STATUS_ERROR_BUFFER_OVERFLOW;
    }
    
    memcpy_fast(&m_buffer[m_buffer_length], data, length);
    m_buffer_length += length;
    SREG = sreg; // Restore register
    
    return STATUS_OK;
}


#ifdef CTWI_ENABLE_SLAVE_MODE
CTWI::status_t CTWI::SlaveQueueBlocking(const uint8_t data[], const uint8_t length)
{
    return MasterQueueBlocking(data, length); // Master & Slave use same method to queue
}
#endif


// This simply provides a framework for the alternative method using Queued
// transfers. When CTWI_USING_BLOCKING_ACCESS defined, this method isn't used.
void CTWI::ProcessCallback(void)
{
    // empty
}


CTWI::status_t CTWI::WaitForComplete(void)
{
    unsigned long end = millis() + (unsigned long)m_timeout;
    
    while (m_state != State::IDLE)
    {
        // Check if timeout is set
        if (m_timeout > 0)
        {
            // Check current time
            if (millis() > end)
            {
                // AVR i2c state machine has potential to lockup
                // Reset i2c bus
                TWCR &= ~(_BV(TWEN));
                TWCR |= (_BV(TWEN));
                m_state = State::IDLE;
                return STATUS_TIMEOUT;
            }
        }
    }

    return m_status;
}

#endif // CTWI_USING_BLOCKING_ACCESS


inline void CTWI::SendStart(void)
{
    // send start condition
    outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT) | _BV(TWSTA));
}


inline void CTWI::Stop(void)
{
    // transmit stop condition
    // leave with TWEA on for slave receiving
    outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT) | _BV(TWEA) | _BV(TWSTO));

    // Stop should execute very quickly
    uint8_t timeout = 100;
    
    // Wait for stop condition to be executed on bus
    while ((TWCR & _BV(TWSTO)) && --timeout);
    
    // Check if AVR i2c bus is locked
    if (timeout == 0)
    {
        // Reset i2c bus
        TWCR &= ~(_BV(TWEN));
        TWCR |= (_BV(TWEN));
    }

    m_state = State::IDLE;
}


inline void CTWI::SendByte(const uint8_t data)
{
    outb(TWDR, data); // save data to the TWDR
}


inline void CTWI::SendACK(void)
{
    outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT) | _BV(TWEA));
}


inline void CTWI::SendNACK(void)
{
    outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT));
}


inline uint8_t CTWI::GetReceivedByte(void)
{
    // retrieve received data byte from i2c TWDR
    return (inb(TWDR));
}


inline void CTWI::InterruptHandlerWrapper(void)
{
    g_callback_object->InterruptHandler();
}


//! I2C (TWI) interrupt service routine
ISR(TWI_vect)
{
    CTWI::InterruptHandlerWrapper();
}


void CTWI::InterruptHandler(void)
{
    switch (TW_STATUS)
    {
    // Master General
    case TW_START:                     // 0x08: Sent start condition
    case TW_REP_START:                 // 0x10: Sent repeated start condition
        SendByte(m_device_address); // send device address
        SendACK(); // begin send
        break;

    // Master Transmitter & Receiver status codes
    case TW_MT_SLA_ACK:                // 0x18: Slave address acknowledged
    case TW_MT_DATA_ACK:               // 0x28: Data acknowledged
        if (m_buffer_index < m_buffer_length)
        {
            SendByte(m_buffer[m_buffer_index++]); // send data
            SendACK(); // begin send
        }
        else
        {
            Stop(); // transmit stop condition, enable SLA ACK
            // i2c transmit is complete
            ProcessCallback(); // Remove packet from queue
        }

        break;
        
    case TW_MT_ARB_LOST:               // 0x38: Bus arbitration lost
    //case TW_MR_ARB_LOST:               // 0x38: Bus arbitration lost
        SendNACK();
        m_state = State::IDLE;
        //m_status = STATUS_ERROR_LOST_ARBITRATION; // Not a true error
        // Start condition will automatically transmit when bus becomes free
        // No need to process further
        break;

    case TW_MT_SLA_NACK:               // 0x20: Slave address not acknowledged
    case TW_MR_SLA_NACK:               // 0x48: Slave address not acknowledged
    case TW_MT_DATA_NACK:              // 0x30: Data not acknowledged
        m_status = ((TW_STATUS == TW_MT_DATA_NACK) ? STATUS_ERROR_DEVICE_REJECTED_DATA : STATUS_ERROR_NO_DEVICE_RESPONSE);
        Stop(); // transmit stop condition, enable SLA ACK
        ProcessCallback(); // Remove packet from queue
        break;

    case TW_MR_DATA_NACK:              // 0x58: Data received, NACK reply issued
        // prevent buffer overflow
        if (m_buffer_index < m_buffer_length)
        {
            // store final received data byte
            m_buffer[m_buffer_index++] = GetReceivedByte();
        }
        Stop(); // transmit stop condition, enable SLA ACK
        // i2c receive is complete
        ProcessCallback(); // Remove packet from queue
        break;

    case TW_MR_DATA_ACK:               // 0x50: Data acknowledged
        // prevent buffer overflow
        if (m_buffer_index < m_buffer_length)
        {
            // store received data byte
            m_buffer[m_buffer_index++] = GetReceivedByte();
        }
        // fall-through to see if more bytes will be received
        [[gnu::fallthrough]]; // Fall-through
    case TW_MR_SLA_ACK:                // 0x40: Slave address acknowledged
        if (m_buffer_index < (m_buffer_length - 1))
        {
            // data byte will be received, reply with ACK (more bytes in transfer)
            SendACK();
        }
        else
        {
            // data byte will be received, reply with NACK (final byte in transfer)
            SendNACK();
        }
        
        break;
#ifdef CTWI_ENABLE_SLAVE_MODE
    // Slave Receiver status codes
    case TW_SR_SLA_ACK:                // 0x60: own SLA+W has been received, ACK has been returned
    case TW_SR_ARB_LOST_SLA_ACK:       // 0x68: own SLA+W has been received, ACK has been returned
    case TW_SR_GCALL_ACK:              // 0x70:     GCA+W has been received, ACK has been returned
    case TW_SR_ARB_LOST_GCALL_ACK:     // 0x78:     GCA+W has been received, ACK has been returned
        // device being addressed as slave for writing (data will be received from master)
        m_state = State::SLAVE_RX;
        m_buffer_index = 0;
        SendACK(); // receive data byte and return ACK
        break;

    case TW_SR_DATA_ACK:               // 0x80: data byte has been received, ACK has been returned
    case TW_SR_GCALL_DATA_ACK:         // 0x90: data byte has been received, ACK has been returned
        // check receive buffer status
        if (m_buffer_index < SIZE_BUFFER)
        {
            m_slave_buffer[m_buffer_index++] = GetReceivedByte();
            SendACK(); // receive data byte and return ACK
        }
        else
        {
            SendNACK(); // receive data byte and return NACK
        }
        
        break;

    case TW_SR_DATA_NACK:              // 0x88: data byte has been received, NACK has been returned
    case TW_SR_GCALL_DATA_NACK:        // 0x98: data byte has been received, NACK has been returned
        SendNACK(); // receive data byte and return NACK
        break;

    case TW_SR_STOP:                   // 0xA0: STOP or REPEATED START has been received while addressed as slave
        SendACK(); // ACK future responses and leave slave receiver state
        m_state = State::IDLE;

        // i2c receive is complete, call m_slave_rx_callback
        if (m_slave_rx_callback != nullptr)
        {            
            return m_slave_rx_callback(m_slave_buffer, m_buffer_index); // tail-recursion
        }
        
        break;

    // Slave Transmitter
    case TW_ST_SLA_ACK:                // 0xA8: own SLA+R has been received, ACK has been returned
    case TW_ST_ARB_LOST_SLA_ACK:       // 0xB0:     GCA+R has been received, ACK has been returned
        // addressed as slave for reading (data must be transmitted back to master)
        m_state = State::SLAVE_TX;
        
        // TODO: Why is this getting called repeatedly during a single request?
        // Block multiple calls using m_reset hack
        if (m_reset == true)
        {
            // request data from application
            if (m_slave_tx_callback != nullptr)
            {
                m_slave_tx_callback();
            }

            // reset data index
            m_buffer_index = 0;
        }
        [[gnu::fallthrough]]; // Fall-through
    // fall-through to transmit first data byte
    case TW_ST_DATA_ACK:               // 0xB8: data byte has been transmitted, ACK has been received

        if (m_buffer_index < m_buffer_length)
        {
            SendByte(m_slave_buffer[m_buffer_index++]); // transmit data byte
            SendACK(); // expect ACK to data byte
        }
        else
        {
            SendByte(m_slave_buffer[0]); // transmit data byte
            SendNACK(); // expect NACK to data byte
        }
        
        break;

    case TW_ST_DATA_NACK:              // 0xC0: data byte has been transmitted, NACK has been received
    case TW_ST_LAST_DATA:              // 0xC8: ACK received, but there is no more data to transmit
        SendACK(); // switch to open slave
        m_reset = true; // Allow buffer to reset
        m_state = State::IDLE;
        break;
#endif
    // Misc
    default:
    case TW_NO_INFO:                   // 0xF8: No relevant state information
        // do nothing
        break;

    case TW_BUS_ERROR:                 // 0x00: Bus error due to illegal start or stop condition
        m_status = STATUS_ERROR_ILLEGAL_START_STOP;
        Stop(); // reset internal hardware and release bus
        ProcessCallback(); // Remove packet from queue
        break;
    }
}
