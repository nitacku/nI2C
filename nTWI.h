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
 * @file        nTWI.h
 * @summary     Interrupt-safe Non-blocking I2C Master/Slave Interface
 * @version     1.2
 * @author      nitacku
 * @data        15 July 2018
 */

// Uncomment to enable slave mode
//#define CTWI_ENABLE_SLAVE_MODE

// Uncomment to use traditional blocking access
//#define CTWI_USING_BLOCKING_ACCESS

#ifndef TWI_H
#define TWI_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <inttypes.h>
#include "queue.h"

class CTWI
{
    public:
    
    enum size_t : uint8_t
    {
        SIZE_BUFFER = 36,
        SIZE_QUEUE = 4,
    };
    
    enum class Speed : uint8_t
    {
        SLOW,
        FAST,
        INIT = FAST,
    };
    
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
    
    enum class Mode : uint8_t
    {
        WRITE,
        READ,
    };
    
    union Address
    {
        Address(const uint32_t _value)
            : value{_value}
        {
            // empty
        }
            
        uint32_t value;
        uint8_t byte[sizeof(uint32_t)];
    };
    
    struct Register
    {
        Register(const Address _address,
                 const uint8_t _address_size)
            : address{_address}
            , address_size{_address_size}
        {
            // empty
        }
        
        Address address;
        uint8_t address_size;
    };
    
    struct Packet
    {
        Packet()
            : length{0}
            , delay{0}
            , callback{nullptr}
        {
            // empty
        }
        
        Packet(const Mode _mode,
               const uint8_t _device_address,
               uint8_t* _data,
               const uint32_t _length,
               const uint8_t _delay,
               void(*_callback)(const status_t status))
            : mode{_mode}
            , device_address{_device_address}
            , data{_data}
            , length{_length}
            , delay{_delay}
            , callback{_callback}
        {
            // empty
        }
        
        Mode mode;                                  // Read or Write
        uint8_t device_address;                     // Device I2C address
        uint8_t* data;                              // Pointer to read/write buffer
        uint32_t length;                            // Length of data to read/write
        uint8_t delay;                              // Optional delay on success
        void (*callback)(const status_t status);    // Optional callback on success
    };
    
    private:
    
    enum mask_t : uint8_t
    {
        MASK_TWCR_CMD = 0x0F,
    };

    // types
    enum class State : uint8_t
    {
        IDLE,
        MASTER_TX,
        MASTER_RX,
        SLAVE_TX,
        SLAVE_RX,
    };
    
    // I2C state and address variables
    volatile State m_state;             // Current I2C state
    volatile status_t m_status;         // Status
    volatile bool m_reset;              // Flag indicating reset needed on m_buffer_length
    uint16_t m_timeout;                 // Time in ms to wait for blocking operations
    uint8_t m_device_address;           // 7 bit slave device address
    volatile uint32_t m_buffer_index;   // Buffer index
    volatile uint32_t m_buffer_length;  // Buffer length
    uint8_t* m_buffer;                  // Data buffer
    
#ifdef CTWI_USING_BLOCKING_ACCESS
    uint8_t m_tx_buffer[SIZE_BUFFER + 1];  // Master/Slave data buffer
#else
    CQueue<Packet> m_queue{SIZE_QUEUE}; // Queue of packets
#endif

#ifdef CTWI_ENABLE_SLAVE_MODE
    uint8_t* m_slave_buffer;            // Slave data buffer
    // Called when this processor is addressed as a slave for writing
    void (*m_slave_rx_callback)(const uint8_t data[], const uint8_t length);
    // Called when this processor is addressed as a slave for reading
    void (*m_slave_tx_callback)(void);
#endif

    public:
    
    // Default constructor
    CTWI(void);
    // Get the number of vacant elements in the queue
    uint8_t GetQueueVacancy(void);
    // Set the I2C transaction bitrate
    void SetSpeed(const Speed speed);
    // Set the timeout interval to wait for operation to complete
    void SetTimeoutMS(const uint16_t timeout);
    // I2C setup and configurations commands
    // Set the local (AVR processor's) I2C device address
    void SetLocalDeviceAddress(const uint8_t address);
    // Set the user function which handles receiving (incoming) data as a slave
    void SetSlaveReceiveHandler(void (*slave_rx_callback)(const uint8_t data[], const uint8_t length));
    // Set the user function which handles transmitting (outgoing) data as a slave
    void SetSlaveTransmitHandler(void (*slave_tx_callback)(void));
    
    // high-level I2C transaction commands
    // Add packet to queue - Must be used in non-blocking mode
    status_t MasterQueueNonBlocking(Packet& packet, const Register* const register_address = nullptr);
    // Write I2C data to a slave on the bus - Must be used in blocking mode
    status_t MasterWriteBlocking(const uint8_t address);
    // Read I2C data from a slave on the bus - Must be used in blocking mode
    status_t MasterReadBlocking(const uint8_t address, uint8_t data[], const uint8_t length);
    // Add data to buffer - Must be used in blocking mode
    status_t MasterQueueBlocking(const uint8_t data[], const uint8_t length = 1);
#ifdef CTWI_ENABLE_SLAVE_MODE
    // Add data to buffer - Must be used in non-blocking mode
    status_t SlaveQueueNonBlocking(const uint8_t data[], const uint8_t length);
    // Add data to buffer - Must be used in blocking mode
    status_t SlaveQueueBlocking(const uint8_t data[], const uint8_t length = 1);
#endif
    
    private:
    
    // Process any entries in queue
    void ProcessQueue(void);
    // Called after transmit complete
    void ProcessCallback(void);
    // Wait for operation to complete
    status_t WaitForComplete(void);
    
    // Low-level I2C transaction commands
    // Send an I2C start condition in Master mode
    inline void SendStart(void) __attribute__((always_inline));
    // Send an I2C stop condition in Master mode
    void Stop(void);
    // Send an (address|R/W) combination or a data byte over I2C
    inline void SendByte(const uint8_t data) __attribute__((always_inline));
    // Send ACK
    inline void SendACK(void) __attribute__((always_inline));
    // Send NACK
    inline void SendNACK(void) __attribute__((always_inline));
    // Read byte
    inline uint8_t GetReceivedByte(void) __attribute__((always_inline));
    // Called by hardware I2C interrupt
    inline void InterruptHandler(void) __attribute__((always_inline));
    
    public:
    
    static inline void InterruptHandlerWrapper(void) __attribute__((always_inline));
};

#endif
