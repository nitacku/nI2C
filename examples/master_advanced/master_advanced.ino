/*
 * Copyright (c) 2018 PhotonicFusion LLC
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
 * @file        master_advanced.ino
 * @summary     Non-blocking read/write utilizing advanced library features
 * @version     1.0
 * @author      nitacku
 * @data        16 August 2018
 */

/*
    |___FEATURE_____________________________|__Wire_|__nI2C_|
    |                                       |       |       |
    | Non-blocking Read & Write.            |  NO   |  YES  |
    |                                       |       |       |
    | Can Read/Write more than 32 bytes     |  NO   |  YES  |
    | with single command.                  |       |       |
    |                                       |       |       |
    | User configurable timeout.            |  NO   |  YES  |
    |                                       |       |       |
    | Uses less than 50 bytes of SRAM       |  NO   |  YES  |
    | when idle.                            |       |       |
    |                                       |       |       |
    | Interrupt safe.                       |  NO   |  YES  |
    | (can be used in interrupts)           |       |       |
    |                                       |       |       |
    | Provides support for slave device     |  NO   |  YES  |
    | register addresses.                   |       |       |
    |                                       |       |       |
    | Provides ability to delay between     |  NO   |  YES  |
    | writes (required for many EEPROMs)    |       |       |
    |                                       |       |       |
    | Adjusts slave device communication    |  NO   |  YES  |
    | speed automatically.                  |       |       |
    |_______________________________________|_______|_______|


 Introduction:
--------------------------------------------------------------------------------
 As you are probably aware, the default Arduino I2C library named 'Wire' has
 a few setbacks. Wire lacks the ability to read/write more than 16 bytes at a
 time. Wire consumes precious SRAM with redundant buffers. Wire was not designed
 to be interrupt safe. And the biggest problem of all, Wire is a blocking
 implementation that freezes the main process during I2C transactions.
 
 nI2C is a solution to Wire's problems. With nI2C, you can read and write as
 many bytes as you like without needing to perform some convoluted reassembly.
 nI2C is non-blocking I2C access, so there's minimal penalty to the main
 process. nI2C takes care of all the bothersome intricacies and gives you back
 your development time! And with all those clock cycles saved, there's even more
 room to be more amazing! 
 

 Description:
--------------------------------------------------------------------------------
 This example is aimed at demonstrating some of the more advanced features
 of nI2C and what really sets it apart from Wire and other libraries out there.
 
 Here are some of the features this example is demonstrating
 * Write 100 bytes with a single command
 * Read 100 bytes and wait until completion
 * Read 100 bytes and allow a callback to signal completion
 * Write with specified delay after completion
 * Interrupt safety
 * User configurable timeout
 * Automatic handling of slave device register address
 
*/

#include <nI2C.h>

enum address_i2c_t : byte
{
    ADDRESS_I2C = 0x08 // Address of I2C device
};

// Global handle to I2C device
CI2C::Handle g_i2c_handle;

// Global buffer for received I2C data
// Could be allocated locally as long as the buffer remains in scope for the
// duration of the I2C transfer. Easiest guarantee is to simply make global.
uint8_t g_rx_buffer[100] = {0};

// Callback function prototype
void RxCallback(const uint8_t error);


void setup(void)
{
    // You may register up to 63 devices with unique addresses. Each device
    // must be registered and receive a unique handle. Simply pass a device's
    // handle to the library to communicate with the device. Devices are allowed
    // to have different address sizes and different communication speeds.
    
    // Register device with address size of 1 byte using Fast communication (400KHz)
    g_i2c_handle = nI2C->RegisterDevice(ADDRESS_I2C, 1, CI2C::Speed::FAST);
    
    // Any transaction that takes longer than the configured timeout will
    // exit and generate an error status. Use this to force reads to complete
    // even if the slave device is stalling (clock-stretching). You may
    // configure the timeout to 0ms if no timeout is desired.
    nI2C->SetTimeoutMS(100); // Set timeout to 100ms in this example.
    
    // Initialize Serial communication
    Serial.begin(9600);
    
    // Millisecond timer interrupt
    OCR0A = 0x7D;
    TIMSK0 |= _BV(OCIE0A);
}


void loop(void)
{
    const uint8_t read_address = 0;
    const uint8_t write_address = 0;
    uint8_t tx_buffer[32];
    
    // Initialize the data buffer
    memset(tx_buffer, 'A', sizeof(tx_buffer));
    
    while (true)
    {
        // Perform non-blocking write.
        // Write 100 bytes to slave device at address <write_address>.
        // Slave device target address will be automatically written.
        uint8_t status = nI2C->Write(g_i2c_handle, write_address, tx_buffer, sizeof(tx_buffer));
        
        if (!status)
        {
            // Perform non-blocking read.
            // RxCallback() will be invoked upon completion of transfer.
            // Request 100 bytes from slave device at address <read_address>.
            // Slave device target address with be automatically written.
            status = nI2C->Read(g_i2c_handle, read_address, g_rx_buffer, sizeof(g_rx_buffer), RxCallback);
            
            if (!status)
            {
                // Perform blocking read.
                // When no callback is specified, the read will wait until all bytes are
                // received or a timeout occurs before returning.
                // Slave device target address will NOT be automatically written.
                status = nI2C->Read(g_i2c_handle, g_rx_buffer, sizeof(g_rx_buffer));
                
                if (!status)
                {
                    PrintBuffer();
                }
            }
        }
        
        if (status)
        {
            /*
            Status values are as follows:
            0:success
            1:busy
            2:timeout
            3:data too long to fit in transmit buffer
            4:memory allocation failure
            5:attempted illegal transition of state
            6:received NACK on transmit of address
            7:received NACK on transmit of data
            8:illegal start or stop condition on bus
            9:lost bus arbitration to other master
            */
            
            Serial.print("Communication Status #: ");
            Serial.println(status);
        }
        
        delay(500);
    }
}


void PrintBuffer(void)
{
    const uint8_t column_width = 25;
    
    // Print data received
    for (uint8_t index = 0; index < sizeof(g_rx_buffer); index++)
    {
        if ((index % column_width) == 0)
        {
            Serial.println();
        }
        
        Serial.print(g_rx_buffer[index], HEX);
    }
    
    Serial.println();
}


void RxCallback(const uint8_t status)
{
    // Check that no errors occurred
    if (status == 0)
    {
        PrintBuffer();
    }
    else
    {
        /*
        Status values are as follows:
        0:success
        1:busy
        2:timeout
        3:data too long to fit in transmit buffer
        4:memory allocation failure
        5:attempted illegal transition of state
        6:received NACK on transmit of address
        7:received NACK on transmit of data
        8:illegal start or stop condition on bus
        9:lost bus arbitration to other master
        */
        
        Serial.print("Communication Status #: ");
        Serial.println(status);
    }
}


// Interrupt is called every millisecond
ISR(TIMER0_COMPA_vect) 
{
    static uint16_t elapsed_ms = 0;
    
    // Only process every 500ms
    if (elapsed_ms++ > 500)
    {
        const uint8_t delay_ms = 10;
        const uint8_t message[] = "Writing with 10ms delay. ";
        
        // Write to slave device during system interrupt and delay after completion
        // Slave device target address with NOT be automatically written.
        nI2C->Write(g_i2c_handle, message, sizeof(message), delay_ms);
        elapsed_ms = 0; // Reset
    }
}
