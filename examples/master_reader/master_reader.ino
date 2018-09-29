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
 * @file        master_reader.ino
 * @summary     Non-blocking read example. For use with slave_sender example.
 * @version     1.0
 * @author      nitacku
 * @data        16 August 2018
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
char g_rx_buffer[32] = {0};

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
    
    // Initialize Serial communication
    Serial.begin(9600);
}


void loop(void)
{
    // Perform non-blocking I2C read
    // RxCallback() will be invoked upon completion of I2C transfer
    // Request 6 bytes from slave device
    uint8_t status = nI2C->Read(g_i2c_handle, (uint8_t*)g_rx_buffer, 6, RxCallback);

    /*--------------------------------------------------------------------------
        Note: You may also want the alternative method listed below which
        automatically handles writing the slave register address before reading.
        This is the standard approach for RTCs and EEPROMs.
        
        Replace <address> with the slave device register address
        
        nI2C->Read(g_i2c_handle, <address>, message, sizeof(message));
    --------------------------------------------------------------------------*/
    
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


void RxCallback(const uint8_t status)
{
    // Check that no errors occurred
    if (status == 0)
    {
        Serial.print("Received: ");
        Serial.println(g_rx_buffer);
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
