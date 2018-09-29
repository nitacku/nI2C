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
 * @file        slave_reader.ino
 * @summary     Slave receive example. For use with master_writer example.
 * @version     1.0
 * @author      nitacku
 * @data        16 August 2018
 */

#include <nI2C.h>

enum address_i2c_t : byte
{
    ADDRESS_I2C = 0x08 // Address of I2C device
};

// Global TWI object
CTWI g_twi;

// Callback function prototype
void RxCallback(const uint8_t data[], const uint8_t length);


void setup(void)
{
    // Assign slave address
    g_twi.SetLocalDeviceAddress(ADDRESS_I2C);
    
    // Set receiver callback
    g_twi.SetSlaveReceiveHandler(RxCallback);
    
    // Initialize Serial communication
    Serial.begin(9600);
}


void loop(void)
{
    delay(100);
}

// Function that executes whenever data is received from master.
// This function is registered as a callback, see setup().
void RxCallback(const uint8_t data[], const uint8_t length)
{
    for (uint8_t index = 0; index < length; index++)
    {
        char c = data[index];
        Serial.print(c);
    }
    
    Serial.println();
}
