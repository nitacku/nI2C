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
 * @file        slave_sender.ino
 * @summary     Slave transmit example. For use with master_reader example.
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
void TxCallback(void);


void setup(void)
{
    // Assign slave address
    g_twi.SetLocalDeviceAddress(ADDRESS_I2C);
    
    // Set receiver callback
    g_twi.SetSlaveTransmitHandler(TxCallback);

    // Initialize Serial communication
    Serial.begin(9600);
}


void loop(void)
{
    delay(1000);
}

// Function that executes whenever data is requested by master.
// This function is registered as a callback, see setup().
void TxCallback(void)
{
    static uint8_t x = 0;
    
    char message[] = "x is ?"; // Null terminator added by compiler
    message[5] = (x % 26) + 65; // Store value into message
    
    Serial.print("Writing: ");
    Serial.println(message);
    
    // Send message to master
    uint8_t status = g_twi.SlaveQueueNonBlocking(message, 6);
    
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
    
    x++;
}
