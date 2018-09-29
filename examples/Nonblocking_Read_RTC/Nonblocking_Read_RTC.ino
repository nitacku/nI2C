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
 * @file        Nonblocking_Read_RTC.ino
 * @summary     Non-blocking read example using the PCF2129 RTC device
 * @version     1.0
 * @author      nitacku
 * @data        16 August 2018
 */

#include <inttypes.h>
#include <nI2C.h>

enum address_i2c_t : uint8_t
{
    ADDRESS_I2C = 0x51, // Address of I2C device
};

// Device register addresses for PCF2129 RTC
enum address_t : uint8_t
{
    ADDRESS_CONTROL_1       = 0x00,
    ADDRESS_CONTROL_2       = 0x01,
    ADDRESS_CONTROL_3       = 0x02,
    ADDRESS_TIME            = 0x03,
    ADDRESS_DATE            = 0x06,
    ADDRESS_ALARM           = 0x0A,
    ADDRESS_CLOCKOUT        = 0x0F,
    ADDRESS_TIMESTAMP       = 0x12,
};

// Global handle to I2C device
CI2C::Handle g_i2c_handle;

// Global buffer for received I2C data
// Could be allocated locally as long as the buffer remains in scope for the
// duration of the I2C transfer. Easiest guarantee is to simply make global.
uint8_t g_rx_buffer[7];

// Callback function prototype
void RxCallback(const uint8_t error);


void setup(void)
{
    // Register device with address size of 1 byte using Fast communication (400KHz)
    g_i2c_handle = nI2C->RegisterDevice(ADDRESS_I2C, 1, CI2C::Speed::FAST);
    
    // Initialize Serial communication
    Serial.begin(9600);
}


void loop(void)
{
    // Perform non-blocking I2C read
    // RxCallback() will be invoked upon completion of I2C transfer
    uint8_t status = nI2C->Read(g_i2c_handle, ADDRESS_TIME, g_rx_buffer, sizeof(g_rx_buffer), RxCallback);
    
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
    
    // Wait 1 second before transmitting again
    delay(1000);
}

// Convert BCD numbers to human-readable format
uint8_t BCD_to_DEC(const uint8_t b)
{
    return (b - (6 * (b >> 4)));
}


void RxCallback(const uint8_t status)
{
    // Check that no errors occurred
    if (status == 0)
    {
        uint8_t second = BCD_to_DEC(g_rx_buffer[0]);
        uint8_t minute = BCD_to_DEC(g_rx_buffer[1]);
        uint8_t hour   = BCD_to_DEC(g_rx_buffer[2]);
        
        char time[15];
        const char* format_string = PSTR("time: %02u:%02u:%02u");
        snprintf_P(time, 15, format_string, hour, minute, second);
        
        Serial.println(time);
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
