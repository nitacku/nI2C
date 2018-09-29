# nI2C
A non-blocking I2C library for Arduino/AVR without limitations!

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
 
