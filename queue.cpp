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
 * @file        queue.h
 * @summary     Dynamically allocated circular buffer queue implementation
 * @version     1.1
 * @author      nitacku
 * @data        15 July 2018
 */

#include "queue.h"

void* memcpy_fast(void* dst, const void* src, uint16_t num)
{
    asm volatile(
         "  movw r30, %[src]        \n\t"
         "  movw r26, %[dst]        \n\t"
         "  sbrs %A[num], 0         \n\t"
         "  rjmp Lcpyeven_%=        \n\t"
         "  rjmp Lcpyodd_%=         \n\t"
         "Lcpyloop_%=:              \n\t"
         "  ld __tmp_reg__, Z+      \n\t"
         "  st X+, __tmp_reg__      \n\t"
         "Lcpyodd_%=:               \n\t"
         "  ld __tmp_reg__, Z+      \n\t"
         "  st X+, __tmp_reg__      \n\t"
         "Lcpyeven_%=:              \n\t"
         "  subi %A[num], 2         \n\t"
         "  brcc Lcpyloop_%=        \n\t"
         "  sbci %B[num], 0         \n\t"
         "  brcc Lcpyloop_%=        \n\t"
         : [num] "+r" (num)
         : [src] "r" (src),
           [dst] "r" (dst)
         : "memory"
         );
    return dst;
}
