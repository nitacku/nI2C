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

#ifndef QUEUE_H_
#define QUEUE_H_

#include <inttypes.h>

#if defined(__AVR__)
extern "C" {
void* memcpy_fast(void* dst, const void* src, uint16_t num) __attribute__ ((noinline));
}
#else
#define memcpy_fast memcpy
#endif


template <class T>
class CQueue
{
    public:
    
    CQueue(const uint8_t elements);
    ~CQueue(void);
    
    void Push(const T& element);
    void Pop(void);
    T* Front(void);
    void Clear(void);
    uint8_t Size(void);
    uint8_t Vacancy(void);
    bool IsFull(void);
    bool IsEmpty(void);
    
    private:
    
    uint8_t m_capacity;
    volatile uint8_t m_head;
    volatile uint8_t m_tail;
    volatile uint8_t m_size;
    T* m_array;

    void Destroy(void);
    uint8_t Next(const uint8_t position);
};

// Constructor
template <class T>
CQueue<T>::CQueue(const uint8_t elements)
{
    // Allocate memory for array
    if (elements > 0)
    {
        m_array = new T[elements];
    }

    m_capacity = elements;
    Clear();
}

// Destructor
template <class T>
CQueue<T>::~CQueue(void)
{
    Destroy();
}


template <class T>
uint8_t CQueue<T>::Next(const uint8_t position)
{
    if ((position + 1) >= m_capacity)
    {
        return 0;
    }

    return position + 1;
}


template <class T>
void CQueue<T>::Push(const T& element)
{
    if (m_array != nullptr) // Ensure array has been initialized
    {
        if (IsFull())
        {
            return; // Do nothing
            //Pop(); // CQueue is full. Remove head entry
        }

        // Copy element into queue
        memcpy_fast(&m_array[m_tail], &element, sizeof(T));

        m_size++;
        m_tail = Next(m_tail);
    }
}


template <class T>
void CQueue<T>::Pop(void)
{
    if (m_size > 0)
    {
        m_size--;
        m_head = Next(m_head);
    }
}


template <class T>
T* CQueue<T>::Front(void)
{
    return ((m_size > 0) ? &m_array[m_head] : nullptr);
}


template <class T>
void CQueue<T>::Clear(void)
{
    m_size = 0;
    m_head = 0;
    m_tail = 0;
}


template <class T>
uint8_t CQueue<T>::Size(void)
{
    return m_size;
}


template <class T>
uint8_t CQueue<T>::Vacancy(void)
{
    return m_capacity - m_size;
}


template <class T>
bool CQueue<T>::IsFull(void)
{
    return (m_size >= m_capacity);
}


template <class T>
bool CQueue<T>::IsEmpty(void)
{
    return (m_size == 0);
}


template <class T>
void CQueue<T>::Destroy(void)
{
    if (m_array != nullptr)
    {
        delete[] m_array;
    }

    m_array = nullptr; // Pointer now invalid
    m_size = 0;
}

#endif
