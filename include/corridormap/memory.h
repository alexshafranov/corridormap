//
// Copyright (c) 2014 Alexander Shafranov <shafranov@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef CORRIDORMAP_MEMORY_H_
#define CORRIDORMAP_MEMORY_H_

#include <string.h> // memset
#include <stddef.h> // size_t

namespace corridormap {

class Memory
{
public:
    virtual ~Memory() {}
    virtual void* allocate(size_t size, size_t align)=0;
    virtual void  deallocate(void* ptr)=0;
};

class Memory_Malloc : public Memory
{
public:
    virtual void* allocate(size_t size, size_t align);
    virtual void  deallocate(void* ptr);
};

template <typename T>
T* allocate(Memory* mem, size_t count, size_t align=sizeof(void*))
{
    return static_cast<T*>(mem->allocate(sizeof(T)*count, align));
}

template <typename T>
class Alloc_Scope
{
public:
    Alloc_Scope(Memory* mem, size_t count, size_t align=sizeof(void*))
        : mem(mem)
        , count(count)
    {
        data = allocate<T>(mem, count, align);
    }

    ~Alloc_Scope()
    {
        mem->deallocate(data);
    }

    operator T*() { return data; }

    size_t count;
    Memory* mem;
    T* data;
};

template <typename T>
void zero_mem(Alloc_Scope<T>& s)
{
    memset(s.data, 0, s.count*sizeof(T));
}

template <typename T>
struct Dequeue
{
    Dequeue(Memory* mem, int max_size)
        : front(0)
        , size(0)
        , max_size(max_size)
        , mem(mem)
    {
        data = allocate<T>(mem, max_size);
    }

    ~Dequeue()
    {
        mem->deallocate(data);
    }

    int front;
    int size;
    int max_size;
    Memory* mem;
    T* data;
};

template <typename T>
int size(const Dequeue<T>& b)
{
    return b.size;
}

template <typename T>
void push_back(Dequeue<T>& b, const T val)
{
    int idx = (b.front + b.size) % b.max_size;
    b.data[idx] = val;
    b.size++;
}

template <typename T>
void push_front(Dequeue<T>&b, const T val)
{
    int idx = (b.front - 1) % b.max_size;
    b.data[idx] = val;
    b.size++;
}

template <typename T>
T pop_back(Dequeue<T>& b)
{
    T val = b.data[(b.front + b.size - 1) % b.max_size];
    b.size--;
    return val;
}

template <typename T>
T pop_front(Dequeue<T>& b)
{
    T val = b.data[b.front % b.max_size];
    b.front++;
    b.size--;
    return val;
}

template <typename T>
T& front(Dequeue<T>& b)
{
    return b.data[b.front % b.max_size];
}

template <typename T>
T& back(Dequeue<T>& b)
{
    int idx = (b.front + b.size - 1) % b.max_size;
    return b.data[idx];
}

template <typename T>
void clear(Dequeue<T>& b)
{
    b.front = 0;
    b.size = 0;
}

}

#endif
