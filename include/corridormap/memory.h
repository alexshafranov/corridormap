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

#include <stddef.h> // size_t

namespace corridormap {

class memory
{
public:
    virtual ~memory() {}
    virtual void* allocate(size_t size, size_t align)=0;
    virtual void  deallocate(void* ptr)=0;
};

class memory_malloc : public memory
{
public:
    virtual void* allocate(size_t size, size_t align);
    virtual void  deallocate(void* ptr);
};

template <typename T>
T* allocate(memory* mem, size_t size, size_t align=sizeof(void*))
{
    return static_cast<T*>(mem->allocate(size, align));
}

}

#endif
