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

#include <stdlib.h>
#include <stdint.h>

#include "corridormap/memory.h"

namespace corridormap {

inline void* align_ptr(void* ptr, size_t value)
{
    return reinterpret_cast<void*>((reinterpret_cast<uintptr_t>(ptr) + (value - 1)) & ~(value - 1));
}

void* memory_malloc::allocate(size_t size, size_t align)
{
    align = align < sizeof(uint32_t) ? sizeof(uint32_t) : align;
    void* p = malloc(sizeof(uint32_t) + align + size);

    uint32_t* pad = static_cast<uint32_t*>(p);
    pad[0] = 0;
    ++pad;

    uint32_t* data = static_cast<uint32_t*>(align_ptr(pad, align));

    for (; pad < data; ++pad)
    {
        pad[0] = 0xffffffff;
    }

    return data;
}

void memory_malloc::deallocate(void* ptr)
{
    if (!ptr)
    {
        return;
    }

    uint32_t* p = reinterpret_cast<uint32_t*>(ptr);

    do
    {
        --p;
    }
    while (p[0] == 0xffffffff);

    free(p);
}

}
