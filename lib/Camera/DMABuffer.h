#pragma once
#include <cstdlib>

#include "rom/lldesc.h"

class DMABuffer {
   public:
    lldesc_t descriptor;
    unsigned char *buffer;
    DMABuffer(int bytes) {
        buffer = new unsigned char[bytes];
        descriptor.length = bytes;
        descriptor.size = descriptor.length;
        descriptor.owner = 1;
        descriptor.sosf = 1;
        descriptor.buf = reinterpret_cast<uint8_t *>(buffer);
        descriptor.offset = 0;
        descriptor.empty = 0;
        descriptor.eof = 1;
        descriptor.qe.stqe_next = 0;
    }

    void next(DMABuffer *next) {
        descriptor.qe.stqe_next = &(next->descriptor);
    }

    int sampleCount() const { return descriptor.length / 4; }

    ~DMABuffer() {
        if (buffer) delete[] buffer;
    }
};