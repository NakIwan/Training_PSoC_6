#ifndef WAVE_H
    #define WAVE_H

    #include <stdint.h>

    /* Number of elements in the wave sound track */
//    #define WAVE_SIZE 59752u
    #define WAVE_SIZE 97998u
//    #define WAVE_SIZE 127950u

    /* Extern reference to the wave sound track data */
    extern const int16_t wave_data[WAVE_SIZE];

#endif
