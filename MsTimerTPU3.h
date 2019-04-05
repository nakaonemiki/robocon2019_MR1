#ifndef MsTimerTPU3_h
#define MsTimerTPU3_h

#if defined (__RX600__)
#define F_CPU 48000000 //PCLK is 48MHz, CPU is operating at 96MHz
#else
#include <avr/interrupt.h>
#endif

namespace MsTimerTPU3 {
    extern unsigned long msecs;
    extern void (*func)();
    extern volatile unsigned long count;
    extern volatile char overflowing;
    extern volatile unsigned int tcnt2;

    void set(unsigned long ms, void (*f)());
    void start();
    void stop();
    void _overflow();

    //変更しました！
}

#endif
