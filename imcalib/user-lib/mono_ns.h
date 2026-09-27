#ifndef __MONO_NS_H
#define __MONO_NS_H

#include <stdint.h>

/* SYSCLK (MHz) */
#define MONO_NS_CPU_MHZ   550u

void     Mono_Ns_Init(void);
void     Mono_Ns_Tick(void);
uint64_t Mono_Ns_Get(void);

#endif
