#ifndef RNG9_H
#define RNG9_H
#include <stdint.h>
/* SINE9 post-processor from do_an_htn, separated from ADC/DMA/UART.
 * Each 12-bit ADC sample produces one 9-bit word (bit0..bit8).
 * Not a verified entropy source / cryptographic RNG.
 */
typedef struct { double phase; } RNG9_State;
void RNG9_Reset(RNG9_State *state);
uint16_t RNG9_Next(RNG9_State *state, uint16_t adc);
#endif
