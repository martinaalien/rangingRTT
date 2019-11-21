#include <stdint.h>
#include <stdbool.h>

#ifndef __TIMER_H__
#define __TIMER_H__

void timer0_init(void);
void timer0_start(uint32_t timeout_us);
bool timer0_timeout(void);
void timer0_capture_init(uint32_t prescaler);
void timer0_capture_start(void);
uint32_t timer0_capture_now(void);

#endif /* __TIMER_H__ */
