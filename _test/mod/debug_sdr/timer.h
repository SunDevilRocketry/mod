#ifndef TEST_DEBUG_SDR_TIMER_H
#define TEST_DEBUG_SDR_TIMER_H

typedef uint16_t time_hours;
typedef uint8_t time_mins;
typedef uint8_t time_secs;
typedef uint16_t time_millis;

typedef struct _SYSTEM_TIME {
    time_hours hours;
    time_mins mins;
    time_secs secs;
    time_millis millis;
} SYSTEM_TIME;

SYSTEM_TIME get_system_time(void);

#endif
