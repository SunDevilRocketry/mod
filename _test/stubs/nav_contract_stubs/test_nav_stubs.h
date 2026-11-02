#ifndef TEST_SENSOR_STUBS_H
#define TEST_SENSOR_STUBS_H

#include <stdbool.h>

#include "nav.h"

extern NAV_STATUS stub__state_reserve;
extern NAV_STATUS stub__state_release;
extern int stub__pof_transitions_logged;

#endif
