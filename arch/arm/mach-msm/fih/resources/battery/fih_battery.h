#ifndef FIH_BATTERY_H
#define FIH_BATTERY_H

#include "../../fih_resources.h"

/* Debbie, 2011/09/15 { */
/* add battery level*/
typedef struct _VOLT_TO_PERCENT
{
    u16 dwVolt;
    u16 dwPercent;
} VOLT_TO_PERCENT;
/* Debbie, 2011/09/15 } */

enum  {
    BATTERY_TYPE_4V2,
    BATTERY_TYPE_4V35,
};


int __init battery_init(void);

#endif /* FIH_BATTERY_H */