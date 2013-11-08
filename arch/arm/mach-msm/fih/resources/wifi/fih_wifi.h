#ifndef FIH_WIFI_H
#define FIH_WIFI_H

#include "../../fih_resources.h"

int __init wifi_init(void);
void *fih_wifi_mem_prealloc(int section, unsigned long size);
#endif /* FIH_WIFI_H */ 