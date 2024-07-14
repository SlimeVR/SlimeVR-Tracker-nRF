#ifndef SLIMENRF_ESB
#define SLIMENRF_ESB

#include "globals.h"

void event_handler(struct esb_evt const *event);
int clocks_start(void);
int esb_initialize(void);
int esb_initialize_rx(void);

void esb_set_addr_discovery(void);
void esb_set_addr_paired(void);

void esb_pair(void);
void esb_reset_pair(void);

#endif