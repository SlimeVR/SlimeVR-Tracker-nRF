#ifndef SLIMENRF_SYSTEM_POWER
#define SLIMENRF_SYSTEM_POWER

void sys_interface_suspend(void);
void sys_interface_resume(void);

void sys_request_WOM(bool force, bool immediate);
void sys_request_system_off(bool immediate);
void sys_request_system_reboot(bool immediate);
void sys_request_system_silent_off(bool immediate);

bool vin_read(void);

#endif