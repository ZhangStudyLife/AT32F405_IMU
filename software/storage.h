#ifndef STORAGE_H
#define STORAGE_H
int storage_load(void);
void storage_start(int erase_only);
int storage_poll(void); /* 0=pending, 1=verified, -1=error/timeout */
#endif
