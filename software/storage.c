#include "platform.h"
#include "calibration.h"
#include "storage.h"
#include <string.h>
#include <stddef.h>
#include <math.h>
#define CAL_ADDRESS 0x0801fc00U
typedef struct {
    uint32_t magic, version;
    CalParams params;
    uint32_t crc;
} CalRecord;
static CalRecord record;
static uint32_t started, word_index;
static int erase_requested, writing;

static uint32_t crc32(const void *data, unsigned size)
{
    const uint8_t *p=data;
    uint32_t crc=0xffffffffU;
    unsigned i,j;
    for(i=0;i<size;i++) {
        crc^=p[i];
        for(j=0;j<8;j++) crc=(crc>>1)^(0xedb88320U & (0U-(crc&1U)));
    }
    return ~crc;
}
int storage_load(void)
{
    CalRecord saved;
    unsigned i,j;
    memcpy(&saved,(const void *)CAL_ADDRESS,sizeof(saved));
    if(saved.magic!=0x4154434cU || saved.version!=1 ||
       saved.crc!=crc32(&saved,offsetof(CalRecord,crc)) || (saved.params.flags & ~3U)) return 0;
    for(i=0;i<3;i++) {
        if(!(fabsf(saved.params.gyro_bias[i])<=8) || !(fabsf(saved.params.accel_bias[i])<=0.35f)) return 0;
        for(j=0;j<3;j++) if(!(fabsf(saved.params.matrix[i][j])<2)) return 0;
        if(saved.params.matrix[i][i]<0.85f || saved.params.matrix[i][i]>1.15f) return 0;
    }
    cal_params=saved.params;
    return 1;
}
void storage_start(int erase_only)
{
    record.magic=0x4154434cU; record.version=1; record.params=cal_params;
    record.crc=crc32(&record,offsetof(CalRecord,crc));
    erase_requested=erase_only; writing=0; word_index=0;
    started=micros();
    flash_unlock();
    flash_flag_clear(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
    FLASH->ctrl_bit.secers=TRUE;
    FLASH->addr=CAL_ADDRESS;
    FLASH->ctrl_bit.erstr=TRUE;
}
int storage_poll(void)
{
    if((uint32_t)(micros()-started)>200000 ||
       flash_flag_get(FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG)!=RESET) {
        FLASH->ctrl_bit.secers=FALSE;
        FLASH->ctrl_bit.fprgm=FALSE;
        flash_lock(); return -1;
    }
    if(flash_flag_get(FLASH_OBF_FLAG)!=RESET) return 0;
    FLASH->ctrl_bit.secers=FALSE;
    FLASH->ctrl_bit.fprgm=FALSE;
    if(erase_requested) { flash_lock(); return 1; }
    if(writing && word_index==sizeof(record)/4) {
        flash_lock();
        return memcmp((const void *)CAL_ADDRESS,&record,sizeof(record))==0 ? 1:-1;
    }
    writing=1;
    FLASH->ctrl_bit.fprgm=TRUE;
    *(volatile uint32_t *)(CAL_ADDRESS+word_index*4)=((const uint32_t *)&record)[word_index];
    word_index++;
    return 0;
}
