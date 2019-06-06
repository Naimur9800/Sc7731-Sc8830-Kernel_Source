#ifndef __SECURE_QOS_SETTING_H__
#define __SECURE_QOS_SETTING_H__

#define REG_READ_FN_BASE            0x8400ff00
#define REG_READ_FN(n)              (REG_READ_FN_BASE + (n))

#define SECURE_FN_REG_READ          REG_READ_FN(0xa)
#define SECURE_FN_REG_WRITE         REG_READ_FN(0xb)

ulong qos_register_read(ulong phyaddr);
ulong qos_register_write(ulong phyaddr, ulong val);

#endif
