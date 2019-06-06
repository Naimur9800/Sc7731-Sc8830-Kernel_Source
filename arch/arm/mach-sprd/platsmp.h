#ifndef __SPRD_PLATSMP_H
#define __SPRD_PLATSMP_H

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
extern volatile int pen_release;

/*
 * sci smp specific entry point for secondary CPUs
 */
extern void sci_secondary_startup(void);

#endif
