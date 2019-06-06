#ifndef __SC2705_HAPTIC_HELPER_H
#define __SC2705_HAPTIC_HELPER_H

typedef void (*vibrator_ctrl)(bool onoff, void *priv);

/*
 * register a callback to receive vibrator control from
 * sys/class/timed_output/vibrator/enable, which is the legacy method
 * for Android framework
 * private_data will be passed through this callback
 */
int timed_vibrator_register(vibrator_ctrl callback, void *private_data);
void timed_vibrator_unregister(vibrator_ctrl callback);

#endif /* __SC2705_HAPTIC_HELPER_H */

