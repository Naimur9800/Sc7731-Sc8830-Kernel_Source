/*
 * Copyright (C) Peel Technologies Inc
 */

#ifndef PHOTON_H
#define PHOTON_H

#include <linux/types.h>

/*
* struct spi_ioc_transfer - describes a single SPI transfer
* @tx_buf: Holds pointer to userspace buffer with transmit data, or null.
* If no data is provided, zeroes are shifted out.
* @rx_buf: Holds pointer to userspace buffer for receive data, or null.
* @len: Length of tx and rx buffers, in bytes.
* @speed_hz: Temporary override of the device's bitrate.
* @bits_per_word: Temporary override of the device's wordsize.
* @delay_usecs: If nonzero, how long to delay after the last bit transfer
* before optionally deselecting the device before the next transfer.
* @cs_change: True to deselect device before starting the next transfer.
*
* This structure is mapped directly to the kernel spi_transfer structure;
* the fields have the same meanings, except of course that the pointers
* are in a different address space (and may be of different sizes in some
* cases, such as 32-bit i386 userspace over a 64-bit x86_64 kernel).
*/
struct spi_ioc_transfer {
	__u64	tx_buf;
	__u64	rx_buf;
	__u32	len;
	__u32	speed_hz;
	__u16	delay_usecs;
	__u8	bits_per_word;
	__u8	cs_change;

/*
* If the contents of 'struct spi_ioc_transfer' ever change
* incompatibly, then the ioctl number (currently 0) must change;
* ioctls with constant size fields get a bit more in the way of
* error checking than ones (like this) where that field varies.
*
* NOTE: struct layout is the same in 64bit and 32bit userspace.
*/
};

struct strIds {
	__u32	u32ID1;
	__u32	u32ID2;
	__u32	u32ID3;
};

/* IOCTL commands */
#define SPI_IOC_MAGIC			'k'

/* Read/Write Message */
#define SPI_IOC_WR_MSG			_IOW(SPI_IOC_MAGIC, 2, __u8)
#define SPI_IOC_RD_MSG			_IOR(SPI_IOC_MAGIC, 3, __u8)
#define SPI_IOC_RD_IDS			_IOR(SPI_IOC_MAGIC, 4, __u8)

#endif /* PHOTON_H */
