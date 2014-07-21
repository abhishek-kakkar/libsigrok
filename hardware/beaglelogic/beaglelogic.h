/*
 * This file is part of the libsigrok and the BeagleLogic project.
 *
 * Copyright (C) 2014 Kumar Abhishek <abhishek@theembeddedkitchen.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BEAGLELOGIC_H_
#define BEAGLELOGIC_H_

#include <fcntl.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/ioctl.h>

#include <stdlib.h>

#include <unistd.h>

/* BeagleLogic device node name */
#define BEAGLELOGIC_DEV_NODE        "/dev/beaglelogic"
#define BEAGLELOGIC_SYSFS_ATTR(a)   "/sys/devices/virtual/misc/beaglelogic/"\
                                      __STRING(a)

/* ioctl calls that can be issued on /dev/beaglelogic */
#define IOCTL_BL_GET_VERSION        _IOR('k', 0x20, uint32_t)

#define IOCTL_BL_GET_SAMPLE_RATE    _IOR('k', 0x21, uint32_t)
#define IOCTL_BL_SET_SAMPLE_RATE    _IOW('k', 0x21, uint32_t)

#define IOCTL_BL_GET_SAMPLE_UNIT    _IOR('k', 0x22, uint32_t)
#define IOCTL_BL_SET_SAMPLE_UNIT    _IOW('k', 0x22, uint32_t)

#define IOCTL_BL_GET_TRIGGER_FLAGS  _IOR('k', 0x23, uint32_t)
#define IOCTL_BL_SET_TRIGGER_FLAGS  _IOW('k', 0x23, uint32_t)

#define IOCTL_BL_CACHE_INVALIDATE    _IO('k', 0x25)

#define IOCTL_BL_GET_BUFFER_SIZE    _IOR('k', 0x26, uint32_t)
#define IOCTL_BL_SET_BUFFER_SIZE    _IOW('k', 0x26, uint32_t)

#define IOCTL_BL_GET_BUFUNIT_SIZE   _IOR('k', 0x27, uint32_t)

#define IOCTL_BL_FILL_TEST_PATTERN   _IO('k', 0x28)

#define IOCTL_BL_START               _IO('k', 0x29)
#define IOCTL_BL_STOP                _IO('k', 0x2A)

/* Possible States of BeagleLogic */
enum beaglelogic_states {
	STATE_BL_DISABLED,	/* Powered off (at module start) */
	STATE_BL_INITIALIZED,	/* Powered on */
	STATE_BL_MEMALLOCD,	/* Buffers allocated */
	STATE_BL_ARMED,		/* All Buffers DMA-mapped and configuration done */
	STATE_BL_RUNNING,	/* Data being captured */
	STATE_BL_REQUEST_STOP,	/* Stop requested */
	STATE_BL_ERROR   	/* Buffer overrun */
};

/* Setting attributes */
enum beaglelogic_triggerflags {
	BL_TRIGGERFLAGS_ONESHOT = 0,
	BL_TRIGGERFLAGS_CONTINUOUS
};

/* Possible sample unit / formats */
enum beaglelogic_sampleunit {
	BL_SAMPLEUNIT_16_BITS = 0,
	BL_SAMPLEUNIT_8_BITS
};

SR_PRIV int beaglelogic_open(void);
SR_PRIV int beaglelogic_open_nonblock(void);
SR_PRIV int beaglelogic_close(int fd);

/* Read from the BeagleLogic file */
SR_PRIV int beaglelogic_read(int fd, void *buf, size_t bytes);

/* Gets and sets the kernel capture buffer size in bytes
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 * 	* size : pointer to var (for get) and value (for set)
 * Returns:
 * 	0 on success, -1 on failure
 */
SR_PRIV int beaglelogic_get_buffersize(int fd, uint32_t *bufsize);
SR_PRIV int beaglelogic_set_buffersize(int fd, uint32_t bufsize);

/* Gets and sets the sample rate (in Hz)
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 * 	* samplerate : pointer to var (for get) and value (for set)
 * Returns:
 * 	0 on success, -1 on failure
 */
SR_PRIV int beaglelogic_get_samplerate(int fd, uint32_t *samplerate);
SR_PRIV int beaglelogic_set_samplerate(int fd, uint32_t samplerate);

/* Gets and sets the sample unit
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 * 	* sampleunit : pointer to var (for get) and value (for set)
 * Possible values:
 * 	* BL_SAMPLEUNIT_16_BITS : 16-bit samples
 * 	* BL_SAMPLEUNIT_8_BITS : 8-bit samples
 * Returns:
 * 	0 on success, -1 on failure
 */
SR_PRIV int beaglelogic_get_sampleunit(int fd,
		enum beaglelogic_sampleunit *sampleunit);

SR_PRIV int beaglelogic_set_sampleunit(int fd,
		enum beaglelogic_sampleunit sampleunit);

/* Gets and sets the trigger flags
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 * 	* triggerflags : pointer to var (for get) and value (for set)
 * Possible values:
 * 	* BL_TRIGGERFLAGS_ONESHOT : One-shot
 * 	* BL_TRIGGERFLAGS_CONTINUOUS : Continuous
 * Returns:
 * 	0 on success, -1 on failure
 */
SR_PRIV int beaglelogic_get_triggerflags(int fd,
		enum beaglelogic_triggerflags *triggerflags);

SR_PRIV int beaglelogic_set_triggerflags(int fd,
		enum beaglelogic_triggerflags triggerflags);

/* Polls for last error and returns the error code
 *
 * This function waits till the capture session ends, so may not be
 * called from the same thread that is reading data from the device
 *
 * Returns:
 * 	0 or 0x1nnnn, where nnnn: The buffer on which buffer overrun occurred
 */
SR_PRIV int beaglelogic_getlasterror(void);

/* Starts a logic capture
 *
 * This function may be called only while operating with mmap or testing
 * as the logic analyzer is triggered automatically on the first read() call
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 *
 * Returns:
 * 	0 on success, -1 on failure
 */
SR_PRIV inline int beaglelogic_start(int fd);

/* Ends the active capture session
 * Use beaglelogic_getlasterror to busy-wait till the handle is released
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 *
 * Returns:
 * 	0 on success, non-zero indicates some error
 */
SR_PRIV inline int beaglelogic_stop(int fd) ;

/* Invalidates the kernel buffer cache
 * To be used with mmap operations only.
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 *
 * Returns:
 * 	0 on success, -1 on failure
 */
SR_PRIV inline int beaglelogic_memcacheinvalidate(int fd);

/* Gets the unit size of the capture buffer
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 *
 * Returns:
 * 	the unit size
 */
SR_PRIV int beaglelogic_get_bufunitsize(int fd);

/* Maps the kernel buffer memory for user inspection / use
 * This gets the size from the beaglelogic device node
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 *
 * Returns:
 * 	NULL if mapping failed, otherwise an address to the buffer
 */
SR_PRIV void * beaglelogic_mmap(int fd);

/* Destroys the memory mapping
 * This gets the size from the beaglelogic device node
 *
 * Parameters:
 * 	* fd : The file number to an open /dev/beaglelogic node
 *	* addr : The beginning of the mapping
 *
 * NOTE: Please destroy the previous mapping before setting the buffer
 * size with beaglelogic_set_buffersize. Otherwise strange things will
 * happen!
 *
 * Returns:
 * 	0 if successful
 */
SR_PRIV int beaglelogic_munmap(int fd, void *addr);

/* Busy-waits for the next sample buffer to be filled in
 * To be used in conjunction with mmap
 *
 * Parameters:
 * 	none. This uses sysfs attributes, hence no params are required
 *
 * Returns:
 * 	The index of the last buffer read. A negative value indicates the
 * 	state of BeagleLogic when not running (from enum beaglelogic_states)
 */
SR_PRIV int beaglelogic_waitfornextbuffer(void);

/* Sources */
SR_PRIV inline int beaglelogic_open(void) {
	return open(BEAGLELOGIC_DEV_NODE, O_RDONLY);
}

SR_PRIV inline int beaglelogic_open_nonblock(void) {
	return open(BEAGLELOGIC_DEV_NODE, O_RDONLY);
}

SR_PRIV inline int beaglelogic_close(int fd) {
	return close(fd);
}

SR_PRIV inline int beaglelogic_read(int fd, void *buf, size_t bytes) {
	return read(fd, buf, bytes);
}

SR_PRIV inline int beaglelogic_get_buffersize(int fd, uint32_t *bufsize) {
	return ioctl(fd, IOCTL_BL_GET_BUFFER_SIZE, bufsize);
}

SR_PRIV inline int beaglelogic_set_buffersize(int fd, uint32_t bufsize) {
	return ioctl(fd, IOCTL_BL_SET_BUFFER_SIZE, bufsize);
}

SR_PRIV inline int beaglelogic_get_samplerate(int fd, uint32_t *samplerate) {
	return ioctl(fd, IOCTL_BL_GET_SAMPLE_RATE, samplerate);
}

SR_PRIV inline int beaglelogic_set_samplerate(int fd, uint32_t samplerate) {
	return ioctl(fd, IOCTL_BL_SET_SAMPLE_RATE, samplerate);
}

SR_PRIV inline int beaglelogic_get_sampleunit(int fd,
		enum beaglelogic_sampleunit *sampleunit) {
	return ioctl(fd, IOCTL_BL_GET_SAMPLE_UNIT, sampleunit);
}

SR_PRIV inline int beaglelogic_set_sampleunit(int fd,
		enum beaglelogic_sampleunit sampleunit) {
	return ioctl(fd, IOCTL_BL_SET_SAMPLE_UNIT, sampleunit);
}

SR_PRIV inline int beaglelogic_get_triggerflags(int fd,
		enum beaglelogic_triggerflags *triggerflags) {
	return ioctl(fd, IOCTL_BL_GET_TRIGGER_FLAGS, triggerflags);
}

SR_PRIV inline int beaglelogic_set_triggerflags(int fd,
		enum beaglelogic_triggerflags triggerflags) {
	return ioctl(fd, IOCTL_BL_SET_TRIGGER_FLAGS, triggerflags);
}

SR_PRIV int beaglelogic_getlasterror(void) {
	int fd = open(BEAGLELOGIC_SYSFS_ATTR(lasterror), O_RDONLY);
	char buf[16];
	char *endptr;
	int lasterror, ret;

	if (!fd)
		return -1;

	if ((ret = read(fd, buf, 16)) < 0)
		return -1;

	close(fd);
	lasterror = strtoul(buf, &endptr, 10);

	return lasterror;
}

SR_PRIV inline int beaglelogic_start(int fd) {
	return ioctl(fd, IOCTL_BL_START);
}

SR_PRIV inline int beaglelogic_stop(int fd) {
	return ioctl(fd, IOCTL_BL_STOP);
}

SR_PRIV inline int beaglelogic_memcacheinvalidate(int fd) {
	return ioctl(fd, IOCTL_BL_CACHE_INVALIDATE);
}

SR_PRIV int beaglelogic_get_bufunitsize(int fd) {
	size_t sz;
	ioctl(fd, IOCTL_BL_GET_BUFUNIT_SIZE, &sz);
	return sz;
}

void * beaglelogic_mmap(int fd) {
	size_t sz;
	void *addr;
	if (beaglelogic_get_buffersize(fd, &sz))
		return MAP_FAILED;
	addr = mmap(NULL, sz, PROT_READ, MAP_SHARED, fd, 0);
	return addr;
}

SR_PRIV int beaglelogic_munmap(int fd, void *addr) {
	size_t sz;
	if (beaglelogic_get_buffersize(fd, &sz))
		return -1;
	return munmap(addr, sz);
}

SR_PRIV int beaglelogic_waitfornextbuffer(void) {
	int fd = open(BEAGLELOGIC_SYSFS_ATTR(state), O_RDONLY);
	char buf[16];
	char *endptr;
	int nextbuf, ret;

	if (!fd)
		return -1;

	if ((ret = read(fd, buf, 16)) < 0)
		return -1;

	close(fd);
	nextbuf = strtoul(buf, &endptr, 10);

	return nextbuf;
}

#endif /* BEAGLELOGIC_H_ */
