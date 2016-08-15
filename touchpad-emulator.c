/*
 * Copyright © 2016 Red Hat, Inc.
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include "config.h"

#include <errno.h>
#include <evemu.h>
#include <fcntl.h>
#include <libevdev/libevdev.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define streq(s1, s2) (strcmp((s1), (s2)) == 0)
#define strneq(s1, s2, n) (strncmp((s1), (s2), (n)) == 0)
#define _cleanup_(_x) __attribute__((cleanup(_x)))

/*
 * DEFINE_TRIVIAL_CLEANUP_FUNC() - define helper suitable for _cleanup_()
 * @_type: type of object to cleanup
 * @_func: function to call on cleanup
 */
#define DEFINE_TRIVIAL_CLEANUP_FUNC(_type, _func)               \
        static inline void _func##p(_type *_p)                  \
        {                                                       \
                if (*_p)                                        \
                        _func(*_p);                             \
        }                                                       \
        struct __useless_struct_to_allow_trailing_semicolon__

static inline void freep(void *p) { free( *(void**)p); }
static inline void fclosep(FILE **p) { if (*p) fclose(*p); }
static inline void closep(int *p) { close(*p); }
static inline void evemu_free(struct evemu_device **dev)
{
	if (*dev)
		evemu_destroy(*dev);
}
static inline void evdev_free(struct libevdev **dev)
{
	if (*dev)
		libevdev_free(*dev);
}

static void
usage(void) {
	printf("%s dest <somefile.evemu> source /dev/input/event4\n",
	       program_invocation_short_name);
        printf("\n");
        printf("Uses the evemu recording to set up a uinput device and\n"
               "converts all input data from the source into corresponding\n"
               "locations on the uinput device.\n"
               "\n"
               "For accurate representation, the source needs to have at\n"
               "least twice the resolution of the destination device.\n"
               "\n"
               "Currently only x/y are mapped, everything else is passed\n"
               "on as-is.\n");
}

static void
pass_event(struct evemu_device *dest,
	   int destfd,
	   struct input_event *event)
{
	evemu_play_one(destfd, event);
}

static void
map_event(struct libevdev *source,
	  struct evemu_device *dest,
	  struct input_event *event)
{
	const struct input_absinfo *src;
	struct input_absinfo dst;
	double val;

	src = libevdev_get_abs_info(source, event->code);
	dst.minimum = evemu_get_abs_minimum(dest, event->code);
	dst.maximum = evemu_get_abs_maximum(dest, event->code);
	dst.resolution = evemu_get_abs_resolution(dest, event->code);

	/* convert to mm off the center of the touchpad */
	val = (event->value - src->minimum - (src->maximum - src->minimum)/2.0)/src->resolution;
	/* calculate back into target coordinates */
	val = val * dst.resolution + (dst.maximum - dst.minimum)/2.0 + dst.minimum;

	printf("%x: mapping %d to %d\n", event->code, event->value, (int)val);

	event->value = max(dst.minimum, min(dst.maximum, (int)val));
}

static void
handle_event(struct libevdev *source,
	     struct evemu_device *dest,
	     int destfd,
	     struct input_event *event)
{
	if (event->type != EV_ABS) {
		pass_event(dest, destfd, event);
		return;
	}

	switch (event->code) {
	case ABS_X:
	case ABS_Y:
	case ABS_MT_POSITION_X:
	case ABS_MT_POSITION_Y:
		map_event(source, dest, event);
		break;
	default:
		break;
	}

	pass_event(dest, destfd, event);
}

static int
mainloop(struct libevdev *source, struct evemu_device *dest, int destfd)
{
	struct pollfd pollfd[2];
	sigset_t sigmask;

	pollfd[0].fd = libevdev_get_fd(source);
	pollfd[0].events = POLLIN;

	sigemptyset(&sigmask);
	sigaddset(&sigmask, SIGINT);
	pollfd[1].fd = signalfd(-1, &sigmask, SFD_NONBLOCK);
	pollfd[1].events = POLLIN;

	sigprocmask(SIG_BLOCK, &sigmask, NULL);

	while (poll(pollfd, 2, -1)) {
		struct input_event ev;
		int rc;

		if (pollfd[1].revents)
			break;

		do {
			rc = libevdev_next_event(source, LIBEVDEV_READ_FLAG_NORMAL, &ev);
			if (rc == LIBEVDEV_READ_STATUS_SYNC) {
				fprintf(stderr, "error: cannot keep up\n");
				return 1;
			} else if (rc != -EAGAIN && rc < 0) {
				fprintf(stderr, "error: %s\n", strerror(-rc));
				return 1;
			} else if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
				handle_event(source, dest, destfd, &ev);
			}

		} while (rc != -EAGAIN);
	}

	return 0;
}

int
main(int argc, char **argv)
{
	_cleanup_(fclosep) FILE *dest = NULL;
	_cleanup_(closep) int sourcefd = -1;
	_cleanup_(evemu_free) struct evemu_device *dev = NULL;
	_cleanup_(evdev_free) struct libevdev *source = NULL;
	_cleanup_(closep) int destfd = -1;
	const char *new_devnode;
	int rc;
	int srcres, dstres;

	if (argc != 5 ||
	    !streq(argv[1], "dest") ||
	    !streq(argv[3], "source")) {
		usage();
		return 1;
	}

	dest = fopen(argv[2], "r");
	if (!dest) {
		perror("Failed to open source file");
		return 1;
	}

	sourcefd = open(argv[4], O_RDONLY|O_NONBLOCK);
	if (sourcefd < 0) {
		perror("Failed to open dest device");
		return 1;
	}

	rc = libevdev_new_from_fd(sourcefd, &source);
	if (rc < 0) {
		errno = -rc;
		perror("Failed to init libevdev");
		return 1;
	}

	dev = evemu_new(NULL);
	rc = evemu_read(dev, dest);
	if (rc < 0) {
		errno = -rc;
		perror("Failed to read evemu file");
		return 1;
	}

	rc = evemu_create_managed(dev);
	if (rc < 0) {
		errno = -rc;
		perror("Failed to create evemu device");
		return 1;
	}

	new_devnode = evemu_get_devnode(dev);
	destfd = open(new_devnode, O_RDWR|O_NONBLOCK);
	if (destfd < 0) {
		perror("Failed to create target fd");
		return 1;
	}


	printf("Mapping %s to %s\n", argv[4], new_devnode);
	srcres = libevdev_get_abs_resolution(source, ABS_X);
	dstres = evemu_get_abs_resolution(dev, ABS_X);
	if (srcres < dstres * 2) {
		printf("Warning: Nyquist not met on X, have %d for emulation of %d\n",
		       srcres, dstres);
	}
	srcres = libevdev_get_abs_resolution(source, ABS_Y);
	dstres = evemu_get_abs_resolution(dev, ABS_Y);
	if (srcres < dstres * 2) {
		printf("Warning: Nyquist not met on Y, have %d for emulation of %d\n",
		       srcres, dstres);
	}

	/* disable the real touchpad, it now pipes through our virtual
	   device */
	libevdev_grab(source, LIBEVDEV_GRAB);
	mainloop(source, dev, destfd);
	libevdev_grab(source, LIBEVDEV_UNGRAB);

	return 0;
}
