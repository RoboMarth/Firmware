/****************************************************************************
 *
 *   Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sf30.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Michael Zhan
 *
 * Driver for the Lightware SF30/x laser rangefinder series
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

/* Configuration Constants */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define SF30_TAKE_RANGE_REG		'd'

// designated SERIAL4/5 on Pixhawk
#define SF30_DEFAULT_PORT		"/dev/ttyS6"

class SF30 : public cdev::CDev
{
public:
	SF30(const char *port = SF30_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~SF30();

	virtual int 			init();

	virtual ssize_t			read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

private:
	char 				_port[20];
	uint8_t _rotation;
	float				_min_distance;
	float				_max_distance;
	int         		        _conversion_interval;
	work_s				_work{};
	ringbuffer::RingBuffer		*_reports;
	int				_measure_ticks;
	int				_fd;
	char				_linebuf[10];
	unsigned			_linebuf_index;
	hrt_abstime			_last_read;

	int				_class_instance;
	int				_orb_class_instance;

	orb_advert_t			_distance_sensor_topic;

	unsigned			_consecutive_fail_count;

	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults SF30_MIN_DISTANCE
	* and SF30_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int				collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sf30_main(int argc, char *argv[]);

SF30::SF30(const char *port, uint8_t rotation) :
	CDev(RANGE_FINDER0_DEVICE_PATH),
	_rotation(rotation),
	_min_distance(0.30f),
	_max_distance(40.0f),
	_conversion_interval(83334),
	_reports(nullptr),
	_measure_ticks(0),
	_fd(-1),
	_linebuf_index(0),
	_last_read(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_consecutive_fail_count(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "sf30_read")),
	_comms_errors(perf_alloc(PC_COUNT, "sf30_com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

}

SF30::~SF30()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
SF30::init()
{
	int hw_model;
	param_get(param_find("SENS_EN_SF30"), &hw_model);

	_min_distance = 0.01f;
	_max_distance = 100.0f;
	_conversion_interval = 25588;

//	switch (hw_model) {
//
//	case 1: /* SF02 (40m, 12 Hz)*/
//		_min_distance = 0.3f;
//		_max_distance = 40.0f;
//		_conversion_interval =	83334;
//		break;
//
//	case 2:  /* SF10/a (25m 32Hz) */
//		_min_distance = 0.01f;
//		_max_distance = 25.0f;
//		_conversion_interval = 31250;
//		break;
//
//	case 3:  /* SF10/b (50m 32Hz) */
//		_min_distance = 0.01f;
//		_max_distance = 50.0f;
//		_conversion_interval = 31250;
//		break;
//
//	case 4:  /* SF10/c (100m 16Hz) */
//		_min_distance = 0.01f;
//		_max_distance = 100.0f;
//		_conversion_interval = 62500;
//		break;
//
//	case 5:
//		/* SF11/c (120m 20Hz) */
//		_min_distance = 0.01f;
//		_max_distance = 120.0f;
//		_conversion_interval = 50000;
//		break;
//
//	default:
//		PX4_ERR("invalid HW model %d.", hw_model);
//		return -1;
//	}

	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) { break; }

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

		if (_reports == nullptr) {
			PX4_ERR("alloc failed");
			ret = -1;
			break;
		}

		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		/* get a publish handle on the range finder topic */
		struct distance_sensor_s ds_report = {};

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			PX4_ERR("failed to create distance_sensor object");
		}

	} while (0);

	return ret;
}

void
SF30::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
SF30::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
SF30::get_minimum_distance()
{
	return _min_distance;
}

float
SF30::get_maximum_distance()
{
	return _max_distance;
}

int
SF30::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {

					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
SF30::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}


	/*
	 * While there is space in the caller's buffer, and reports, copy them.
	 * Note that we may be pre-empted by the workq thread while we are doing this;
	 * we are careful to avoid racing with them.
	 */
	while (count--) {
		if (_reports->get(rbuf)) {
			ret += sizeof(*rbuf);
			rbuf++;
		}
	}

	/* if there was no data, warn the caller */
	return ret ? ret : -EAGAIN;
}

int
SF30::collect()
{
	int	ret;

	perf_begin(_sample_perf);

//	/* clear buffer if last read was too long ago */
//	int64_t read_elapsed = hrt_elapsed_time(&_last_read);
//
//	/* the buffer for read chars is buflen minus null termination */
//	char readbuf[sizeof(_linebuf)];
//	unsigned readlen = sizeof(readbuf) - 1;
//
//	/* read from the sensor (uart buffer) */
//	ret = ::read(_fd, &readbuf[0], readlen);
//
//	if (ret < 0) {
//		PX4_DEBUG("read err: %d", ret);
//		perf_count(_comms_errors);
//		perf_end(_sample_perf);
//
//		/* only throw an error if we time out */
//		if (read_elapsed > (_conversion_interval * 2)) {
//			return ret;
//
//		} else {
//			return -EAGAIN;
//		}
//
//	} else if (ret == 0) {
//		return -EAGAIN;
//	}
//
//	_last_read = hrt_absolute_time();
//
//	float distance_m = -1.0f;
//	bool valid = false;
//
//	for (int i = 0; i < ret; i++) {
//		if (OK == sf30_parser(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &distance_m)) {
//			valid = true;
//		}
//	}
//
//	if (!valid) {
//		return -EAGAIN;
//	

	uint8_t readbuf[2];
	ret = ::read(_fd, &readbuf[0], 2);
	::tcflush(_fd, TCIFLUSH);

	if (ret <= 1) {		
		PX4_ERR("read err: %d", ret);
		return ret;
	}

	bool valid = true;

	float distance_m = (readbuf[0] - 128.0f + readbuf[1] / 100.0f);

	if (distance_m <= 0) {
		PX4_ERR("read error, possible byte misalignment");
		return ret;
	}

	PX4_DEBUG("val (float): %8.4f, raw: %s, valid: %s", (double)distance_m, _linebuf, ((valid) ? "OK" : "NO"));

	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = _rotation;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	report.signal_quality = -1;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it */
	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
SF30::start()
{
	/* reset the state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&SF30::cycle_trampoline, this, 1);

}

void
SF30::stop()
{
	work_cancel(HPWORK, &_work);
}

void
SF30::cycle_trampoline(void *arg)
{
	SF30 *dev = static_cast<SF30 *>(arg);

	dev->cycle();
}

void
SF30::cycle()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

		unsigned speed = B921600;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}
		
	}

	/* perform collection */
	int collect_ret = collect();

//	if (collect_ret == -EAGAIN) {
//		/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
//		work_queue(HPWORK,
//			   &_work,
//			   (worker_t)&SF30::cycle_trampoline,
//			   this,
//			   USEC2TICK(1042 * 8));
//		return;
//	}

	if (OK != collect_ret) {

		/* we know the sensor needs about four seconds to initialize */
		if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
			PX4_ERR("collection error #%u", _consecutive_fail_count);
		}

		_consecutive_fail_count++;

		/* restart the measurement state machine */
		start();
		return;

	} else {
		/* apparently success */
		_consecutive_fail_count = 0;
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&SF30::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));
}

void
SF30::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %d ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace sf30
{

SF30	*g_dev;

int	start(const char *port, uint8_t rotation);
int	stop();
int	test();
int	reset();
int	info();

/**
 * Start the driver.
 */
int
start(const char *port, uint8_t rotation)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	/* create the driver */
	g_dev = new SF30(port, rotation);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(RANGE_FINDER0_DEVICE_PATH, 0);

	if (fd < 0) {
		PX4_ERR("device open fail (%i)", errno);
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	return 0;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return -1;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		return -1;
	}

	return 0;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	struct distance_sensor_s report;
	ssize_t sz;

	int fd = open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'sf30 start' if the driver is not running", RANGE_FINDER0_DEVICE_PATH);
		return -1;
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return -1;
	}

	print_message(report);

	/* start the sensor polling at 2 Hz rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return -1;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		int ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			PX4_ERR("timed out");
			break;
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("read failed: got %d vs exp. %d", sz, sizeof(report));
			break;
		}

		print_message(report);
	}

	/* reset the sensor polling to the default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("ioctl SENSORIOCSPOLLRATE failed");
		return -1;
	}

	return 0;
}

/**
 * Reset the driver.
 */
int
reset()
{
	int fd = open(RANGE_FINDER0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return -1;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return -1;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return -1;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

} // namespace

int
sf30_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = SF30_DEFAULT_PORT;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return -1;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		return sf30::start(device_path, rotation);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return sf30::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return sf30::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return sf30::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return sf30::info();
	}

out_error:
	PX4_ERR("unrecognized command, try 'start', 'test', 'reset' or 'info'");
	return -1;
}
