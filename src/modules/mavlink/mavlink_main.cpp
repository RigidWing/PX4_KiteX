/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_main.cpp
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <poll.h>
#include <termios.h>
#include <time.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <dataman/dataman.h>
#include <version/version.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/mavlink_log.h>

#include "mavlink_bridge_header.h"
#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_receiver.h"
#include "mavlink_rate_limiter.h"
#include "mavlink_command_sender.h"

// Guard against MAVLink misconfiguration
#ifndef MAVLINK_CRC_EXTRA
#error MAVLINK_CRC_EXTRA has to be defined on PX4 systems
#endif

// Guard against flow control misconfiguration
#if defined (CRTSCTS) && defined (__PX4_NUTTX) && (CRTSCTS != (CRTS_IFLOW | CCTS_OFLOW))
#error The non-standard CRTSCTS define is incorrect. Fix this in the OS or replace with (CRTS_IFLOW | CCTS_OFLOW)
#endif

#define DEFAULT_REMOTE_PORT_UDP			14550 ///< GCS port per MAVLink spec
#define DEFAULT_DEVICE_NAME			"/dev/ttyS1"
#define MAX_DATA_RATE				10000000	///< max data rate in bytes/s
#define MAIN_LOOP_DELAY 			10000	///< 100 Hz @ 1000 bytes/s data rate
#define FLOW_CONTROL_DISABLE_THRESHOLD		40	///< picked so that some messages still would fit it.
//#define MAVLINK_PRINT_PACKETS

static Mavlink *_mavlink_instances = nullptr;

/**
 * mavlink app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mavlink_main(int argc, char *argv[]);

extern mavlink_system_t mavlink_system;

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	Mavlink *m = Mavlink::get_instance((unsigned)chan);

	if (m != nullptr) {
		m->send_bytes(ch, length);
#ifdef MAVLINK_PRINT_PACKETS

		for (unsigned i = 0; i < length; i++) {
			printf("%02x", (unsigned char)ch[i]);
		}

#endif
	}
}

void mavlink_start_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance((unsigned)chan);

	if (m != nullptr) {
		(void)m->begin_send();
#ifdef MAVLINK_PRINT_PACKETS
		printf("START PACKET (%u): ", (unsigned)chan);
#endif
	}
}

void mavlink_end_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance((unsigned)chan);

	if (m != nullptr) {
		(void)m->send_packet();
#ifdef MAVLINK_PRINT_PACKETS
		printf("\n");
#endif
	}
}

/*
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t *mavlink_get_channel_status(uint8_t channel)
{
	Mavlink *m = Mavlink::get_instance((unsigned)channel);

	if (m != nullptr) {
		return m->get_status();

	} else {
		return nullptr;
	}
}

/*
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t *mavlink_get_channel_buffer(uint8_t channel)
{
	Mavlink *m = Mavlink::get_instance((unsigned)channel);

	if (m != nullptr) {
		return m->get_buffer();

	} else {
		return nullptr;
	}
}

static void usage();

bool Mavlink::_boot_complete = false;
bool Mavlink::_config_link_on = false;

Mavlink::Mavlink() :
	_device_name("/dev/ttyS1"),
	_task_should_exit(false),
	next(nullptr),
	_instance_id(0),
	_transmitting_enabled(true),
	_transmitting_enabled_commanded(false),
	_mavlink_log_pub(nullptr),
	_task_running(false),
	_mavlink_buffer{},
	_mavlink_status{},
	_hil_enabled(false),
	_generate_rc(false),
	_use_hil_gps(false),
	_forward_externalsp(false),
	_is_usb_uart(false),
	_wait_to_transmit(false),
	_received_messages(false),
	_main_loop_delay(1000),
	_subscriptions(nullptr),
	_streams(nullptr),
	_mavlink_shell(nullptr),
	_mavlink_ulog(nullptr),
	_mavlink_ulog_stop_requested(false),
	_mode(MAVLINK_MODE_NORMAL),
	_channel(MAVLINK_COMM_0),
	_radio_id(0),
	_logbuffer(5, sizeof(mavlink_log_s)),
	_receive_thread{},
	_forwarding_on(false),
	_ftp_on(false),
	_uart_fd(-1),
	_baudrate(57600),
	_datarate(1000),
	_datarate_events(500),
	_rate_mult(1.0f),
	_last_hw_rate_timestamp(0),
	_mavlink_param_queue_index(0),
	mavlink_link_termination_allowed(false),
	_subscribe_to_stream(nullptr),
	_subscribe_to_stream_rate(0.0f),
	_udp_initialised(false),
	_flow_control_mode(Mavlink::FLOW_CONTROL_OFF),
	_last_write_success_time(0),
	_last_write_try_time(0),
	_mavlink_start_time(0),
	_protocol_version_switch(-1),
	_protocol_version(0),
	_bytes_tx(0),
	_bytes_txerr(0),
	_bytes_rx(0),
	_bytes_timestamp(0),
	_rate_tx(0.0f),
	_rate_txerr(0.0f),
	_rate_rx(0.0f),
#ifdef __PX4_POSIX
	_myaddr {},
	_src_addr{},
	_bcast_addr{},
	_src_addr_initialized(false),
	_broadcast_address_found(false),
	_broadcast_address_not_found_warned(false),
	_broadcast_failed_warned(false),
	_network_buf{},
	_network_buf_len(0),
#endif
	_socket_fd(-1),
	_protocol(SERIAL),
	_network_port(14556),
	_remote_port(DEFAULT_REMOTE_PORT_UDP),
	_rstatus {},
	_message_buffer {},
	_message_buffer_mutex {},
	_send_mutex {},
	_param_initialized(false),
	_broadcast_mode(Mavlink::BROADCAST_MODE_OFF),
	_param_system_id(PARAM_INVALID),
	_param_component_id(PARAM_INVALID),
	_param_radio_id(PARAM_INVALID),
	_param_system_type(PARAM_INVALID),
	_param_use_hil_gps(PARAM_INVALID),
	_param_forward_externalsp(PARAM_INVALID),
	_param_broadcast(PARAM_INVALID),
	_system_type(0),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mavlink_el")),
	_txerr_perf(perf_alloc(PC_COUNT, "mavlink_txe"))
{
	_instance_id = Mavlink::instance_count();

	/* set channel according to instance id */
	switch (_instance_id) {
	case 0:
		_channel = MAVLINK_COMM_0;
		break;

	case 1:
		_channel = MAVLINK_COMM_1;
		break;

	case 2:
		_channel = MAVLINK_COMM_2;
		break;

	case 3:
		_channel = MAVLINK_COMM_3;
		break;
#ifdef MAVLINK_COMM_4

	case 4:
		_channel = MAVLINK_COMM_4;
		break;
#endif
#ifdef MAVLINK_COMM_5

	case 5:
		_channel = MAVLINK_COMM_5;
		break;
#endif
#ifdef MAVLINK_COMM_6

	case 6:
		_channel = MAVLINK_COMM_6;
		break;
#endif

	default:
		PX4_WARN("instance ID is out of range");
		px4_task_exit(1);
		break;
	}

	_rstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_GENERIC;
}

Mavlink::~Mavlink()
{
	perf_free(_loop_perf);
	perf_free(_txerr_perf);

	if (_task_running) {
		/* task wakes up every 10ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				//TODO store main task handle in Mavlink instance to allow killing task
				//task_delete(_mavlink_task);
				break;
			}
		} while (_task_running);
	}
}

void
Mavlink::set_proto_version(unsigned version)
{
	if ((version == 1 || version == 0) &&
	    ((_protocol_version_switch == 0) || (_protocol_version_switch == 1))) {
		get_status()->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
		_protocol_version = 1;

	} else if (version == 2 &&
		   ((_protocol_version_switch == 0) || (_protocol_version_switch == 2))) {
		get_status()->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
		_protocol_version = 2;
	}
}

void
Mavlink::count_txerr()
{
	perf_count(_txerr_perf);
}

int
Mavlink::instance_count()
{
	unsigned inst_index = 0;
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		inst_index++;
	}

	return inst_index;
}

Mavlink *
Mavlink::get_instance(unsigned instance)
{
	Mavlink *inst;
	LL_FOREACH(::_mavlink_instances, inst) {
		if (instance == inst->get_instance_id()) {
			return inst;
		}
	}

	return nullptr;
}

Mavlink *
Mavlink::get_instance_for_device(const char *device_name)
{
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		if (strcmp(inst->_device_name, device_name) == 0) {
			return inst;
		}
	}

	return nullptr;
}

Mavlink *
Mavlink::get_instance_for_network_port(unsigned long port)
{
	Mavlink *inst;

	LL_FOREACH(::_mavlink_instances, inst) {
		if (inst->_network_port == port) {
			return inst;
		}
	}

	return nullptr;
}

int
Mavlink::destroy_all_instances()
{
	/* start deleting from the end */
	Mavlink *inst_to_del = nullptr;
	Mavlink *next_inst = ::_mavlink_instances;

	unsigned iterations = 0;

	PX4_INFO("waiting for instances to stop");

	while (next_inst != nullptr) {
		inst_to_del = next_inst;
		next_inst = inst_to_del->next;

		/* set flag to stop thread and wait for all threads to finish */
		inst_to_del->_task_should_exit = true;

		while (inst_to_del->_task_running) {
			printf(".");
			fflush(stdout);
			usleep(10000);
			iterations++;

			if (iterations > 1000) {
				PX4_ERR("Couldn't stop all mavlink instances.");
				return PX4_ERROR;
			}
		}

	}

	//we know all threads have exited, so it's safe to manipulate the linked list and delete objects.
	while (_mavlink_instances) {
		inst_to_del = _mavlink_instances;
		LL_DELETE(_mavlink_instances, inst_to_del);
		delete inst_to_del;
	}

	printf("\n");
	PX4_INFO("all instances stopped");
	return OK;
}

int
Mavlink::get_status_all_instances()
{
	Mavlink *inst = ::_mavlink_instances;

	unsigned iterations = 0;

	while (inst != nullptr) {

		printf("\ninstance #%u:\n", iterations);
		inst->display_status();

		/* move on */
		inst = inst->next;
		iterations++;
	}

	/* return an error if there are no instances */
	return (iterations == 0);
}

bool
Mavlink::instance_exists(const char *device_name, Mavlink *self)
{
	Mavlink *inst = ::_mavlink_instances;

	while (inst != nullptr) {

		/* don't compare with itself */
		if (inst != self && !strcmp(device_name, inst->_device_name)) {
			return true;
		}

		inst = inst->next;
	}

	return false;
}

void
Mavlink::forward_message(const mavlink_message_t *msg, Mavlink *self)
{
	Mavlink *inst;
	LL_FOREACH(_mavlink_instances, inst) {
		if (inst != self) {
			const mavlink_msg_entry_t *meta = mavlink_get_msg_entry(msg->msgid);

			// Extract target system and target component if set
			unsigned target_system_id = (meta->target_system_ofs != 0) ? ((uint8_t *)msg)[meta->target_system_ofs] : 0;
			unsigned target_component_id = (meta->target_component_ofs != 0) ? ((uint8_t *)msg)[meta->target_component_ofs] : 233;

			// Broadcast or addressing this system and not trying to talk
			// to the autopilot component -> pass on to other components
			if ((target_system_id == 0 || target_system_id == self->get_system_id())
			    && (target_component_id == 0 || target_component_id != self->get_component_id())) {

				inst->pass_message(msg);
			}
		}
	}
}


int
Mavlink::get_uart_fd(unsigned index)
{
	Mavlink *inst = get_instance(index);

	if (inst) {
		return inst->get_uart_fd();
	}

	return -1;
}

int
Mavlink::get_uart_fd()
{
	return _uart_fd;
}

int
Mavlink::get_instance_id()
{
	return _instance_id;
}

mavlink_channel_t
Mavlink::get_channel()
{
	return _channel;
}

void Mavlink::mavlink_update_system()
{
	if (!_param_initialized) {
		_param_system_id = param_find("MAV_SYS_ID");
		_param_component_id = param_find("MAV_COMP_ID");
		_param_proto_ver = param_find("MAV_PROTO_VER");
		_param_radio_id = param_find("MAV_RADIO_ID");
		_param_system_type = param_find("MAV_TYPE");
		_param_use_hil_gps = param_find("MAV_USEHILGPS");
		_param_forward_externalsp = param_find("MAV_FWDEXTSP");
		_param_broadcast = param_find("MAV_BROADCAST");
	}

	/* update system and component id */
	int32_t system_id;
	param_get(_param_system_id, &system_id);

	int32_t component_id;
	param_get(_param_component_id, &component_id);

	int32_t proto = 0;
	param_get(_param_proto_ver, &proto);

	if (_protocol_version_switch != proto) {
		_protocol_version_switch = proto;
		set_proto_version(proto);
	}

	param_get(_param_radio_id, &_radio_id);

	/* only allow system ID and component ID updates
	 * after reboot - not during operation */
	if (!_param_initialized) {
		if (system_id > 0 && system_id < 255) {
			mavlink_system.sysid = system_id;
		}

		if (component_id > 0 && component_id < 255) {
			mavlink_system.compid = component_id;
		}

		_param_initialized = true;
	}

	int32_t system_type;
	param_get(_param_system_type, &system_type);

	if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END) {
		_system_type = system_type;
	}

	int32_t use_hil_gps;
	param_get(_param_use_hil_gps, &use_hil_gps);

	_use_hil_gps = (bool)use_hil_gps;

	int32_t forward_externalsp;
	param_get(_param_forward_externalsp, &forward_externalsp);

	param_get(_param_broadcast, &_broadcast_mode);

	_forward_externalsp = (bool)forward_externalsp;
}

int Mavlink::get_system_id()
{
	return mavlink_system.sysid;
}

int Mavlink::get_component_id()
{
	return mavlink_system.compid;
}

int Mavlink::mavlink_open_uart(int baud, const char *uart_name, bool force_flow_control)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 500000: speed = B500000; break;

	case 921600: speed = B921600; break;

	case 1000000: speed = B1000000; break;

#ifdef B1500000

	case 1500000: speed = B1500000; break;
#endif

#ifdef B3000000

	case 3000000: speed = B3000000; break;
#endif

	default:
		PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n500000\n921600\n1000000\n",
			baud);
		return -EINVAL;
	}

	/* back off 1800 ms to avoid running into the USB setup timing */
	while (_mode == MAVLINK_MODE_CONFIG &&
	       hrt_absolute_time() < 1800U * 1000U) {
		usleep(50000);
	}

	/* open uart */
	_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	/* if this is a config link, stay here and wait for it to open */
	if (_uart_fd < 0 && _mode == MAVLINK_MODE_CONFIG) {

		int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		struct actuator_armed_s armed;

		/* get the system arming state and abort on arming */
		while (_uart_fd < 0) {

			/* abort if an arming topic is published and system is armed */
			bool updated = false;
			orb_check(armed_sub, &updated);

			if (updated) {
				/* the system is now providing arming status feedback.
				 * instead of timing out, we resort to abort bringing
				 * up the terminal.
				 */
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);

				if (armed.armed) {
					/* this is not an error, but we are done */
					orb_unsubscribe(armed_sub);
					return -1;
				}
			}

			usleep(100000);
			_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);
		}

		orb_unsubscribe(armed_sub);
	}

	if (_uart_fd < 0) {
		return _uart_fd;
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	_is_usb_uart = false;

	/* Initialize the uart config */
	if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
		PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
		::close(_uart_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			::close(_uart_fd);
			return -1;
		}

	} else {
		_is_usb_uart = true;
		/* USB has no baudrate, but use a magic number for 'fast' */
		_baudrate = 2000000;
		_rstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB;
	}

#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
	/* Put in raw mode */
	cfmakeraw(&uart_config);
#endif

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", uart_name);
		::close(_uart_fd);
		return -1;
	}

	/*
	 * Setup hardware flow control. If the port has no RTS pin this call will fail,
	 * which is not an issue, but requires a separate call so we can fail silently.
	*/

	/* setup output flow control */
	if (enable_flow_control(force_flow_control ? FLOW_CONTROL_ON : FLOW_CONTROL_AUTO)) {
		PX4_WARN("hardware flow control not supported");
	}

	return _uart_fd;
}

int
Mavlink::enable_flow_control(enum FLOW_CONTROL_MODE mode)
{
	// We can't do this on USB - skip
	if (_is_usb_uart) {
		_flow_control_mode = FLOW_CONTROL_OFF;
		return OK;
	}

	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (mode) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;

	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_mode = mode;
	}

	return ret;
}

int
Mavlink::set_hil_enabled(bool hil_enabled)
{
	int ret = OK;

	/* enable HIL */
	if (hil_enabled && !_hil_enabled && (_mode != MAVLINK_MODE_IRIDIUM)) {
		_hil_enabled = true;
		configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);
	}

	/* disable HIL */
	if (!hil_enabled && _hil_enabled) {
		_hil_enabled = false;
		configure_stream("HIL_ACTUATOR_CONTROLS", 0.0f);

	} else {
		ret = PX4_ERROR;
	}

	return ret;
}

unsigned
Mavlink::get_free_tx_buf()
{
	/*
	 * Check if the OS buffer is full and disable HW
	 * flow control if it continues to be full
	 */
	int buf_free = 0;

	// if we are using network sockets, return max length of one packet
	if (get_protocol() == UDP || get_protocol() == TCP) {
		return  1500;

	} else {
		// No FIONSPACE on Linux todo:use SIOCOUTQ  and queue size to emulate FIONSPACE
#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
		//Linux cp210x does not support TIOCOUTQ
		buf_free = 256;
#else
		(void) ioctl(_uart_fd, FIONSPACE, (unsigned long)&buf_free);
#endif

		if (_flow_control_mode == FLOW_CONTROL_AUTO && buf_free < FLOW_CONTROL_DISABLE_THRESHOLD) {
			/* Disable hardware flow control in FLOW_CONTROL_AUTO mode:
			 * if no successful write since a defined time
			 * and if the last try was not the last successful write
			 */
			if (_last_write_try_time != 0 &&
			    hrt_elapsed_time(&_last_write_success_time) > 500 * 1000UL &&
			    _last_write_success_time != _last_write_try_time) {
				enable_flow_control(FLOW_CONTROL_OFF);
			}
		}
	}

	return buf_free;
}

void
Mavlink::begin_send()
{
	// must protect the network buffer so other calls from receive_thread do not
	// mangle the message.
	pthread_mutex_lock(&_send_mutex);
}

int
Mavlink::send_packet()
{
	int ret = -1;

#ifdef __PX4_POSIX

	/* Only send packets if there is something in the buffer. */
	if (_network_buf_len == 0) {
		pthread_mutex_unlock(&_send_mutex);
		return 0;
	}

	if (get_protocol() == UDP) {


		ret = sendto(_socket_fd, _network_buf, _network_buf_len, 0,
			     (struct sockaddr *)&_src_addr, sizeof(_src_addr));

		struct telemetry_status_s &tstatus = get_rx_status();

		/* resend message via broadcast if no valid connection exists */
		if ((_mode != MAVLINK_MODE_ONBOARD) && broadcast_enabled() &&
		    (!get_client_source_initialized()
		     || (hrt_elapsed_time(&tstatus.heartbeat_time) > 3 * 1000 * 1000))) {

			if (!_broadcast_address_found) {
				find_broadcast_address();
			}

			if (_broadcast_address_found && _network_buf_len > 0) {

				int bret = sendto(_socket_fd, _network_buf, _network_buf_len, 0,
						  (struct sockaddr *)&_bcast_addr, sizeof(_bcast_addr));

				if (bret <= 0) {
					if (!_broadcast_failed_warned) {
						PX4_ERR("sending broadcast failed, errno: %d: %s", errno, strerror(errno));
						_broadcast_failed_warned = true;
					}

				} else {
					_broadcast_failed_warned = false;
				}
			}
		}

	} else if (get_protocol() == TCP) {
		/* not implemented, but possible to do so */
		PX4_ERR("TCP transport pending implementation");
	}

	_network_buf_len = 0;
#endif

	pthread_mutex_unlock(&_send_mutex);
	return ret;
}

void
Mavlink::send_bytes(const uint8_t *buf, unsigned packet_len)
{
	/* If the wait until transmit flag is on, only transmit after we've received messages.
	   Otherwise, transmit all the time. */
	if (!should_transmit()) {
		return;
	}

	_last_write_try_time = hrt_absolute_time();

	if (_mavlink_start_time == 0) {
		_mavlink_start_time = _last_write_try_time;
	}

	if (get_protocol() == SERIAL) {
		/* check if there is space in the buffer, let it overflow else */
		unsigned buf_free = get_free_tx_buf();

		if (buf_free < packet_len) {
			/* not enough space in buffer to send */
			count_txerr();
			count_txerrbytes(packet_len);
			return;
		}
	}

	size_t ret = -1;

	/* send message to UART */
	if (get_protocol() == SERIAL) {
		ret = ::write(_uart_fd, buf, packet_len);
	}

#ifdef __PX4_POSIX

	else {
		if (_network_buf_len + packet_len < sizeof(_network_buf) / sizeof(_network_buf[0])) {
			memcpy(&_network_buf[_network_buf_len], buf, packet_len);
			_network_buf_len += packet_len;

			ret = packet_len;
		}
	}

#endif

	if (ret != (size_t) packet_len) {
		count_txerr();
		count_txerrbytes(packet_len);

	} else {
		_last_write_success_time = _last_write_try_time;
		count_txbytes(packet_len);
	}
}

void
Mavlink::find_broadcast_address()
{
#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
	struct ifconf ifconf;
	int ret;

#if defined(__APPLE__) && defined(__MACH__)
	// On Mac, we can't determine the required buffer
	// size in advance, so we just use what tends to work.
	ifconf.ifc_len = 1024;
#else
	// On Linux, we can determine the required size of the
	// buffer first by providing NULL to ifc_req.
	ifconf.ifc_req = nullptr;
	ifconf.ifc_len = 0;

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);

	if (ret != 0) {
		PX4_WARN("getting required buffer size failed");
		return;
	}

#endif

	PX4_DEBUG("need to allocate %d bytes", ifconf.ifc_len);

	// Allocate buffer.
	ifconf.ifc_req = (struct ifreq *)(new uint8_t[ifconf.ifc_len]);

	if (ifconf.ifc_req == nullptr) {
		PX4_ERR("Could not allocate ifconf buffer");
		return;
	}

	memset(ifconf.ifc_req, 0, ifconf.ifc_len);

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);

	if (ret != 0) {
		PX4_ERR("getting network config failed");
		delete[] ifconf.ifc_req;
		return;
	}

	size_t offset = 0;
	// Later used to point to next network interface in buffer.
	struct ifreq *cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

	// The ugly `for` construct is used because it allows to use
	// `continue` and `break`.
	for (;
	     offset < ifconf.ifc_len;
#if defined(__APPLE__) && defined(__MACH__)
	     // On Mac, to get to next entry in buffer, jump by the size of
	     // the interface name size plus whatever is greater, either the
	     // sizeof sockaddr or ifr_addr.sa_len.
	     offset += IF_NAMESIZE
		       + (sizeof(struct sockaddr) > cur_ifreq->ifr_addr.sa_len ?
			  sizeof(struct sockaddr) : cur_ifreq->ifr_addr.sa_len)
#else
	     // On Linux, it's much easier to traverse the buffer, every entry
	     // has the constant length.
	     offset += sizeof(struct ifreq)
#endif
	    ) {
		// Point to next network interface in buffer.
		cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

		PX4_DEBUG("looking at %s", cur_ifreq->ifr_name);

		// ignore loopback network
		if (strcmp(cur_ifreq->ifr_name, "lo") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo0") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo1") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo2") == 0) {
			PX4_DEBUG("skipping loopback");
			continue;
		}

		struct in_addr &sin_addr = ((struct sockaddr_in *)&cur_ifreq->ifr_addr)->sin_addr;

		// Accept network interfaces to local network only. This means it's an IP starting with:
		// 192./172./10.
		// Also see https://tools.ietf.org/html/rfc1918#section-3

		uint8_t first_byte = sin_addr.s_addr & 0xFF;

		if (first_byte != 192 && first_byte != 172 && first_byte != 10) {
			continue;
		}

		if (!_broadcast_address_found) {
			const struct in_addr netmask_addr = query_netmask_addr(_socket_fd, *cur_ifreq);
			const struct in_addr broadcast_addr = compute_broadcast_addr(sin_addr, netmask_addr);

			PX4_INFO("using network interface %s, IP: %s", cur_ifreq->ifr_name, inet_ntoa(sin_addr));
			PX4_INFO("with netmask: %s", inet_ntoa(netmask_addr));
			PX4_INFO("and broadcast IP: %s", inet_ntoa(broadcast_addr));

			_bcast_addr.sin_family = AF_INET;
			_bcast_addr.sin_addr = broadcast_addr;

			_broadcast_address_found = true;

		} else {
			PX4_DEBUG("ignoring additional network interface %s, IP:  %s",
				  cur_ifreq->ifr_name, inet_ntoa(sin_addr));
		}
	}

	if (_broadcast_address_found) {
		_bcast_addr.sin_port = htons(_remote_port);

		int broadcast_opt = 1;

		if (setsockopt(_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0) {
			PX4_WARN("setting broadcast permission failed");
		}

		_broadcast_address_not_found_warned = false;

	} else {
		if (!_broadcast_address_not_found_warned) {
			PX4_WARN("no broadcasting address found");
			_broadcast_address_not_found_warned = true;
		}
	}

	delete[] ifconf.ifc_req;

#endif
}

#ifdef __PX4_POSIX
const in_addr
Mavlink::query_netmask_addr(const int socket_fd, const ifreq &ifreq)
{
	struct ifreq netmask_ifreq;
	memset(&netmask_ifreq, 0, sizeof(netmask_ifreq));
	strncpy(netmask_ifreq.ifr_name, ifreq.ifr_name, IF_NAMESIZE);
	ioctl(socket_fd, SIOCGIFNETMASK, &netmask_ifreq);

	return ((struct sockaddr_in *)&netmask_ifreq.ifr_addr)->sin_addr;
}

const in_addr
Mavlink::compute_broadcast_addr(const in_addr &host_addr, const in_addr &netmask_addr)
{
	struct in_addr broadcast_addr;
	broadcast_addr.s_addr = ~netmask_addr.s_addr | host_addr.s_addr;

	return broadcast_addr;
}
#endif

void
Mavlink::init_udp()
{
#if defined (__PX4_LINUX) || defined (__PX4_DARWIN) || defined(__PX4_CYGWIN)

	PX4_DEBUG("Setting up UDP with port %d", _network_port);

	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_network_port);

	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed: %s", strerror(errno));
		return;
	}

	if (bind(_socket_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed: %s", strerror(errno));
		return;
	}

	/* set default target address, but not for onboard mode (will be set on first received packet) */
	if (!_src_addr_initialized) {
		_src_addr.sin_family = AF_INET;
		inet_aton("127.0.0.1", &_src_addr.sin_addr);
	}

	_src_addr.sin_port = htons(_remote_port);

#endif
}

void
Mavlink::handle_message(const mavlink_message_t *msg)
{
	if (!accepting_commands()) {
		return;
	}

	/*
	 *  NOTE: this is called from the receiver thread
	 */

	if (get_forwarding_on()) {
		/* forward any messages to other mavlink instances */
		Mavlink::forward_message(msg, this);
	}
}

void
Mavlink::send_statustext_info(const char *string)
{
	mavlink_log_info(&_mavlink_log_pub, string);
}

void
Mavlink::send_statustext_critical(const char *string)
{
	mavlink_log_critical(&_mavlink_log_pub, string);
	PX4_ERR(string);
}

void
Mavlink::send_statustext_emergency(const char *string)
{
	mavlink_log_emergency(&_mavlink_log_pub, string);
}

void Mavlink::send_autopilot_capabilites()
{
	struct vehicle_status_s status;

	MavlinkOrbSubscription *status_sub = this->add_orb_subscription(ORB_ID(vehicle_status));

	if (status_sub->update(&status)) {
		mavlink_autopilot_version_t msg = {};

		msg.capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
		msg.flight_sw_version = px4_firmware_version();
		msg.middleware_sw_version = px4_firmware_version();
		msg.os_sw_version = px4_os_version();
		msg.board_version = px4_board_version();
		uint64_t fw_git_version_binary = px4_firmware_version_binary();
		memcpy(&msg.flight_custom_version, &fw_git_version_binary, sizeof(msg.flight_custom_version));
		memcpy(&msg.middleware_custom_version, &fw_git_version_binary, sizeof(msg.middleware_custom_version));
		uint64_t os_git_version_binary = px4_os_version_binary();
		memcpy(&msg.os_custom_version, &os_git_version_binary, sizeof(msg.os_custom_version));
#ifdef CONFIG_CDCACM_VENDORID
		msg.vendor_id = CONFIG_CDCACM_VENDORID;
#else
		msg.vendor_id = 0;
#endif
#ifdef CONFIG_CDCACM_PRODUCTID
		msg.product_id = CONFIG_CDCACM_PRODUCTID;
#else
		msg.product_id = 0;
#endif
		uuid_uint32_t uid;
		board_get_uuid32(uid);
		msg.uid = (((uint64_t)uid[PX4_CPU_UUID_WORD32_UNIQUE_M]) << 32) | uid[PX4_CPU_UUID_WORD32_UNIQUE_H];

		mavlink_msg_autopilot_version_send_struct(get_channel(), &msg);
	}
}

void Mavlink::send_protocol_version()
{
	mavlink_protocol_version_t msg = {};

	msg.version = _protocol_version * 100;
	msg.min_version = 100;
	msg.max_version = 200;
	uint64_t mavlink_lib_git_version_binary = px4_mavlink_lib_version_binary();
	// TODO add when available
	//memcpy(&msg.spec_version_hash, &mavlink_spec_git_version_binary, sizeof(msg.spec_version_hash));
	memcpy(&msg.library_version_hash, &mavlink_lib_git_version_binary, sizeof(msg.library_version_hash));

	// Switch to MAVLink 2
	int curr_proto_ver = _protocol_version;
	set_proto_version(2);
	// Send response - if it passes through the link its fine to use MAVLink 2
	mavlink_msg_protocol_version_send_struct(get_channel(), &msg);
	// Reset to previous value
	set_proto_version(curr_proto_ver);
}

MavlinkOrbSubscription *Mavlink::add_orb_subscription(const orb_id_t topic, int instance)
{
	/* check if already subscribed to this topic */
	MavlinkOrbSubscription *sub;

	LL_FOREACH(_subscriptions, sub) {
		if (sub->get_topic() == topic && sub->get_instance() == instance) {
			/* already subscribed */
			return sub;
		}
	}

	/* add new subscription */
	MavlinkOrbSubscription *sub_new = new MavlinkOrbSubscription(topic, instance);

	LL_APPEND(_subscriptions, sub_new);

	return sub_new;
}

int
Mavlink::interval_from_rate(float rate)
{
	if (rate > 0.000001f) {
		return (1000000.0f / rate);

	} else if (rate < 0.0f) {
		return -1;

	} else {
		return 0;
	}
}

int
Mavlink::configure_stream(const char *stream_name, const float rate)
{
	/* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = interval_from_rate(rate);

	/* search if stream exists */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				LL_DELETE(_streams, stream);
				delete stream;
			}

			return OK;
		}
	}

	if (interval == 0) {
		/* stream was not active and is requested to be disabled, do nothing */
		return OK;
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	stream = create_mavlink_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
		LL_APPEND(_streams, stream);

		return OK;
	}

	/* if we reach here, the stream list does not contain the stream */
	PX4_WARN("stream %s not found", stream_name);

	return PX4_ERROR;
}

void
Mavlink::adjust_stream_rates(const float multiplier)
{
	/* do not allow to push us to zero */
	if (multiplier < MAVLINK_MIN_MULTIPLIER) {
		return;
	}

	/* search if stream exists */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		/* set new interval */
		int interval = stream->get_interval();

		if (interval > 0) {
			interval /= multiplier;

			/* limit min / max interval */
			if (interval < MAVLINK_MIN_INTERVAL) {
				interval = MAVLINK_MIN_INTERVAL;
			}

			if (interval > MAVLINK_MAX_INTERVAL) {
				interval = MAVLINK_MAX_INTERVAL;
			}

			/* set new interval */
			stream->set_interval(interval);
		}
	}
}

void
Mavlink::configure_stream_threadsafe(const char *stream_name, const float rate)
{
	/* orb subscription must be done from the main thread,
	 * set _subscribe_to_stream and _subscribe_to_stream_rate fields
	 * which polled in mavlink main loop */
	if (!_task_should_exit) {
		/* wait for previous subscription completion */
		while (_subscribe_to_stream != nullptr) {
			usleep(MAIN_LOOP_DELAY / 2);
		}

		/* copy stream name */
		unsigned n = strlen(stream_name) + 1;
		char *s = new char[n];
		strcpy(s, stream_name);

		/* set subscription task */
		_subscribe_to_stream_rate = rate;
		_subscribe_to_stream = s;

		/* wait for subscription */
		do {
			usleep(MAIN_LOOP_DELAY / 2);
		} while (_subscribe_to_stream != nullptr);

		delete[] s;
	}
}

int
Mavlink::message_buffer_init(int size)
{

	_message_buffer.size = size;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	_message_buffer.data = (char *)malloc(_message_buffer.size);

	int ret;

	if (_message_buffer.data == nullptr) {
		ret = PX4_ERROR;
		_message_buffer.size = 0;

	} else {
		ret = OK;
	}

	return ret;
}

void
Mavlink::message_buffer_destroy()
{
	_message_buffer.size = 0;
	_message_buffer.write_ptr = 0;
	_message_buffer.read_ptr = 0;
	free(_message_buffer.data);
}

int
Mavlink::message_buffer_count()
{
	int n = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (n < 0) {
		n += _message_buffer.size;
	}

	return n;
}

int
Mavlink::message_buffer_is_empty()
{
	return _message_buffer.read_ptr == _message_buffer.write_ptr;
}


bool
Mavlink::message_buffer_write(const void *ptr, int size)
{
	// bytes available to write
	int available = _message_buffer.read_ptr - _message_buffer.write_ptr - 1;

	if (available < 0) {
		available += _message_buffer.size;
	}

	if (size > available) {
		// buffer overflow
		return false;
	}

	char *c = (char *) ptr;
	int n = _message_buffer.size - _message_buffer.write_ptr;	// bytes to end of the buffer

	if (n < size) {
		// message goes over end of the buffer
		memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), c, n);
		_message_buffer.write_ptr = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	int p = size - n;	// number of bytes to write
	memcpy(&(_message_buffer.data[_message_buffer.write_ptr]), &(c[n]), p);
	_message_buffer.write_ptr = (_message_buffer.write_ptr + p) % _message_buffer.size;
	return true;
}

int
Mavlink::message_buffer_get_ptr(void **ptr, bool *is_part)
{
	// bytes available to read
	int available = _message_buffer.write_ptr - _message_buffer.read_ptr;

	if (available == 0) {
		return 0;	// buffer is empty
	}

	int n = 0;

	if (available > 0) {
		// read pointer is before write pointer, all available bytes can be read
		n = available;
		*is_part = false;

	} else {
		// read pointer is after write pointer, read bytes from read_ptr to end of the buffer
		n = _message_buffer.size - _message_buffer.read_ptr;
		*is_part = _message_buffer.write_ptr > 0;
	}

	*ptr = &(_message_buffer.data[_message_buffer.read_ptr]);
	return n;
}

void
Mavlink::message_buffer_mark_read(int n)
{
	_message_buffer.read_ptr = (_message_buffer.read_ptr + n) % _message_buffer.size;
}

void
Mavlink::pass_message(const mavlink_message_t *msg)
{
	if (_forwarding_on) {
		/* size is 8 bytes plus variable payload */
		int size = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len;
		pthread_mutex_lock(&_message_buffer_mutex);
		message_buffer_write(msg, size);
		pthread_mutex_unlock(&_message_buffer_mutex);
	}
}

float
Mavlink::get_rate_mult()
{
	return _rate_mult;
}

MavlinkShell *
Mavlink::get_shell()
{
	if (!_mavlink_shell) {
		_mavlink_shell = new MavlinkShell();

		if (!_mavlink_shell) {
			PX4_ERR("Failed to allocate a shell");

		} else {
			int ret = _mavlink_shell->start();

			if (ret != 0) {
				PX4_ERR("Failed to start shell (%i)", ret);
				delete _mavlink_shell;
				_mavlink_shell = nullptr;
			}
		}
	}

	return _mavlink_shell;
}

void
Mavlink::close_shell()
{
	if (_mavlink_shell) {
		delete _mavlink_shell;
		_mavlink_shell = nullptr;
	}
}

void
Mavlink::update_rate_mult()
{
	float const_rate = 0.0f;
	float rate = 0.0f;

	/* scale down rates if their theoretical bandwidth is exceeding the link bandwidth */
	MavlinkStream *stream;
	LL_FOREACH(_streams, stream) {
		if (stream->const_rate()) {
			const_rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;

		} else {
			rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;
		}
	}

	float mavlink_ulog_streaming_rate_inv = 1.0f;

	if (_mavlink_ulog) {
		mavlink_ulog_streaming_rate_inv = 1.f - _mavlink_ulog->current_data_rate();
	}

	/* scale up and down as the link permits */
	float bandwidth_mult = (float)(_datarate * mavlink_ulog_streaming_rate_inv - const_rate) / rate;

	/* if we do not have flow control, limit to the set data rate */
	if (!get_flow_control_enabled()) {
		bandwidth_mult = fminf(1.0f, bandwidth_mult);
	}

	/* check if we have radio feedback */
	struct telemetry_status_s &tstatus = get_rx_status();

	bool radio_critical = false;
	bool radio_found = false;

	/* 2nd pass: Now check hardware limits */
	if (tstatus.type == telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO) {

		radio_found = true;

		if (tstatus.txbuf < RADIO_BUFFER_LOW_PERCENTAGE) {
			radio_critical = true;
		}
	}

	float hardware_mult = _rate_mult;

	/* scale down if we have a TX err rate suggesting link congestion */
	if (_rate_txerr > 0.0f && !radio_critical) {
		hardware_mult = (_rate_tx) / (_rate_tx + _rate_txerr);

	} else if (radio_found && tstatus.telem_time != _last_hw_rate_timestamp) {

		if (tstatus.txbuf < RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 20% */
			hardware_mult *= 0.80f;

		} else if (tstatus.txbuf < RADIO_BUFFER_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 2.5% */
			hardware_mult *= 0.975f;

		} else if (tstatus.txbuf > RADIO_BUFFER_HALF_PERCENTAGE) {
			/* this indicates spare bandwidth, increase by 2.5% */
			hardware_mult *= 1.025f;
			/* limit to a max multiplier of 1 */
			hardware_mult = fminf(1.0f, hardware_mult);
		}

	} else if (!radio_found) {
		/* no limitation, set hardware to 1 */
		hardware_mult = 1.0f;
	}

	_last_hw_rate_timestamp = tstatus.telem_time;

	/* pick the minimum from bandwidth mult and hardware mult as limit */
	_rate_mult = fminf(bandwidth_mult, hardware_mult);

	/* ensure the rate multiplier never drops below 5% so that something is always sent */
	_rate_mult = fmaxf(0.05f, _rate_mult);
}

int
Mavlink::task_main(int argc, char *argv[])
{
	int ch;
	_baudrate = 57600;
	_datarate = 0;
	_mode = MAVLINK_MODE_NORMAL;
	bool _force_flow_control = false;

#ifdef __PX4_NUTTX
	/* the NuttX optarg handler does not
	 * ignore argv[0] like the POSIX handler
	 * does, nor does it deal with non-flag
	 * verbs well. So we remove the application
	 * name and the verb.
	 */
	argc -= 2;
	argv += 2;
#endif

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;
	int myoptind = 1;
	const char *myoptarg = nullptr;
#ifdef __PX4_POSIX
	char *eptr;
	int temp_int_arg;
#endif

	while ((ch = px4_getopt(argc, argv, "b:r:d:u:o:m:t:fwxz", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			_baudrate = strtoul(myoptarg, nullptr, 10);

			if (_baudrate < 9600 || _baudrate > 3000000) {
				PX4_ERR("invalid baud rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'r':
			_datarate = strtoul(myoptarg, nullptr, 10);

			if (_datarate < 10 || _datarate > MAX_DATA_RATE) {
				PX4_ERR("invalid data rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'd':
			_device_name = myoptarg;
			set_protocol(SERIAL);
			break;

#ifdef __PX4_POSIX

		case 'u':
			temp_int_arg = strtoul(myoptarg, &eptr, 10);

			if (*eptr == '\0') {
				_network_port = temp_int_arg;
				set_protocol(UDP);

			} else {
				PX4_ERR("invalid data udp_port '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'o':
			temp_int_arg = strtoul(myoptarg, &eptr, 10);

			if (*eptr == '\0') {
				_remote_port = temp_int_arg;
				set_protocol(UDP);

			} else {
				PX4_ERR("invalid remote udp_port '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 't':
			_src_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;

			} else {
				PX4_ERR("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}

			break;
#else

		case 'u':
		case 'o':
		case 't':
			PX4_ERR("UDP options not supported on this platform");
			err_flag = true;
			break;
#endif

//		case 'e':
//			mavlink_link_termination_allowed = true;
//			break;

		case 'm':
			if (strcmp(myoptarg, "custom") == 0) {
				_mode = MAVLINK_MODE_CUSTOM;

			} else if (strcmp(myoptarg, "camera") == 0) {
				// left in here for compatibility
				_mode = MAVLINK_MODE_ONBOARD;

			} else if (strcmp(myoptarg, "onboard") == 0) {
				_mode = MAVLINK_MODE_ONBOARD;

			} else if (strcmp(myoptarg, "osd") == 0) {
				_mode = MAVLINK_MODE_OSD;

			} else if (strcmp(myoptarg, "magic") == 0) {
				_mode = MAVLINK_MODE_MAGIC;

			} else if (strcmp(myoptarg, "config") == 0) {
				_mode = MAVLINK_MODE_CONFIG;

			} else if (strcmp(myoptarg, "iridium") == 0) {
				_mode = MAVLINK_MODE_IRIDIUM;
				_rstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_IRIDIUM;

			} else if (strcmp(myoptarg, "minimal") == 0) {
				_mode = MAVLINK_MODE_MINIMAL;
			}

			break;

		case 'f':
			_forwarding_on = true;
			break;

		case 'w':
			_wait_to_transmit = true;
			break;

		case 'x':
			_ftp_on = true;
			break;

		case 'z':
			_force_flow_control = true;
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		usage();
		return PX4_ERROR;
	}

	if (_datarate == 0) {
		/* convert bits to bytes and use 1/2 of bandwidth by default */
		_datarate = _baudrate / 20;
	}

	if (_datarate > MAX_DATA_RATE) {
		_datarate = MAX_DATA_RATE;
	}

	if (get_protocol() == SERIAL) {
		if (Mavlink::instance_exists(_device_name, this)) {
			PX4_ERR("%s already running", _device_name);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on %s @ %dB",
			 mavlink_mode_str(_mode), _datarate, _device_name, _baudrate);

		/* flush stdout in case MAVLink is about to take it over */
		fflush(stdout);

		/* default values for arguments */
		_uart_fd = mavlink_open_uart(_baudrate, _device_name, _force_flow_control);

		if (_uart_fd < 0 && _mode != MAVLINK_MODE_CONFIG) {
			PX4_ERR("could not open %s", _device_name);
			return PX4_ERROR;

		} else if (_uart_fd < 0 && _mode == MAVLINK_MODE_CONFIG) {
			/* the config link is optional */
			return OK;
		}

	} else if (get_protocol() == UDP) {
		if (Mavlink::get_instance_for_network_port(_network_port) != nullptr) {
			PX4_ERR("port %d already occupied", _network_port);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on udp port %hu remote port %hu",
			 mavlink_mode_str(_mode), _datarate, _network_port, _remote_port);
	}

	/* initialize send mutex */
	pthread_mutex_init(&_send_mutex, nullptr);

	/* if we are passing on mavlink messages, we need to prepare a buffer for this instance */
	if (_forwarding_on) {
		/* initialize message buffer if multiplexing is on.
		 * make space for two messages plus off-by-one space as we use the empty element
		 * marker ring buffer approach.
		 */
		if (OK != message_buffer_init(2 * sizeof(mavlink_message_t) + 1)) {
			PX4_ERR("msg buf alloc fail");
			return 1;
		}

		/* initialize message buffer mutex */
		pthread_mutex_init(&_message_buffer_mutex, nullptr);
	}

	/* Initialize system properties */
	mavlink_update_system();

	MavlinkOrbSubscription *cmd_sub = add_orb_subscription(ORB_ID(vehicle_command));
	uint64_t cmd_time = 0;
	MavlinkOrbSubscription *param_sub = add_orb_subscription(ORB_ID(parameter_update));
	uint64_t param_time = 0;
	MavlinkOrbSubscription *status_sub = add_orb_subscription(ORB_ID(vehicle_status));
	uint64_t status_time = 0;
	MavlinkOrbSubscription *ack_sub = add_orb_subscription(ORB_ID(vehicle_command_ack));
	uint64_t ack_time = 0;
	/* We don't want to miss the first advertise of an ACK, so we subscribe from the
	 * beginning and not just when the topic exists. */
	ack_sub->subscribe_from_beginning(true);
	cmd_sub->subscribe_from_beginning(true);

	MavlinkOrbSubscription *mavlink_log_sub = add_orb_subscription(ORB_ID(mavlink_log));

	struct vehicle_status_s status;
	status_sub->update(&status_time, &status);

	/* Activate sending the data by default (for the IRIDIUM mode it will be disabled after the first round of packages is sent)*/
	_transmitting_enabled = true;
	_transmitting_enabled_commanded = true;

	if (_mode == MAVLINK_MODE_IRIDIUM) {
		_transmitting_enabled_commanded = false;
	}

	/* add default streams depending on mode */
	if (_mode != MAVLINK_MODE_IRIDIUM) {

		/* HEARTBEAT is constant rate stream, rate never adjusted */
		configure_stream("HEARTBEAT", 1.0f);

		/* STATUSTEXT stream is like normal stream but gets messages from logbuffer instead of uORB */
		configure_stream("STATUSTEXT", 20.0f);

		/* COMMAND_LONG stream: use unlimited rate to send all commands */
		configure_stream("COMMAND_LONG");

	}

	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("EXTENDED_SYS_STATE", 1.0f);
		configure_stream("HIGHRES_IMU", 1.5f);
		configure_stream("ATTITUDE", 2.0f); //Kitex
		configure_stream("ATTITUDE_QUATERNION", 10.0f); //Kitex
		configure_stream("RC_CHANNELS", 5.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream("ALTITUDE", 1.0f);
		configure_stream("GPS_RAW_INT", 1.0f);
		configure_stream("ADSB_VEHICLE", 0.5f); //Kitex
		configure_stream("COLLISION", 0.5f); //Kitex
		configure_stream("DISTANCE_SENSOR", 0.5f);
		// configure_stream("OPTICAL_FLOW_RAD", 1.0f);//Kitex
		// configure_stream("VISION_POSITION_ESTIMATE", 1.0f);//Kitex
		configure_stream("ESTIMATOR_STATUS", 0.5f);
		configure_stream("NAV_CONTROLLER_OUTPUT", 1.0f);//Kitex
		configure_stream("GLOBAL_POSITION_INT", 1.0f);//Kitex
		configure_stream("LOCAL_POSITION_NED", 10.0f);//Kitex
		configure_stream("POSITION_TARGET_GLOBAL_INT", 1.0f);//Kitex
		configure_stream("POSITION_TARGET_LOCAL_NED", 1.0f);//Kitex
		configure_stream("ATTITUDE_TARGET", 1.0f);//Kitex
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream("DEBUG", 1.0f);
		configure_stream("DEBUG_VECT", 1.0f);
		configure_stream("VFR_HUD", 1.0f);//Kitex
		configure_stream("WIND_COV", 1.0f);
		configure_stream("CAMERA_IMAGE_CAPTURED");
		break;

	case MAVLINK_MODE_ONBOARD:
		configure_stream("SYS_STATUS", 5.0f);
		configure_stream("EXTENDED_SYS_STATE", 5.0f);
		configure_stream("HIGHRES_IMU", 50.0f);
		configure_stream("ATTITUDE", 100.0f);
		configure_stream("ATTITUDE_QUATERNION", 50.0f);
		configure_stream("RC_CHANNELS", 20.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 10.0f);
		configure_stream("ALTITUDE", 10.0f);
		configure_stream("GPS_RAW_INT");
		configure_stream("ADSB_VEHICLE");
		configure_stream("COLLISION");
		configure_stream("DISTANCE_SENSOR", 10.0f);
		configure_stream("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream("VISION_POSITION_ESTIMATE", 10.0f);
		configure_stream("ESTIMATOR_STATUS", 1.0f);
		configure_stream("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream("GLOBAL_POSITION_INT", 50.0f);
		configure_stream("LOCAL_POSITION_NED", 30.0f);
		configure_stream("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream("ATTITUDE_TARGET", 10.0f);
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("NAMED_VALUE_FLOAT", 10.0f);
		configure_stream("DEBUG", 10.0f);
		configure_stream("DEBUG_VECT", 10.0f);
		configure_stream("VFR_HUD", 10.0f);
		configure_stream("WIND_COV", 10.0f);
		configure_stream("POSITION_TARGET_LOCAL_NED", 10.0f);
		configure_stream("SYSTEM_TIME", 1.0f);
		configure_stream("TIMESYNC", 10.0f);
		configure_stream("CAMERA_CAPTURE", 2.0f);
		configure_stream("CAMERA_TRIGGER");
		configure_stream("CAMERA_IMAGE_CAPTURED");
		configure_stream("ACTUATOR_CONTROL_TARGET0", 10.0f);
		break;

	case MAVLINK_MODE_OSD:
		configure_stream("SYS_STATUS", 5.0f);
		configure_stream("EXTENDED_SYS_STATE", 1.0f);
		configure_stream("ATTITUDE", 25.0f);
		configure_stream("RC_CHANNELS", 5.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream("ALTITUDE", 1.0f);
		configure_stream("GPS_RAW_INT", 1.0f);
		configure_stream("ESTIMATOR_STATUS", 1.0f);
		configure_stream("GLOBAL_POSITION_INT", 10.0f);
		configure_stream("ATTITUDE_TARGET", 10.0f);
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("VFR_HUD", 25.0f);
		configure_stream("WIND_COV", 2.0f);
		configure_stream("SYSTEM_TIME", 1.0f);
		break;

	case MAVLINK_MODE_MAGIC:
		//stream nothing
		break;

	case MAVLINK_MODE_CONFIG:
		// Enable a number of interesting streams we want via USB
		configure_stream("SYS_STATUS", 1.0f);
		configure_stream("EXTENDED_SYS_STATE", 2.0f);
		configure_stream("HIGHRES_IMU", 50.0f);
		configure_stream("ATTITUDE", 50.0f);
		configure_stream("ATTITUDE_QUATERNION", 50.0f);
		configure_stream("RC_CHANNELS", 10.0f);
		configure_stream("SERVO_OUTPUT_RAW_0", 20.0f);
		configure_stream("SERVO_OUTPUT_RAW_1", 20.0f);
		configure_stream("ALTITUDE", 10.0f);
		configure_stream("GPS_RAW_INT");
		configure_stream("ADSB_VEHICLE");
		configure_stream("COLLISION");
		configure_stream("DISTANCE_SENSOR", 10.0f);
		configure_stream("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream("VISION_POSITION_ESTIMATE", 10.0f);
		configure_stream("ESTIMATOR_STATUS", 5.0f);
		configure_stream("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream("GLOBAL_POSITION_INT", 10.0f);
		configure_stream("LOCAL_POSITION_NED", 30.0f);
		configure_stream("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream("SYSTEM_TIME", 1.0f);
		configure_stream("TIMESYNC", 10.0f);
		configure_stream("ATTITUDE_TARGET", 8.0f);
		configure_stream("HOME_POSITION", 0.5f);
		configure_stream("NAMED_VALUE_FLOAT", 50.0f);
		configure_stream("DEBUG", 50.0f);
		configure_stream("DEBUG_VECT", 50.0f);
		configure_stream("VFR_HUD", 20.0f);
		configure_stream("WIND_COV", 10.0f);
		configure_stream("CAMERA_TRIGGER");
		configure_stream("CAMERA_IMAGE_CAPTURED");
		configure_stream("ACTUATOR_CONTROL_TARGET0", 30.0f);
		configure_stream("MANUAL_CONTROL", 5.0f);
		break;

	case MAVLINK_MODE_IRIDIUM:
		configure_stream("HIGH_LATENCY2", 0.015f);
		break;

	case MAVLINK_MODE_MINIMAL:
		configure_stream("SYS_STATUS", 0.1f);
		configure_stream("EXTENDED_SYS_STATE", 0.1f);
		configure_stream("ATTITUDE", 10.0f);
		configure_stream("RC_CHANNELS", 0.5f);
		configure_stream("ALTITUDE", 0.5f);
		configure_stream("GPS_RAW_INT", 0.5f);
		configure_stream("GLOBAL_POSITION_INT", 5.0f);
		configure_stream("HOME_POSITION", 0.1f);
		configure_stream("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream("VFR_HUD", 1.0f);
		break;

	default:
		break;
	}

	/* set main loop delay depending on data rate to minimize CPU overhead */
	_main_loop_delay = (MAIN_LOOP_DELAY * 1000) / _datarate;

	/* hard limit to 1000 Hz at max */
	if (_main_loop_delay < MAVLINK_MIN_INTERVAL) {
		_main_loop_delay = MAVLINK_MIN_INTERVAL;
	}

	/* hard limit to 100 Hz at least */
	if (_main_loop_delay > MAVLINK_MAX_INTERVAL) {
		_main_loop_delay = MAVLINK_MAX_INTERVAL;
	}

	/* now the instance is fully initialized and we can bump the instance count */
	LL_APPEND(_mavlink_instances, this);

	/* init socket if necessary */
	if (get_protocol() == UDP) {
		init_udp();
	}

	/* if the protocol is serial, we send the system version blindly */
	if (get_protocol() == SERIAL) {
		send_autopilot_capabilites();
	}

	/* start the MAVLink receiver last to avoid a race */
	MavlinkReceiver::receive_start(&_receive_thread, this);

	while (!_task_should_exit) {
		/* main loop */
		usleep(_main_loop_delay);

		perf_begin(_loop_perf);

		hrt_abstime t = hrt_absolute_time();

		update_rate_mult();

		if (param_sub->update(&param_time, nullptr)) {
			/* parameters updated */
			mavlink_update_system();
		}

		check_radio_config();

		if (status_sub->update(&status_time, &status)) {
			/* switch HIL mode if required */
			set_hil_enabled(status.hil_state == vehicle_status_s::HIL_STATE_ON);

			set_manual_input_mode_generation(status.rc_input_mode == vehicle_status_s::RC_IN_MODE_GENERATED);
		}

		struct vehicle_command_s vehicle_cmd;

		if (cmd_sub->update(&cmd_time, &vehicle_cmd)) {
			if ((vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY) &&
			    (_mode == MAVLINK_MODE_IRIDIUM)) {
				if (vehicle_cmd.param1 > 0.5f) {
					if (!_transmitting_enabled) {
						PX4_INFO("Enable transmitting with IRIDIUM mavlink on device %s by command", _device_name);
					}

					_transmitting_enabled = true;
					_transmitting_enabled_commanded = true;

				} else {
					if (_transmitting_enabled) {
						PX4_INFO("Disable transmitting with IRIDIUM mavlink on device %s by command", _device_name);
					}

					_transmitting_enabled = false;
					_transmitting_enabled_commanded = false;
				}
			}
		}

		/* send command ACK */
		uint16_t current_command_ack = 0;
		struct vehicle_command_ack_s command_ack;

		if (ack_sub->update(&ack_time, &command_ack)) {
			if (!command_ack.from_external) {
				mavlink_command_ack_t msg;
				msg.result = command_ack.result;
				msg.command = command_ack.command;
				msg.progress = command_ack.result_param1;
				msg.result_param2 = command_ack.result_param2;
				msg.target_system = command_ack.target_system;
				msg.target_component = command_ack.target_component;
				current_command_ack = command_ack.command;

				// TODO: always transmit the acknowledge once it is only sent over the instance the command is received
				//bool _transmitting_enabled_temp = _transmitting_enabled;
				//_transmitting_enabled = true;
				mavlink_msg_command_ack_send_struct(get_channel(), &msg);
				//_transmitting_enabled = _transmitting_enabled_temp;
			}
		}

		struct mavlink_log_s mavlink_log;

		if (mavlink_log_sub->update_if_changed(&mavlink_log)) {
			_logbuffer.put(&mavlink_log);
		}

		/* check for shell output */
		if (_mavlink_shell && _mavlink_shell->available() > 0) {
			if (get_free_tx_buf() >= MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
				mavlink_serial_control_t msg;
				msg.baudrate = 0;
				msg.flags = SERIAL_CONTROL_FLAG_REPLY;
				msg.timeout = 0;
				msg.device = SERIAL_CONTROL_DEV_SHELL;
				msg.count = _mavlink_shell->read(msg.data, sizeof(msg.data));
				mavlink_msg_serial_control_send_struct(get_channel(), &msg);
			}
		}

		/* check for ulog streaming messages */
		if (_mavlink_ulog) {
			if (_mavlink_ulog_stop_requested) {
				_mavlink_ulog->stop();
				_mavlink_ulog = nullptr;
				_mavlink_ulog_stop_requested = false;

			} else {
				if (current_command_ack == vehicle_command_s::VEHICLE_CMD_LOGGING_START) {
					_mavlink_ulog->start_ack_received();
				}

				int ret = _mavlink_ulog->handle_update(get_channel());

				if (ret < 0) { //abort the streaming on error
					if (ret != -1) {
						PX4_WARN("mavlink ulog stream update failed, stopping (%i)", ret);
					}

					_mavlink_ulog->stop();
					_mavlink_ulog = nullptr;
				}
			}
		}

		/* check for requested subscriptions */
		if (_subscribe_to_stream != nullptr) {
			if (OK == configure_stream(_subscribe_to_stream, _subscribe_to_stream_rate)) {
				if (fabsf(_subscribe_to_stream_rate) > 0.00001f) {
					if (get_protocol() == SERIAL) {
						PX4_DEBUG("stream %s on device %s enabled with rate %.1f Hz", _subscribe_to_stream, _device_name,
							  (double)_subscribe_to_stream_rate);

					} else if (get_protocol() == UDP) {
						PX4_DEBUG("stream %s on UDP port %d enabled with rate %.1f Hz", _subscribe_to_stream, _network_port,
							  (double)_subscribe_to_stream_rate);
					}

				} else {
					if (get_protocol() == SERIAL) {
						PX4_INFO("stream %s on device %s disabled", _subscribe_to_stream, _device_name);

					} else if (get_protocol() == UDP) {
						PX4_INFO("stream %s on UDP port %d disabled", _subscribe_to_stream, _network_port);
					}
				}

			} else {
				if (get_protocol() == SERIAL) {
					PX4_WARN("stream %s on device %s not found", _subscribe_to_stream, _device_name);

				} else if (get_protocol() == UDP) {
					PX4_WARN("stream %s on UDP port %d not found", _subscribe_to_stream, _network_port);
				}
			}

			_subscribe_to_stream = nullptr;
		}

		/* update streams */
		MavlinkStream *stream;
		LL_FOREACH(_streams, stream) {
			stream->update(t);
		}

		if (_mode == MAVLINK_MODE_IRIDIUM) {
			if ((_last_write_success_time > 0u) && !_transmitting_enabled_commanded && _transmitting_enabled) {
				_transmitting_enabled = false;
				PX4_INFO("Disable Iridium Mavlink after first packet is sent");
			}
		}

		/* pass messages from other UARTs */
		if (_forwarding_on) {

			bool is_part;
			uint8_t *read_ptr;
			uint8_t *write_ptr;

			pthread_mutex_lock(&_message_buffer_mutex);
			int available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
			pthread_mutex_unlock(&_message_buffer_mutex);

			if (available > 0) {
				// Reconstruct message from buffer

				mavlink_message_t msg;
				write_ptr = (uint8_t *)&msg;

				// Pull a single message from the buffer
				size_t read_count = available;

				if (read_count > sizeof(mavlink_message_t)) {
					read_count = sizeof(mavlink_message_t);
				}

				memcpy(write_ptr, read_ptr, read_count);

				// We hold the mutex until after we complete the second part of the buffer. If we don't
				// we may end up breaking the empty slot overflow detection semantics when we mark the
				// possibly partial read below.
				pthread_mutex_lock(&_message_buffer_mutex);

				message_buffer_mark_read(read_count);

				/* write second part of buffer if there is some */
				if (is_part && read_count < sizeof(mavlink_message_t)) {
					write_ptr += read_count;
					available = message_buffer_get_ptr((void **)&read_ptr, &is_part);
					read_count = sizeof(mavlink_message_t) - read_count;
					memcpy(write_ptr, read_ptr, read_count);
					message_buffer_mark_read(available);
				}

				pthread_mutex_unlock(&_message_buffer_mutex);

				resend_message(&msg);
			}
		}

		/* update TX/RX rates*/
		if (t > _bytes_timestamp + 1000000) {
			if (_bytes_timestamp != 0) {
				float dt = (t - _bytes_timestamp) / 1000.0f;
				_rate_tx = _bytes_tx / dt;
				_rate_txerr = _bytes_txerr / dt;
				_rate_rx = _bytes_rx / dt;
				_bytes_tx = 0;
				_bytes_txerr = 0;
				_bytes_rx = 0;
			}

			_bytes_timestamp = t;
		}

		perf_end(_loop_perf);

		/* confirm task running only once fully initialized */
		_task_running = true;
	}

	/* first wait for threads to complete before tearing down anything */
	pthread_join(_receive_thread, nullptr);

	delete _subscribe_to_stream;
	_subscribe_to_stream = nullptr;

	/* delete streams */
	MavlinkStream *stream_to_del = nullptr;
	MavlinkStream *stream_next = _streams;

	while (stream_next != nullptr) {
		stream_to_del = stream_next;
		stream_next = stream_to_del->next;
		delete stream_to_del;
	}

	_streams = nullptr;

	/* delete subscriptions */
	MavlinkOrbSubscription *sub_to_del = nullptr;
	MavlinkOrbSubscription *sub_next = _subscriptions;

	while (sub_next != nullptr) {
		sub_to_del = sub_next;
		sub_next = sub_to_del->next;
		delete sub_to_del;
	}

	_subscriptions = nullptr;

	if (_uart_fd >= 0 && !_is_usb_uart) {
		/* close UART */
		::close(_uart_fd);
	}

	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}

	if (_forwarding_on) {
		message_buffer_destroy();
		pthread_mutex_destroy(&_message_buffer_mutex);
	}

	if (_mavlink_ulog) {
		_mavlink_ulog->stop();
		_mavlink_ulog = nullptr;
	}

	PX4_INFO("exiting channel %i", (int)_channel);

	return OK;
}

void Mavlink::check_radio_config()
{
	/* radio config check */
	if (_uart_fd >= 0 && _radio_id != 0 && _rstatus.type == telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO) {
		/* request to configure radio and radio is present */
		FILE *fs = fdopen(_uart_fd, "w");

		if (fs) {
			/* switch to AT command mode */
			usleep(1200000);
			fprintf(fs, "+++\n");
			usleep(1200000);

			if (_radio_id > 0) {
				/* set channel */
				fprintf(fs, "ATS3=%u\n", _radio_id);
				usleep(200000);

			} else {
				/* reset to factory defaults */
				fprintf(fs, "AT&F\n");
				usleep(200000);
			}

			/* write config */
			fprintf(fs, "AT&W");
			usleep(200000);

			/* reboot */
			fprintf(fs, "ATZ");
			usleep(200000);

			// XXX NuttX suffers from a bug where
			// fclose() also closes the fd, not just
			// the file stream. Since this is a one-time
			// config thing, we leave the file struct
			// allocated.
#ifndef __PX4_NUTTX
			fclose(fs);
#endif

		} else {
			PX4_WARN("open fd %d failed", _uart_fd);
		}

		/* reset param and save */
		_radio_id = 0;
		param_set_no_notification(_param_radio_id, &_radio_id);
	}
}

int Mavlink::start_helper(int argc, char *argv[])
{
	/* create the instance in task context */
	Mavlink *instance = new Mavlink();

	int res;

	if (!instance) {

		/* out of memory */
		res = -ENOMEM;
		PX4_ERR("OUT OF MEM");

	} else {
		/* this will actually only return once MAVLink exits */
		res = instance->task_main(argc, argv);
		instance->_task_running = false;

	}

	return res;
}

int
Mavlink::start(int argc, char *argv[])
{
	MavlinkULog::initialize();
	MavlinkCommandSender::initialize();

	// Wait for the instance count to go up one
	// before returning to the shell
	int ic = Mavlink::instance_count();

	if (ic == Mavlink::MAVLINK_MAX_INSTANCES) {
		PX4_ERR("Maximum MAVLink instance count of %d reached.",
			(int)Mavlink::MAVLINK_MAX_INSTANCES);
		return 1;
	}

	// Instantiate thread
	char buf[24];
	sprintf(buf, "mavlink_if%d", ic);

	// This is where the control flow splits
	// between the starting task and the spawned
	// task - start_helper() only returns
	// when the started task exits.
	px4_task_spawn_cmd(buf,
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   2650,
			   (px4_main_t)&Mavlink::start_helper,
			   (char *const *)argv);

	// Ensure that this shell command
	// does not return before the instance
	// is fully initialized. As this is also
	// the only path to create a new instance,
	// this is effectively a lock on concurrent
	// instance starting. XXX do a real lock.

	// Sleep 500 us between each attempt
	const unsigned sleeptime = 500;

	// Wait 100 ms max for the startup.
	const unsigned limit = 100 * 1000 / sleeptime;

	unsigned count = 0;

	while (ic == Mavlink::instance_count() && count < limit) {
		::usleep(sleeptime);
		count++;
	}

	return OK;
}

void
Mavlink::display_status()
{

	if (_rstatus.heartbeat_time > 0) {
		printf("\tGCS heartbeat:\t%llu us ago\n", (unsigned long long)hrt_elapsed_time(&_rstatus.heartbeat_time));
	}

	printf("\tmavlink chan: #%u\n", _channel);

	if (_rstatus.timestamp > 0) {

		printf("\ttype:\t\t");

		switch (_rstatus.type) {
		case telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO:
			printf("3DR RADIO\n");
			printf("\trssi:\t\t%d\n", _rstatus.rssi);
			printf("\tremote rssi:\t%u\n", _rstatus.remote_rssi);
			printf("\ttxbuf:\t\t%u\n", _rstatus.txbuf);
			printf("\tnoise:\t\t%d\n", _rstatus.noise);
			printf("\tremote noise:\t%u\n", _rstatus.remote_noise);
			printf("\trx errors:\t%u\n", _rstatus.rxerrors);
			printf("\tfixed:\t\t%u\n", _rstatus.fixed);
			break;

		case telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_USB:
			printf("USB CDC\n");
			break;

		default:
			printf("GENERIC LINK OR RADIO\n");
			break;
		}

	} else {
		printf("\tno telem status.\n");
	}

	printf("\tflow control:\t%s\n", (_flow_control_mode) ? "ON" : "OFF");
	printf("\trates:\n");
	printf("\ttx: %.3f kB/s\n", (double)_rate_tx);
	printf("\ttxerr: %.3f kB/s\n", (double)_rate_txerr);
	printf("\trx: %.3f kB/s\n", (double)_rate_rx);
	printf("\trate mult: %.3f\n", (double)_rate_mult);

	if (_mavlink_ulog) {
		printf("\tULog rate: %.1f%% of max %.1f%%\n", (double)_mavlink_ulog->current_data_rate() * 100.,
		       (double)_mavlink_ulog->maximum_data_rate() * 100.);
	}

	printf("\taccepting commands: %s, FTP enabled: %s\n", accepting_commands() ? "YES" : "NO", _ftp_on ? "YES" : "NO");
	printf("\ttransmitting enabled: %s\n", _transmitting_enabled ? "YES" : "NO");
	printf("\tmode: %s\n", mavlink_mode_str(_mode));
	printf("\tMAVLink version: %i\n", _protocol_version);

	printf("\ttransport protocol: ");

	switch (_protocol) {
	case UDP:
		printf("UDP (%i)\n", _network_port);
		break;

	case TCP:
		printf("TCP\n");
		break;

	case SERIAL:
		printf("serial (%s @%i)\n", _device_name, _baudrate);
		break;
	}
}

int
Mavlink::stream_command(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME;
	float rate = -1.0f;
	const char *stream_name = nullptr;
	unsigned short network_port = 0;
	char *eptr;
	int temp_int_arg;
	bool provided_device = false;
	bool provided_network_port = false;
	/*
	 * Called via main with original argv
	 *   mavlink start
	 *
	 *  Remove 2
	 */
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int i = 0;

	while (i < argc) {

		if (0 == strcmp(argv[i], "-r") && i < argc - 1) {
			rate = strtod(argv[i + 1], nullptr);

			if (rate < 0.0f) {
				err_flag = true;
			}

			i++;

		} else if (0 == strcmp(argv[i], "-d") && i < argc - 1) {
			provided_device = true;
			device_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-s") && i < argc - 1) {
			stream_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-u") && i < argc - 1) {
			provided_network_port = true;
			temp_int_arg = strtoul(argv[i + 1], &eptr, 10);

			if (*eptr == '\0') {
				network_port = temp_int_arg;

			} else {
				err_flag = true;
			}

			i++;

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag && rate >= 0.0f && stream_name != nullptr) {

		Mavlink *inst = nullptr;

		if (provided_device && !provided_network_port) {
			inst = get_instance_for_device(device_name);

		} else if (provided_network_port && !provided_device) {
			inst = get_instance_for_network_port(network_port);

		} else if (provided_device && provided_network_port) {
			PX4_WARN("please provide either a device name or a network port");
			return 1;
		}

		if (inst != nullptr) {
			inst->configure_stream_threadsafe(stream_name, rate);

		} else {

			// If the link is not running we should complain, but not fall over
			// because this is so easy to get wrong and not fatal. Warning is sufficient.
			if (provided_device) {
				PX4_WARN("mavlink for device %s is not running", device_name);

			} else {
				PX4_WARN("mavlink for network on port %hu is not running", network_port);
			}

			return 1;
		}

	} else {
		PX4_INFO("usage: mavlink stream [-d device] [-u network_port] -s stream -r rate");
		return 1;
	}

	return OK;
}

void
Mavlink::set_boot_complete()
{
	_boot_complete = true;

#ifdef __PX4_POSIX
	Mavlink *inst;
	LL_FOREACH(::_mavlink_instances, inst) {
		if ((inst->get_mode() != MAVLINK_MODE_ONBOARD) &&
		    (!inst->broadcast_enabled()) &&
		    ((inst->get_protocol() == UDP) || (inst->get_protocol() == TCP))) {
			PX4_INFO("MAVLink only on localhost (set param MAV_BROADCAST = 1 to enable network)");
		}
	}
#endif

}

static void usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module implements the MAVLink protocol, which can be used on a Serial link or UDP network connection.
It communicates with the system via uORB: some messages are directly handled in the module (eg. mission
protocol), others are published via uORB (eg. vehicle_command).

Streams are used to send periodic messages with a specific rate, such as the vehicle attitude.
When starting the mavlink instance, a mode can be specified, which defines the set of enabled streams with their rates.
For a running instance, streams can be configured via `mavlink stream` command.

There can be multiple independent instances of the module, each connected to one serial device or network port.

### Implementation
The implementation uses 2 threads, a sending and a receiving thread. The sender runs at a fixed rate and dynamically
reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the
physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

**Careful**: some of the data is accessed and modified from both threads, so when changing code or extend the
functionality, this needs to be take into account, in order to avoid race conditions and corrupt data.

### Examples
Start mavlink on ttyS1 serial with baudrate 921600 and maximum sending rate of 80kB/s:
$ mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000

Start mavlink on UDP port 14556 and enable the HIGHRES_IMU message with 50Hz:
$ mavlink start -u 14556 -r 1000000
$ mavlink stream -u 14556 -s HIGHRES_IMU -r 50
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mavlink", "communication");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start a new instance");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 10, 10000000, "Maximum sending data rate in B/s (if 0, use baudrate / 20)", true);
#ifdef __PX4_POSIX
	PRINT_MODULE_USAGE_PARAM_INT('u', 14556, 0, 65536, "Select UDP Network Port (local)", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 14550, 0, 65536, "Select UDP Network Port (remote)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('t', "127.0.0.1", nullptr,
					"Partner IP (broadcasting can be enabled via MAV_BROADCAST param)", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('m', "normal", "custom|camera|onboard|osd|magic|config|iridium|minimal",
					"Mode: sets default streams and rates", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Enable message forwarding to other Mavlink instances", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('w', "Wait to send, until first message received", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Enable FTP", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('z', "Force flow control always on", true);

	PRINT_MODULE_USAGE_ARG("on|off", "Enable/disable", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop-all", "Stop all instances");

	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status for all instances");

	PRINT_MODULE_USAGE_COMMAND_DESCR("stream", "Configure the sending rate of a stream for a running instance");
#ifdef __PX4_POSIX
	PRINT_MODULE_USAGE_PARAM_INT('u', 0, 0, 65536, "Select Mavlink instance via local Network Port", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Select Mavlink instance via Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('s', nullptr, nullptr, "Mavlink stream to configure", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', 0.f, 0.f, 2000.f, "Rate in Hz (0 = turn off)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("boot_complete",
					 "Enable sending of messages. (Must be) called as last step in startup script.");

}

int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		PX4_WARN("mavlink stop is deprecated, use stop-all instead");
		usage();
		return 1;

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

	} else if (!strcmp(argv[1], "status")) {
		return Mavlink::get_status_all_instances();

	} else if (!strcmp(argv[1], "stream")) {
		return Mavlink::stream_command(argc, argv);

	} else if (!strcmp(argv[1], "boot_complete")) {
		Mavlink::set_boot_complete();
		return 0;

	} else {
		usage();
		return 1;
	}

	return 0;
}
