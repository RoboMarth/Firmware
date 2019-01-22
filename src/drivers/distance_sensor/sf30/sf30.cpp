#include "sf30.h"

extern "C" __EXPORT int sf30_main(int argc, char *argv[]);

SF30::SF30(const char *port)
{
        strncpy(_port, port, sizeof(_port));
        _port[sizeof(_port) - 1] = '\0';
}

SF30::~SF30()
{
        orb_unadvertise(_distance_sensor_topic);
}

int SF30::task_spawn(int argc, char *argv[])
{
        px4_main_t entry_point = (px4_main_t)&run_trampoline;
        int stack_size = 1256;
 
        int task_id = px4_task_spawn_cmd("sf30", SCHED_DEFAULT,
                                         SCHED_PRIORITY_FAST_DRIVER, stack_size,
                                         entry_point, (char *const *)argv);
        
        if (task_id < 0) {
                task_id = -1;
                return -errno;   
        }       
        
        _task_id = task_id;
                             
        return PX4_OK; 	
}

SF30 *SF30::instantiate(int argc, char *argv[])
{
        SF30 *sf30 = new SF30(SF30_DEFAULT_PORT);
        return sf30;
}

int SF30::custom_command(int argc, char *argv[])
{
        return print_usage("Unrecognized command.");
}

int SF30::print_usage(const char *reason)
{
	return PX4_OK;
}

void SF30::run()
{
	open_serial();

        struct distance_sensor_s report = {};
        _distance_sensor_topic = orb_advertise(ORB_ID(distance_sensor), &report);
        
        if (_distance_sensor_topic == nullptr) {
                PX4_WARN("Advertise failed.");
		close_serial();
                return;     
        }

	tcflush(_fd, TCIFLUSH);

	while (!should_exit()) {
		
		if (read_most_recent_bytes() == PX4_OK) {

			float distance = (_high_byte & 0x7F) + (_low_byte / 100.0f);

			report.timestamp = _report_timestamp;
			report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
			report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
			report.current_distance = distance;
			report.min_distance = 0.2f;
			report.max_distance = 100.0f;

			orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
		}
	}

	PX4_INFO("Exiting.");
	close_serial();
}

int SF30::read_most_recent_bytes()
{
	uint8_t bytes_buffer[2];
	int ret = ::read(_fd, bytes_buffer, sizeof(bytes_buffer));
	int err = errno;

	_report_timestamp = hrt_absolute_time(); // most accurate place to get timestamp?

	uint8_t temp_buffer[2];
	int ret2 = ::read(_fd, temp_buffer, sizeof(temp_buffer));
	// int err2 = errno;

	// if the bytes read are the last two bytes available
	if (ret == 2 && ret2 == -1) { 
		
		// sanity check: if the bytes are correctly formatted
		if (is_high_byte(bytes_buffer[0]) && !is_high_byte(bytes_buffer[1])) {
			_high_byte = bytes_buffer[0];
			_low_byte = bytes_buffer[1];
			return PX4_OK;

		} else {
			tcflush(_fd, TCIFLUSH);
			PX4_ERR("ret: %d, ret2: %d", ret, ret2);
			PX4_ERR("sanity fail, buffer reset");
			return PX4_ERROR;
		}
		
	} else { // something's wrong

		// error in ::read system call
		if (ret < 0 && err != 11) {
			tcflush(_fd, TCIFLUSH);
			PX4_ERR("read err3: %d, errno: %d, buffer reset", ret, err);

		} //else if (ret >= 0) {
			// don't do anything here, otherwise won't ever catch up
			// tcflush(_fd, TCIFLUSH);
			// PX4_ERR("falling behind, buffer reset");
		//}

		// otherwise just no data (ret == -1 && err == 11), so don't publish
		return PX4_ERROR;
	}
}

bool SF30::is_high_byte(uint8_t byte)
{
	return (0b10000000 & byte) ==
	        0b10000000;
}

int SF30::open_serial()
{
        _fd = px4_open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (_fd < 0) {
                PX4_WARN("Failed to open serial port");
                return PX4_ERROR;
        }
        
        struct termios uart_config;
        
        int termios_state;
        
        // Fill the struct for the new configuration.
        tcgetattr(_fd, &uart_config);
        
        // Input flags - Turn off input processing:
        // convert break to null byte, no CR to NL translation,
        // no NL to CR translation, don't mark parity errors or breaks
        // no input parity check, don't strip high bit off,
        // no XON/XOFF software flow control
        //
        uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |  INLCR | IGNCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF);
        
        uart_config.c_iflag |= IGNPAR;
        
        // Output flags - Turn off output processing:
        // no CR to NL translation, no NL to CR-NL translation,
        // no NL to CR translation, no column 0 CR suppression,
        // no Ctrl-D suppression, no fill characters, no case mapping,
        // no local output processing
        uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
        
        // No line processing:
        // echo off, echo newline off, canonical mode off,
        // extended input processing off, signal chars off
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
        
        // No parity, one stop bit, disable flow control.
        uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        
        uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);
        
        uart_config.c_cc[VMIN] = 1;
        
        uart_config.c_cc[VTIME] = 0;
        
        unsigned speed = BAUD_RATE;
        
        // Set the baud rate.
        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
                PX4_WARN("ERR CFG: %d ISPD", termios_state);
                return PX4_ERROR;
        }
        
        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
                PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
                return PX4_ERROR;
        }
        
        if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
                PX4_WARN("ERR baud %d ATTR", termios_state);
                return PX4_ERROR;
        }
        
        return _fd;
}

int SF30::close_serial()
{
        int ret = px4_close(_fd);
          
        if (ret != 0) {
                PX4_WARN("Could not close serial port");
        }

        return ret;
}
        
int sf30_main(int argc, char *argv[])
{
	return SF30::main(argc, argv);
}
