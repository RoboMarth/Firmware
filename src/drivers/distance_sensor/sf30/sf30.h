#include <cstring>
#include <termios.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/distance_sensor.h>

#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_tasks.h>

#define SF30_DEFAULT_PORT 	"/dev/ttyS2"
#define BAUD_RATE 		B921600
#define OUTPUT_INTERVAL_US 	25588

class SF30 : public ModuleBase<SF30>
{
public:
	SF30(const char *port = SF30_DEFAULT_PORT);

	virtual ~SF30();

	static int task_spawn(int argc, char *argv[]);

	static SF30 *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	// returns PX4_OK if bytes are publishable, PX4_ERROR if not
	int read_most_recent_bytes();

	bool is_high_byte(uint8_t byte);

	int open_serial();

	int close_serial();

	int _fd = -1;

	char _port[20];

	orb_advert_t _distance_sensor_topic = nullptr;

	uint8_t _high_byte = 0;

	uint8_t _low_byte = 0;

	uint64_t _report_timestamp = 0;

};
