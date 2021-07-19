#include <linux/delay.h>
#include <linux/slab.h>

#include "ssp_cmd_define.h"
#include "ssp_comm.h"
#include "ssp_debug.h"
#include "ssp_sysfs.h"
#include "ssp_type_define.h"
#include "ssp_system_checker.h"

#ifdef CONFIG_SSP_ENG_DEBUG
#define SSC_DEFAULT_DELAY (500)
#define SSC_DEFAULT_COUNT (50)

struct sensor_rate {
	int32_t sampling_ms;
	int32_t report_ms;
};

struct system_env_backup {
	uint64_t pre_sensor_state;
	struct sensor_rate pre_rate[LEGACY_SENSOR_MAX];
};

struct sensor_test_factor {
	int32_t comm_test_response_cnt;
	int32_t event_test_response_cnt;

	uint64_t expected_delay_ns;
	uint64_t pre_event_timestamp;
	uint64_t standard_deviation;
	uint64_t min_diff;
	uint64_t max_diff;
};

struct system_check_data {
	struct ssp_data *data;
	bool is_system_checking;
	struct mutex system_test_mutex;
	struct system_env_backup env_backup;
	struct sensor_test_factor sensor[LEGACY_SENSOR_MAX];
};

static struct system_check_data ssc;
static uint32_t ssc_delay_us;
static uint32_t ssc_count;

static inline void start_system_checker(void)
{
	mutex_lock(&ssc.system_test_mutex);
	ssc.is_system_checking = true;
	mutex_unlock(&ssc.system_test_mutex);
}

static inline void stop_system_checker(void)
{
	mutex_lock(&ssc.system_test_mutex);
	ssc.is_system_checking = false;
	mutex_unlock(&ssc.system_test_mutex);
}

static void flush_all_sensors(struct ssp_data *data)
{
	int32_t type = 0;

	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(data->sensor_probe_state & (1ULL << type)))
			continue;
		ssp_send_command(data, CMD_GETVALUE, type, SENSOR_FLUSH, 0, NULL, 0, NULL, NULL);
		usleep_range(ssc_delay_us, ssc_delay_us * 2);
	}
}

static void run_comm_test(struct ssp_data *data)
{
	int32_t i;

	ssp_infof("[SSC] run comm test...");
	start_system_checker();

	for (i = 0; i < ssc_count; i++)
		flush_all_sensors(data);

	msleep(100);
	stop_system_checker();
}

static void set_sensors_sampling_rate(void)
{
	int32_t type = 0, rate  = 0;

	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(ssc.data->sensor_probe_state & (1ULL << type)))
			continue;

		rate = (int32_t)(ssc.sensor[type].expected_delay_ns / 1000000);
		ssc.data->delay[type].sampling_period = ssc.data->delay[type].max_report_latency = rate;
	}
}

static void run_event_test(struct ssp_data *data)
{
	int32_t type = 0;

	ssp_infof("[SSC] run event test...");
	set_sensors_sampling_rate();
	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(atomic64_read(&data->sensor_en_state) & (1ULL << type)))
			continue;

		ssc.sensor[type].min_diff = 0xffffffffffffffff;
		ssc.sensor[type].max_diff = 0;
		enable_legacy_sensor(data, type);
		usleep_range(500, 1000);
	}

	msleep(100);
	start_system_checker();
	msleep(1000);
	stop_system_checker();
}

static void sensor_on_off(struct ssp_data *data)
{
	int32_t type = 0;

	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(atomic64_read(&data->sensor_en_state) & (1ULL << type)))
			continue;

		enable_legacy_sensor(data, type);
		usleep_range(500, 1000);
		disable_legacy_sensor(data, type);
		usleep_range(500, 1000);
	}
}

static void run_system_test(struct ssp_data *data)
{
	int32_t count = 0;

	ssp_infof("[SSC] run system test...");
	ssp_send_command(data, CMD_SETVALUE, TYPE_MCU, HUB_SYSTEM_CHECK, 0, NULL, 0, NULL, NULL);
	usleep_range(500, 1000);

	for (count = 0 ; count < ssc_count; count++)
		sensor_on_off(data);
}

void comm_test_cb(int32_t type)
{
	ssc.sensor[type].comm_test_response_cnt++;
}

void event_test_cb(int32_t type, uint64_t timestamp)
{
	uint64_t temp = 0;

	if (type >= LEGACY_SENSOR_MAX || type < SENSOR_TYPE_ACCELEROMETER)
		return;

	if (!ssc.sensor[type].pre_event_timestamp) {
		ssc.sensor[type].pre_event_timestamp = timestamp;
		return;
	}

	temp = timestamp - ssc.sensor[type].pre_event_timestamp;
	temp = temp > ssc.sensor[type].expected_delay_ns ?
		temp - ssc.sensor[type].expected_delay_ns : ssc.sensor[type].expected_delay_ns - temp;

	ssc.sensor[type].min_diff = temp < ssc.sensor[type].min_diff ? temp : ssc.sensor[type].min_diff;
	ssc.sensor[type].max_diff = temp > ssc.sensor[type].max_diff ? temp : ssc.sensor[type].max_diff;
	ssc.sensor[type].standard_deviation += (temp * temp);
	ssc.sensor[type].event_test_response_cnt++;
	ssc.sensor[type].pre_event_timestamp = timestamp;
}

static void restore_test_env(struct ssp_data *data)
{
	int32_t type = 0;

	ssp_infof("");
	atomic64_set(&data->sensor_en_state, ssc.env_backup.pre_sensor_state);
	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(atomic64_read(&data->sensor_en_state) & (1ULL << type)))
			continue;

		data->delay[type].sampling_period = ssc.env_backup.pre_rate[type].sampling_ms;
		data->delay[type].max_report_latency = ssc.env_backup.pre_rate[type].report_ms;
	}
}

static uint64_t test_sqrt(uint64_t in)
{
	int32_t count = 30;
	uint64_t out = 1;

	for (count; count > 0; --count)
		out = (out + in / out) / 2;

	return out;
}

static void report_test_result(struct ssp_data *data)
{
	enum system_test_op {
		OPERATION_OPEN = 0,
		OPERATION_READ,
		OPERATION_CLOSE,
		OPERATION_MAX,
	};

	struct sensor_debug_factor {
		int32_t count;
		uint64_t tt_sum;
		uint64_t tt_max;
		uint64_t tt_min;
	} __attribute__((__packed__));

	struct system_test {
		uint8_t type;
		struct sensor_debug_factor op[OPERATION_MAX];
	} __attribute__((__packed__));

	int32_t type = 0;
	int32_t count = 0;
	int32_t index = 0;
	int32_t buffer_length = 0;
	int8_t *buffer = NULL;

	ssp_infof("[SSC] == comm test result ======");
	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(data->sensor_probe_state & (1ULL << type)))
			continue;

		ssp_infof("[SSC] sensor(%2d) : %d", type, ssc.sensor[type].comm_test_response_cnt);
		if (ssc.sensor[type].comm_test_response_cnt != ssc_count)
			ssp_infof("sensor(%d) failed!!", type);
	}

	ssp_infof("[SSC] == event test result ======");
	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(data->sensor_probe_state & (1ULL << type)))
			continue;

		ssp_infof("[SSC] sensor(%2d) : cnt(%4d) std dev(%6lld), min(%6lld), max(%6lld)",
			  type,
			  ssc.sensor[type].event_test_response_cnt,
			  test_sqrt(ssc.sensor[type].standard_deviation / ssc.sensor[type].event_test_response_cnt),
			  ssc.sensor[type].min_diff,
			  ssc.sensor[type].max_diff);
	}

	ssp_infof("[SSC] == system test result ======");
	ssp_send_command(ssc.data, CMD_GETVALUE, TYPE_MCU, HUB_SYSTEM_CHECK,
			 1000, NULL, 0, (char **)&buffer, &buffer_length);
	if (!buffer || buffer_length == 0) {
		ssp_infof("[SSC] ERROR! Fail to get system test result!!");
		return;
	}
	count = buffer[0];
	index++;
	for (type = 0; type < count; type++) {
		struct system_test *result = (struct system_test *)(buffer + index);

		index += sizeof(struct system_test);

		ssp_infof("[SSC] sensor(%2d)", result->type);
		ssp_infof("[SSC]    OPEN  - cnt(%4d) min(%7lld), max(%7lld), avg(%7lld)",
				result->op[OPERATION_OPEN].count,
				result->op[OPERATION_OPEN].tt_min,
				result->op[OPERATION_OPEN].tt_max,
				result->op[OPERATION_OPEN].tt_sum / result->op[OPERATION_OPEN].count);

		ssp_infof("[SSC]    READ  - cnt(%4d) min(%7lld), max(%7lld), avg(%7lld)",
				result->op[OPERATION_READ].count,
				result->op[OPERATION_READ].tt_min,
				result->op[OPERATION_READ].tt_max,
				result->op[OPERATION_READ].tt_sum / result->op[OPERATION_READ].count);

		ssp_infof("[SSC]    CLOSE - cnt(%4d) min(%7lld), max(%7lld), avg(%7lld)",
				result->op[OPERATION_CLOSE].count,
				result->op[OPERATION_CLOSE].tt_min,
				result->op[OPERATION_CLOSE].tt_max,
				result->op[OPERATION_CLOSE].tt_sum / result->op[OPERATION_CLOSE].count);
	}

	kfree(buffer);
}

void run_system_check(void)
{
	static int32_t test_cnt = 1;

	ssp_infof("[SSC] !! #%d test begin !!!!!!!!!!!!!!!!!!", test_cnt++);
	ssp_infof("[SSC] !! DELAY(%dusec) COUNT(%d)", ssc_delay_us, ssc_count);

	run_system_test(ssc.data);
	run_comm_test(ssc.data);
	run_event_test(ssc.data);

	usleep_range(500, 1000);

	restore_test_env(ssc.data);
	report_test_result(ssc.data);
	reset_mcu(ssc.data, RESET_TYPE_KERNEL_SYSFS);
}

void system_ready_cb(void)
{
	stop_system_checker();
	run_system_check();
}

static void prepare_test_env(struct ssp_data *data)
{
	int32_t idx = 0, type = 0;
	int32_t spec_count = data->sensor_spec_size / sizeof(struct sensor_spec_t);
	struct sensor_spec_t *spec = (struct sensor_spec_t *)data->sensor_spec;

	ssp_infof("");
	// init check_system data
	memset(&ssc, 0, sizeof(ssc));
	ssc.data = data;
	for (idx = 0; idx < spec_count; idx++) {
		type = spec[idx].uid;
		ssc.sensor[type].expected_delay_ns = spec[idx].min_delay * 1000;
	}
	// backup shub env
	ssc.env_backup.pre_sensor_state = atomic64_read(&data->sensor_en_state);
	atomic64_set(&ssc.data->sensor_en_state, ssc.data->sensor_probe_state);
	for (type = SENSOR_TYPE_ACCELEROMETER; type < LEGACY_SENSOR_MAX; type++) {
		if (!(atomic64_read(&data->sensor_en_state) & (1ULL << type)))
			continue;

		ssc.env_backup.pre_rate[type].sampling_ms = data->delay[type].sampling_period;
		ssc.env_backup.pre_rate[type].report_ms = data->delay[type].max_report_latency;
	}
}

void sensorhub_system_check(struct ssp_data *data, uint32_t test_delay_us, uint32_t test_count)
{
	ssc_delay_us = test_delay_us && test_delay_us < 5000 ? test_delay_us : SSC_DEFAULT_DELAY;
	ssc_count = test_count && test_count < 100 ? test_count : SSC_DEFAULT_COUNT;

	prepare_test_env(data);
	start_system_checker();
	reset_mcu(data, RESET_TYPE_KERNEL_SYSFS);
}

void ssp_system_checker_init(void)
{
	mutex_init(&ssc.system_test_mutex);
}

bool is_system_checking(void)
{
	return ssc.is_system_checking;
}

void ssp_system_check_lock(void)
{
	mutex_lock(&ssc.system_test_mutex);
}

void ssp_system_check_unlock(void)
{
	mutex_unlock(&ssc.system_test_mutex);
}

#endif
