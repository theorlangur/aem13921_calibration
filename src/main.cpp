/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/settings/settings.h>
#include <zephyr/input/input.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor/aem13921.hpp>

#include <zephyr/bluetooth/bluetooth.h>

LOG_MODULE_REGISTER(main_log, LOG_LEVEL_INF);


template<class... O>
struct overloaded:O...
{
    using O::operator()...;
};
template<class... O>
overloaded(O... o)->overloaded<O...>;

/**********************************************************************/
/* For testing purposes                                               */
/**********************************************************************/
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec buck_enable = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, buck_enable_gpios);
static const struct gpio_dt_spec apm_src1 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, apm_src1_gpios);
static const struct gpio_dt_spec apm_src2 = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, apm_src2_gpios);
static const struct gpio_dt_spec apm_buck = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, apm_buck_gpios);
static const struct gpio_dt_spec apm_5v = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, apm_5v_gpios);

static const struct device *const aem13921_dev = DEVICE_DT_GET(DT_NODELABEL(aem13921));
static const struct gpio_dt_spec aem13921_irq = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, apm_irq_gpios);

static const struct gpio_dt_spec measurement_active = GPIO_DT_SPEC_GET(DT_NODELABEL(measurement_active), gpios);

static struct gpio_callback aem_cb_data;

static const struct led_dt_spec led0 = LED_DT_SPEC_GET(DT_NODELABEL(led0));


static struct bt_le_ext_adv *ext_adv;
// Up to ~240 bytes fits cleanly in a single unfragmented extended PDU
static char adv_str[220] = "AEM13921: Initial...";

static struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, adv_str, sizeof(adv_str) - 1),
};

void init_ext_beacon(uint8_t version, int err_pins, int err_aem) {
    bt_enable(nullptr);

    // Create a non-connectable, non-scannable extended advertising set
    bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, nullptr, &ext_adv);

    int total_len;
    total_len = snprintf(adv_str, sizeof(adv_str), "AEM13921 Calibrator: ");
    if (err_pins < 0)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " EPins:%d(%s)", err_pins, strerror(err_pins));
    if (err_aem < 0)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " EAEM:%d(%s)", err_aem, strerror(-err_aem));
    if (!err_pins && !err_aem)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " OK, Version: %d", version);

    ad[0].data_len = total_len;
    bt_le_ext_adv_set_data(ext_adv, ad, ARRAY_SIZE(ad), nullptr, 0);
    bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
}

static uint64_t total_load_power = 0;
static struct k_work aem_work;
static struct k_work aem_finished;
static struct k_work_delayable update_cfg_delayable;

bool g_measurement_state_active = false;
bool g_first_measurement_pending = false;
bool g_async_measurement_stop_request = false;

bool buck_enable_state = false;
bool apm_src1_enable_state = false;
bool apm_src2_enable_state = false;
bool apm_buck_enable_state = false;
bool apm_5v_enable_state = false;

constexpr int measurement_duration_in_windows = (60/*seconds*/ / 0.233/*single window duration*/) + 1;
int windows_accumulated_left = 0;
int errors_encountered = 0;
size_t total_power_src1 = 0;
size_t total_power_src2 = 0;
size_t total_power_buck = 0;
size_t total_power_5v = 0;

static void aem_work_handler(struct k_work *work)
{
    auto state = epeas::aem1392x_get_irq_state(aem13921_dev);
    if (g_first_measurement_pending)
    {
	led_on_dt(&led0);
	g_first_measurement_pending = false;
	//nothing to actually read, we just wanted to clear the IRQ
	if (!g_measurement_state_active)
	{
	    //we're done
	    k_work_submit(&aem_finished);
	}
	return;
    }

    if (state)
    {
	auto s = *state;
	if (s.bits.apm_done)
	{
	    auto add_apm = [](size_t &total, const auto &apm, const char *pPrefix){
		if (apm)
		{
		    std::visit(
			    overloaded{
				[&](const epeas::apm_counter_acc_t &rv) { total += rv.get_value(); },
				[&](const epeas::apm_counter_no_acc_t &rv) { total += rv.value.counter; },
				[&](const epeas::apm_power_meter_acc_t &rv) { total += rv.get_value(); }
			    },
			    *apm
			);
		}else
		{
		    LOG_ERR("%s: failed with %d %s", pPrefix, apm.error(), strerror(apm.error()));
		}
	    };

	    if (apm_buck_enable_state)
	    {
		auto apm_load = epeas::aem1392x_get_apm_load(aem13921_dev);
		add_apm(total_power_buck, apm_load, "APM Load");
	    }

	    if (apm_src1_enable_state)
	    {
		auto apm_src1 = epeas::aem1392x_get_apm_src1(aem13921_dev);
		add_apm(total_power_src1, apm_src1, "APM Src1");
	    }

	    if (apm_src2_enable_state)
	    {
		auto apm_src2 = epeas::aem1392x_get_apm_src2(aem13921_dev);
		add_apm(total_power_src2, apm_src2, "APM Src2");
	    }

	    if (apm_5v_enable_state)
	    {
		auto apm_5v = epeas::aem1392x_get_apm_ch5v(aem13921_dev);
		if (apm_5v)
		{
		    total_power_5v += apm_5v->value;
		}else
		{
		    LOG_ERR("apm 5v: failed with %d %s", apm_5v.error(), strerror(apm_5v.error()));
		}
	    }

	    auto apm_err = epeas::aem1392x_get_apm_error(aem13921_dev);
	    LOG_INF("APM Err: 0x%X", apm_err->byte);
	}

	if (s.bits.apm_error)
	{
	    ++errors_encountered;
	    auto apm_err = epeas::aem1392x_get_apm_error(aem13921_dev);
	    if (apm_err)
	    {
		LOG_INF("APM Err: 0x%X", apm_err->byte);
	    }else
	    {
		LOG_ERR("apm err: failed with %d %s", apm_err.error(), strerror(apm_err.error()));
	    }
	}
    }else
    {
	LOG_ERR("irq state: failed with %d %s", state.error(), strerror(state.error()));
    }

    if (!g_measurement_state_active)
    {
	led_off_dt(&led0);
	//we're done
	k_work_submit(&aem_finished);
    }
}

static void aem_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (g_measurement_state_active)
    {
	if (g_first_measurement_pending && !g_async_measurement_stop_request)
	{
	    //g_first_measurement_pending will be reset in aem_work and the calculations skipped
	    //here we just set the indicator pin high to show that we've started measurement
	    gpio_pin_set_dt(&measurement_active, 1);
	}else
	{
	    //normal mode
	    if (--windows_accumulated_left <= 0 || g_async_measurement_stop_request)
	    {
		//we're done
		gpio_pin_set_dt(&measurement_active, 0);
		g_measurement_state_active = false;
		g_async_measurement_stop_request = false;
	    }
	}
	k_work_submit(&aem_work);
    }
}

void update_config_pins()
{
    buck_enable_state = gpio_pin_get_dt(&buck_enable);
    apm_src1_enable_state = gpio_pin_get_dt(&apm_src1);
    apm_src2_enable_state = gpio_pin_get_dt(&apm_src2);
    apm_buck_enable_state = gpio_pin_get_dt(&apm_buck);
    apm_5v_enable_state = gpio_pin_get_dt(&apm_5v);
}

static void aem_update_cfg(struct k_work *work)
{
    update_config_pins();
    bt_le_ext_adv_stop(ext_adv);
    int total_len = snprintf(adv_str, sizeof(adv_str), "AEM13921:");
    if (buck_enable_state)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " B;");
    if (apm_src1_enable_state)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " AS1;");
    if (apm_src2_enable_state)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " AS2;");
    if (apm_buck_enable_state)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " AB;");
    if (apm_5v_enable_state)
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " A5;");

    ad[0].data_len = total_len;
    bt_le_ext_adv_set_data(ext_adv, ad, ARRAY_SIZE(ad), nullptr, 0);
    bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
}

static struct gpio_callback cfg_update_cb;

static void update_pin_cfg_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (!g_measurement_state_active)
    {
	k_work_reschedule(&update_cfg_delayable, K_MSEC(30));
    }
}

int configure_config_pins()
{
    int err = 0;
    err = gpio_pin_configure_dt(&buck_enable, GPIO_INPUT);
    if (err < 0) return err;

    err = gpio_pin_configure_dt(&apm_src1, GPIO_INPUT);
    if (err < 0) return err;

    err = gpio_pin_configure_dt(&apm_src2, GPIO_INPUT);
    if (err < 0) return err;

    err = gpio_pin_configure_dt(&apm_buck, GPIO_INPUT);
    if (err < 0) return err;

    err = gpio_pin_configure_dt(&apm_5v, GPIO_INPUT);
    if (err < 0) return err;

    err = gpio_pin_configure_dt(&measurement_active, GPIO_OUTPUT);
    if (err < 0) return err;

    k_work_init_delayable(&update_cfg_delayable, aem_update_cfg);
    err = gpio_pin_interrupt_configure_dt(&buck_enable, GPIO_INT_EDGE_BOTH);
    if (err < 0) return err;
    err = gpio_pin_interrupt_configure_dt(&apm_src1, GPIO_INT_EDGE_BOTH);
    if (err < 0) return err;
    err = gpio_pin_interrupt_configure_dt(&apm_src2, GPIO_INT_EDGE_BOTH);
    if (err < 0) return err;
    err = gpio_pin_interrupt_configure_dt(&apm_buck, GPIO_INT_EDGE_BOTH);
    if (err < 0) return err;
    err = gpio_pin_interrupt_configure_dt(&apm_5v, GPIO_INT_EDGE_BOTH);
    if (err < 0) return err;
    gpio_init_callback(&cfg_update_cb, update_pin_cfg_isr, BIT(buck_enable.pin) | BIT(apm_src1.pin) | BIT(apm_src2.pin) | BIT(apm_buck.pin) | BIT(apm_5v.pin));
    err = gpio_add_callback(apm_src1.port, &cfg_update_cb);
    if (err < 0) return err;

    return err;
}

int update_aem_configuration()
{
    if (buck_enable_state)
	epeas::aem1392x_configure_buck(aem13921_dev, epeas::buck_cfg_t{.bits = {.vload = epeas::vload_t::_3_0V, .tmult = epeas::tmult_t::x4}});
    else
	epeas::aem1392x_configure_buck(aem13921_dev, epeas::buck_cfg_t{.bits = {.vload = epeas::vload_t::off, .tmult = epeas::tmult_t::x4}});

    epeas::apm_cfg_t apm_cfg{.bits = {
	.src1 = apm_src1_enable_state,
	.src2 = apm_src2_enable_state,
	.buck = apm_buck_enable_state,
	.ch5v = apm_5v_enable_state,
	.mode = epeas::apm_mode_t::PowerMeter, 
	.win = epeas::apm_window_t::_233ms
    }};
    epeas::aem1392x_configure_src_boost(aem13921_dev, epeas::src_t::_1, epeas::boost_cfg_t{.bits = {.enabled = apm_cfg.bits.src1, .high_power = true}});
    epeas::aem1392x_configure_src_boost(aem13921_dev, epeas::src_t::_2, epeas::boost_cfg_t{.bits = {.enabled = apm_cfg.bits.src2, .high_power = true}});

    if (apm_src1_enable_state)
	epeas::aem1392x_configure_src_regulation_voltage(aem13921_dev, epeas::src_t::_1, 1.5f);
    if (apm_src2_enable_state)
	epeas::aem1392x_configure_src_regulation_voltage(aem13921_dev, epeas::src_t::_2, 1.5f);

    epeas::aem1392x_set_storage_overdischarge_threshold_voltage(aem13921_dev, 3.3f);
    epeas::aem1392x_set_storage_charge_ready_threshold_voltage(aem13921_dev, 3.4f);
    epeas::aem1392x_set_storage_overcharge_threshold_voltage(aem13921_dev, 3.8f);

    {
	epeas::voltage5v_t cfg{};
	cfg.bits.enabled = apm_cfg.bits.ch5v;
	cfg.bits.enabled_cv = false;
	cfg.bits.threshold = 0;
	epeas::aem1392x_configure_5v_charger(aem13921_dev, cfg);
    }

    epeas::aem1392x_configure_apm(aem13921_dev, apm_cfg);
    epeas::aem1392x_set_apm_accumulation_period(aem13921_dev, 1);
    bool needs_irq = (apm_cfg.byte & 0xF) != 0;//first four bits -> enable bits for APM
    {
	epeas::irq_event_t irq_cfg;
	irq_cfg.set_apm_done(needs_irq);
	irq_cfg.set_apm_error(needs_irq);
	epeas::aem1392x_configure_irq(aem13921_dev, irq_cfg);
    }
    if (needs_irq)
    {
	[[maybe_unused]]auto state = epeas::aem1392x_get_irq_state(aem13921_dev);//just to allow future IRQs
    }
    epeas::aem1392x_commit(aem13921_dev);
    return 0;
}

void prepare_measurements()
{
    g_async_measurement_stop_request = false;
    windows_accumulated_left = measurement_duration_in_windows;
    errors_encountered = 0;
    total_power_src1 = 0;
    total_power_src2 = 0;
    total_power_buck = 0;
    total_power_5v = 0;
    g_first_measurement_pending = true;
}

extern "C" void trigger_input_cb(struct input_event *evt, void *user_data)
{
    if (evt->type == INPUT_EV_KEY && evt->code == INPUT_KEY_0 && evt->value == 1) {
	g_measurement_state_active = !g_measurement_state_active;
	LOG_INF("trigger pressed: new active state: %d", g_measurement_state_active);
	if (g_measurement_state_active)
	{
	    bt_le_ext_adv_stop(ext_adv);
	    update_config_pins();
	    if (apm_src1_enable_state || apm_src2_enable_state || apm_buck_enable_state || apm_5v_enable_state)
		prepare_measurements();
	    update_aem_configuration();
	}else
	{
	    //request to stop whatever is running now
	    g_async_measurement_stop_request = true;
	}
    }
}
INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(buttons)), trigger_input_cb, NULL);


static void aem_work_done(struct k_work *work)
{
    bt_le_ext_adv_stop(ext_adv);
    //enable/update BLE beacon
    int windows_collected = measurement_duration_in_windows - windows_accumulated_left;
    int total_len = snprintf(adv_str, sizeof(adv_str), "AEM13921: W:%d; E:%d;", windows_collected, errors_encountered);
    if (apm_src1_enable_state)
    {
	double avg_power = windows_collected ? double(total_power_src1) / windows_collected : 0;
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " S1:%.2lf;", avg_power);
    }
    if (apm_src2_enable_state)
    {
	double avg_power = windows_collected ? double(total_power_src2) / windows_collected : 0;
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " S2:%.2f;", avg_power);
    }
    if (apm_buck_enable_state)
    {
	double avg_power = windows_collected ? double(total_power_buck) / windows_collected : 0;
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " B:%.2f;", avg_power);
    }
    if (apm_5v_enable_state)
    {
	double avg_power = windows_collected ? double(total_power_5v) / windows_collected : 0;
	total_len += snprintf(adv_str + total_len, sizeof(adv_str) - total_len, " 5:%.2f;", avg_power);
    }
    ad[0].data_len = total_len;
    bt_le_ext_adv_set_data(ext_adv, ad, ARRAY_SIZE(ad), nullptr, 0);
    bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
}

int configure_aem_isr()
{
    k_work_init(&aem_work, aem_work_handler);
    k_work_init(&aem_finished, aem_work_done);
    int err = 0;
    err = gpio_pin_configure_dt(&aem13921_irq, GPIO_INPUT);
    if (err < 0) 
    {
	LOG_INF("aem13921_irq cfg: %d (%s)", err, strerror(err));
	return err;
    }
    err = gpio_pin_interrupt_configure_dt(&aem13921_irq, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&aem_cb_data, aem_isr, BIT(aem13921_irq.pin));
    err = gpio_add_callback(aem13921_irq.port, &aem_cb_data);
    if (err < 0)
    {
	LOG_INF("aem13921_irq cb: %d (%s)", err, strerror(err));
	return err;
    }
    return err;
}

int main(void)
{
    uint8_t v = 0;
    int err_pins = 0, err_aem = 0;
    err_pins = configure_config_pins();
    if (device_is_ready(aem13921_dev))
    {
	err_aem = epeas::aem1392x_get_version(aem13921_dev, &v);
	if (err_aem < 0) 
	    LOG_INF("cfg pins: %d (%s)", err_aem, strerror(err_aem));
	err_aem = configure_aem_isr();
    }else
    {
	err_aem = -ENODEV;
    }

    init_ext_beacon(v, err_pins, err_aem);

    if (err_pins || err_aem)
    {
	while (1) {
	    led_on_dt(&led0);
	    k_msleep(300);
	    led_off_dt(&led0);
	    k_msleep(300);

	}
    }else
    {
	while (1) k_sleep(K_FOREVER);
    }

    return 0;
}

