/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/bluetooth/services/ias.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/gpio.h>


#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Custom Service Variables */
#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
    BT_UUID_CUSTOM_SERVICE_VAL);

static struct bt_uuid_128 vnd_enc_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

#define VND_MAX_LEN 20

static struct bt_conn *default_conn;

#define VND_LONG_MAX_LEN 74

static void pong_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t val)
{
	printk("val %u\n", val);

	// remote_ready = (val == BT_GATT_CCC_NOTIFY);

	// if (remote_ready && remote_handle) {
	// 	pong_conn_ready(initiator);
	// }
}

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(vnd_svc,
    BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
    BT_GATT_CHARACTERISTIC(&vnd_enc_uuid.uuid,
                   BT_GATT_CHRC_NOTIFY,
                   BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(pong_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
              BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
              BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
              BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err 0x%02x)\n", err);
    } else {
        printk("Connected\n");
    }

    if (!default_conn) {
		default_conn = bt_conn_ref(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);

    if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static void alert_stop(void)
{
    printk("Alert stopped\n");
}

static void alert_start(void)
{
    printk("Mild alert started\n");
}

static void alert_high_start(void)
{
    printk("High alert started\n");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

BT_IAS_CB_DEFINE(ias_callbacks) = {
    .no_alert = alert_stop,
    .mild_alert = alert_start,
    .high_alert = alert_high_start,
};

static void bt_ready(void)
{
    int err;

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

void main(void)
{
    struct bt_gatt_attr *vnd_ind_attr;
    char str[BT_UUID_STR_LEN];
    int err;
    uint32_t dtr = 0;
    const struct device *const dev = DEVICE_DT_GET_ONE(st_vl53l0x);
    struct sensor_value value;
    int ret;

#ifdef CONFIG_BOARD_ARDUINO_NANO_33_BLE
	const struct device *const lis3mdl = DEVICE_DT_GET_ONE(st_lis3mdl_magn);
	const struct device *const lsm6ds0 = DEVICE_DT_GET_ONE(st_lsm6ds0);
	struct sensor_value magn_xyz[3], accel_xyz[3];

    while (!dtr) {
        uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }
    if (!device_is_ready(lis3mdl)) {
		printk("%s: device not ready.\n", lis3mdl->name);
		return;
	}
	if (!device_is_ready(lsm6ds0)) {
		printk("%s: device not ready.\n", lsm6ds0->name);
		return;
	}

#endif

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    bt_ready();

    bt_gatt_cb_register(&gatt_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

    vnd_ind_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count,
                        &vnd_enc_uuid.uuid);

    bt_uuid_to_str(&vnd_enc_uuid.uuid, str, sizeof(str));
    printk("Indicate VND attr %p (UUID %s)\n", vnd_ind_attr, str);

    if (!device_is_ready(dev)) {
        printk("sensor: device not ready.\n");
        return;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    /* Implement notification. At the moment there is no suitable way
     * of starting delayed work so we do it here
     */
    int k = 10;
    char string[256] = {};
    while (1) {
        k_sleep(K_MSEC(100));
        if(!k--) {
            gpio_pin_toggle_dt(&led);
            k = 10;
        }

        ret = sensor_sample_fetch(dev);
        if (ret) {
            printk("sensor_sample_fetch failed ret %d\n", ret);
            return;
        }
        if (sensor_sample_fetch(lis3mdl) < 0) {
			printk("LIS3MDL Sensor sample update error\n");
			return;
		}
        if (sensor_sample_fetch(lsm6ds0) < 0) {
			printk("LSM6DS0 Sensor sample update error\n");
			return;
		}

        ret = sensor_channel_get(dev, SENSOR_CHAN_PROX, &value);
        ret = sensor_channel_get(dev,
                     SENSOR_CHAN_DISTANCE,
                     &value);

		sensor_channel_get(lis3mdl, SENSOR_CHAN_MAGN_XYZ, magn_xyz);
		sensor_channel_get(lsm6ds0, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);

        float val = sensor_value_to_double(&value);
        uint16_t valint = val * 10000;

        printk("distance is %d\n", valint);

        /* magneto data */
		sprintf(
            string,
		    "LIS3MDL: Magnetic field (gauss): x: %f, y: %f, z: %f\n",
		    sensor_value_to_double(&magn_xyz[0]),
		    sensor_value_to_double(&magn_xyz[1]),
		    sensor_value_to_double(&magn_xyz[2]));

        printk(string);

		/* acceleration */
		// printk(
		//    "LSM6DS0: Acceleration (m.s-2): x: %f, y: %f, z: %f\n",
		//    sensor_value_to_double(&accel_xyz[0]),
		//    sensor_value_to_double(&accel_xyz[1]),
		//    sensor_value_to_double(&accel_xyz[2]));

        /* Heartrate measurements simulation */
        bt_hrs_notify(69);

        // ind_params.attr = vnd_ind_attr;
        // ind_params.func = indicate_cb;
        // ind_params.destroy = indicate_destroy;
        // ind_params.data = &valint;
        // ind_params.len = sizeof(valint);

        // bt_gatt_indicate(NULL, &ind_params);
        ret = bt_gatt_notify(default_conn, &vnd_svc.attrs[1], &valint, sizeof(valint));
        if(ret) {
            printk("%d\n", ret);
        }
    }
}
