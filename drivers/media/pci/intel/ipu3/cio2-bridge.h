/* SPDX-License-Identifier: GPL-2.0 */
/* Author: Dan Scally <djrscally@gmail.com> */
#ifndef __CIO2_BRIDGE_H
#define __CIO2_BRIDGE_H

#include <linux/gpio/machine.h>
#include <linux/property.h>
#include <linux/regulator/machine.h>

#define MAX_CONNECTED_DEVICES				4
#define SWNODE_SENSOR_HID				0
#define SWNODE_SENSOR_PORT				1
#define SWNODE_SENSOR_ENDPOINT				2
#define SWNODE_CIO2_PORT				3
#define SWNODE_CIO2_ENDPOINT				4
#define SWNODE_NULL_TERMINATOR				5

#define CIO2_HID					"INT343E"
#define CIO2_PCI_ID					0x9d32
#define PMIC_HID					"INT3472"

#define ENDPOINT_SENSOR					0
#define ENDPOINT_CIO2					1

#define CRS_END_TAG_DESCRIPTOR				0x79
#define CRS_RESOURCE_SIZE_BITMASK			0x80
#define CRS_RESOURCE_TYPE_BITMASK			0x7f
#define CRS_RESOURCE_S_SIZE_BITMASK			0x07
#define CRS_RESOURCE_L_SIZE_OFFSET_LOW			0x01
#define CRS_RESOURCE_L_SIZE_OFFSET_HIGH			0x02
#define CRS_GPIO_CONNECTION_TYPE_OFFSET			0x04
#define CRS_GPIO_PIN_TABLE_OFFSET_LOW			0x0e
#define CRS_GPIO_PIN_TABLE_OFFSET_HIGH			0x0f
#define CRS_GPIO_RESOURCE_NAME_OFFSET_LOW		0x11
#define CRS_GPIO_RESOURCE_NAME_OFFSET_HIGH		0x12
#define CRS_GPIO_VENDOR_DATA_OFFSET_LOW			0x13
#define CRS_GPIO_VENDOR_DATA_OFFSET_HIGH		0x14

#define NODE_SENSOR(_HID, _PROPS)			\
	((const struct software_node) {			\
		.name = _HID,				\
		.properties = _PROPS,			\
	})

#define NODE_PORT(_PORT, _SENSOR_NODE)			\
	((const struct software_node) {			\
		_PORT,					\
		_SENSOR_NODE,				\
	})

#define NODE_ENDPOINT(_EP, _PORT, _PROPS)		\
	((const struct software_node) {			\
		_EP,					\
		_PORT,					\
		_PROPS,					\
	})

#define GPIO_REGULATOR_NAME_LENGTH			17
#define GPIO_REGULATOR_SUPPLY_NAME_LENGTH		9

#define GPIO_REGULATOR(_NAME, _SUPPLY, _ID, _OPS)	\
	((const struct regulator_desc) {		\
		.name = _NAME,				\
		.supply_name = _SUPPLY,			\
		.id = _ID,				\
		.type = REGULATOR_VOLTAGE,		\
		.ops = _OPS,				\
		.owner = THIS_MODULE,			\
	})

const guid_t pmic_gpio_data_guid = GUID_INIT(0x79234640, 0x9e10, 0x4fea, 0xa5,
					     0xc1, 0xb5, 0xaa, 0x8b, 0x19, 0x75, 0x6f);

const guid_t sensor_module_guid = GUID_INIT(0x822ace8f, 0x2814, 0x4174, 0xa5,
					    0x6b, 0x5f, 0x02, 0x9f, 0xe0, 0x79, 0xee);

struct cio2_gpio_regulator {
	char regulator_name[GPIO_REGULATOR_NAME_LENGTH];
	char supply_name[GPIO_REGULATOR_SUPPLY_NAME_LENGTH];
	struct gpio_desc *gpio;
	struct regulator_dev *rdev;
	struct regulator_desc rdesc;
	struct list_head list;
};

struct cio2_gpio_pin {
	struct gpiod_lookup_table *lookup;
	struct list_head list;
};

struct pmic {
	struct device *dev;
	struct acpi_device *adev;
	unsigned int n_regulators;
	struct list_head regulators;
	unsigned int n_gpios;
	struct list_head gpios;
};

struct sensor {
	char name[ACPI_ID_LEN];
	struct device *dev;
	struct acpi_device *adev;
	struct pmic pmic;
	struct gpiod_lookup_table gpios;
	struct software_node swnodes[6];
	struct property_entry dev_properties[3];
	struct property_entry ep_properties[4];
	struct property_entry cio2_properties[3];
	u32 *data_lanes;
};

struct cio2_bridge {
	int n_sensors;
	struct sensor sensors[MAX_CONNECTED_DEVICES];
};

/* Data representation as it is in ACPI SSDB buffer */
struct sensor_bios_data_packed {
	u8 version;
	u8 sku;
	u8 guid_csi2[16];
	u8 devfunction;
	u8 bus;
	u32 dphylinkenfuses;
	u32 clockdiv;
	u8 link;
	u8 lanes;
	u32 csiparams[10];
	u32 maxlanespeed;
	u8 sensorcalibfileidx;
	u8 sensorcalibfileidxInMBZ[3];
	u8 romtype;
	u8 vcmtype;
	u8 platforminfo;
	u8 platformsubinfo;
	u8 flash;
	u8 privacyled;
	u8 degree;
	u8 mipilinkdefined;
	u32 mclkspeed;
	u8 controllogicid;
	u8 reserved1[3];
	u8 mclkport;
	u8 reserved2[13];
};

/* Fields needed by bridge driver */
struct sensor_bios_data {
	struct device *dev;
	u8 link;
	u8 lanes;
	u8 degree;
	u32 mclkspeed;
};

struct pmic_cldb {
	u8 version;
	/*
	 * control logic type
	 * 0: UNKNOWN
	 * 1: DISCRETE(CRD-D)
	 * 2: PMIC TPS68470
	 * 3: PMIC uP6641
	 */
	u8 control_logic_type;
	u8 control_logic_id;
	u8 sensor_card_sku;
	u8 reserved[28];
};

struct cio2_sensor_regulator_map {
	char *sensor_module_name;
	unsigned int n_supplies;
	struct regulator_consumer_supply *supplies;
};

/*
 * Here follows platform specific mapping information that we can pass to
 * regulator_init_data when we register our regulators. They're just mapped
 * via index, I.E. the first regulator pin that the code finds for the
 * i2c-OVTI2680:00 device is avdd, the second is dovdd and so on.
 */

const struct regulator_consumer_supply miix_510_ov2680[] = {
	{ "i2c-OVTI2680:00", "avdd" },
	{ "i2c-OVTI2680:00", "dovdd" },
};

const struct cio2_sensor_regulator_map cio2_sensor_regulator_maps[] = {
	{ "GNDF140809R", 2, miix_510_ov2680 },
};

#endif

