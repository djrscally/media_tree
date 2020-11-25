// SPDX-License-Identifier: GPL-2.0
/* Author: Dan Scally <djrscally@gmail.com> */
#ifndef __CIO2_BRIDGE_H
#define __CIO2_BRIDGE_H

#include <linux/gpio/machine.h>
#include <linux/property.h>
#include <linux/regulator/machine.h>

#define CIO2_HID				"INT343E"
#define CIO2_NUM_PORTS                          4
#define CIO2_PMIC_HID                           "INT3472"
#define CIO2_PMIC_MAX_GPIOS                     2

#define NODE_SENSOR(_HID, _PROPS)		\
	((const struct software_node) {		\
		.name = _HID,			\
		.properties = _PROPS,		\
	})

#define NODE_PORT(_PORT, _SENSOR_NODE)		\
	((const struct software_node) {		\
		_PORT,				\
		_SENSOR_NODE,			\
	})

#define NODE_ENDPOINT(_EP, _PORT, _PROPS)	\
	((const struct software_node) {		\
		_EP,				\
		_PORT,				\
		_PROPS,				\
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

const guid_t cio2_pmic_gpio_guid = GUID_INIT(0x79234640, 0x9e10, 0x4fea,
                                             0xa5, 0xc1, 0xb5, 0xaa, 0x8b,
                                             0x19, 0x75, 0x6f);

const guid_t cio2_sensor_module_guid = GUID_INIT(0x822ace8f, 0x2814, 0x4174,
                                                 0xa5, 0x6b, 0x5f, 0x02, 0x9f,
                                                 0xe0, 0x79, 0xee);

enum cio2_sensor_swnodes {
        SWNODE_SENSOR_HID,
        SWNODE_SENSOR_PORT,
        SWNODE_SENSOR_ENDPOINT,
        SWNODE_CIO2_PORT,
        SWNODE_CIO2_ENDPOINT,
        NR_OF_SENSOR_SWNODES
};

struct cio2_gpio_pin {
	struct list_head list;
	struct gpiod_lookup_table lookup;
};

struct cio2_pmic_cldb {
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

struct cio2_gpio_regulator {
	char regulator_name[GPIO_REGULATOR_NAME_LENGTH];
	char supply_name[GPIO_REGULATOR_SUPPLY_NAME_LENGTH];
	struct gpio_desc *gpio;
	struct regulator_dev *rdev;
	struct regulator_desc rdesc;
	struct list_head list;
};

struct cio2_pmic {
	struct acpi_device *adev;
	unsigned int n_regulators;
	struct list_head regulators;
	unsigned int n_gpios;
	struct list_head gpios;
        struct cio2_pmic_cldb cldb;
};

/* Data representation as it is in ACPI SSDB buffer */
struct cio2_sensor_ssdb {
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
} __packed__;

struct cio2_property_names {
        char clock_frequency[16];
        char rotation[9];
        char bus_type[9];
        char data_lanes[11];
        char remote_endpoint[16];
};

struct cio2_node_names {
        char port[6];
        char endpoint[10];
        char remote_port[6];
};

struct cio2_sensor {
        char name[ACPI_ID_LEN];
        struct acpi_device *adev;
        struct cio2_pmic pmic;

        struct software_node swnodes[6];
        struct cio2_node_names node_names;

        u32 data_lanes[4];
        struct cio2_sensor_ssdb ssdb;
        struct cio2_property_names prop_names;
        struct property_entry ep_properties[4];
        struct property_entry dev_properties[3];
        struct property_entry cio2_properties[3];
        struct software_node_ref_args local_ref[1];
        struct software_node_ref_args remote_ref[1];

        unsigned int n_gpios;
        struct gpiod_lookup_table *gpios;
};

struct cio2_bridge {
        struct pci_dev *cio2;
        char cio2_node_name[ACPI_ID_LEN];
        struct software_node cio2_hid_node;
        unsigned int n_sensors;
        struct cio2_sensor sensors[CIO2_NUM_PORTS];
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

const struct regulator_consumer_supply surface_go2_ov5693[] = {
        { "i2c-INT33BE:00", "avdd" },
        { "i2c-INT33BE:00", "dovdd" },
};

const struct regulator_consumer_supply surface_book_ov5693[] = {
        { "i2c-INT33BE:00", "avdd" },
        { "i2c-INT33BE:00", "dovdd" },
};

const struct cio2_sensor_regulator_map cio2_sensor_regulator_maps[] = {
	{ "GNDF140809R", 2, miix_510_ov2680 },
        { "YHCU", 2, surface_go2_ov5693 },
        { "MSHW0070", 2, surface_book_ov5693 },
};

#endif
