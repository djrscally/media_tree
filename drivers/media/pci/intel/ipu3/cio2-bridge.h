/* SPDX-License-Identifier: GPL-2.0 */
/* Author: Dan Scally <djrscally@gmail.com> */
#ifndef __CIO2_BRIDGE_H
#define __CIO2_BRIDGE_H

#include <linux/property.h>

#define CIO2_HID				"INT343E"
#define CIO2_NUM_PORTS				4

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

enum cio2_sensor_swnodes {
	SWNODE_SENSOR_HID,
	SWNODE_SENSOR_PORT,
	SWNODE_SENSOR_ENDPOINT,
	SWNODE_CIO2_PORT,
	SWNODE_CIO2_ENDPOINT,
	NR_OF_SENSOR_SWNODES
};

/* Data representation as it is in ACPI SSDB buffer */
struct cio2_sensor_ssdb {
	u8 version;				/* 0000 */
	u8 sku;					/* 0001 */
	u8 guid_csi2[16];			/* 0002 */
	u8 devfunction;				/* 0003 */
	u8 bus;					/* 0004 */
	u32 dphylinkenfuses;			/* 0005 */
	u32 clockdiv;				/* 0009 */
	u8 link;				/* 0013 */
	u8 lanes;				/* 0014 */
	u32 csiparams[10];			/* 0015 */
	u32 maxlanespeed;			/* 0019 */
	u8 sensorcalibfileidx;			/* 0023 */
	u8 sensorcalibfileidxInMBZ[3];		/* 0024 */
	u8 romtype;				/* 0025 */
	u8 vcmtype;				/* 0026 */
	u8 platforminfo;			/* 0027 */
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
} __packed;

struct cio2_property_names {
	char clock_frequency[16];
	char rotation[9];
	char bus_type[9];
	char data_lanes[11];
	char remote_endpoint[16];
};

struct cio2_node_names {
	char port[7];
	char endpoint[11];
	char remote_port[7];
};

struct cio2_sensor {
	char name[ACPI_ID_LEN];
	struct acpi_device *adev;

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
};

struct cio2_bridge {
	char cio2_node_name[ACPI_ID_LEN];
	struct software_node cio2_hid_node;
	unsigned int n_sensors;
	struct cio2_sensor sensors[CIO2_NUM_PORTS];
};

#endif
