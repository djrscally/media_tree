// SPDX-License-Identifier: GPL-2.0
// Author: Dan Scally <djrscally@gmail.com>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/driver.h>
#include <media/v4l2-subdev.h>

#include "cio2-bridge.h"

/*
 * Extend this array with ACPI Hardware ID's of devices known to be
 * working
 */
static const char * const supported_devices[] = {
	"INT33BE",
	"OVTI2680",
};

static struct software_node cio2_hid_node = { CIO2_HID };

static struct cio2_bridge bridge;

static const char * const port_names[] = {
	"port0", "port1", "port2", "port3"
};

static int cio2_bridge_gpio_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct gpio_desc *gpio;
	int value;

	gpio = rdev_get_drvdata(rdev);

	value = gpiod_get_value_cansleep(gpio);

	return value;
}

static struct regulator_ops cio2_bridge_gpio_regulator_ops = {
	.is_enabled         = cio2_bridge_gpio_regulator_is_enabled,
};

static const struct property_entry remote_endpoints[] = {
	PROPERTY_ENTRY_REF("remote-endpoint", /* Sensor 0, Sensor Property */
			   &bridge.sensors[0].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint", /* Sensor 0, CIO2 Property */
			   &bridge.sensors[0].swnodes[SWNODE_SENSOR_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[1].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[1].swnodes[SWNODE_SENSOR_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[2].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[2].swnodes[SWNODE_SENSOR_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[3].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[3].swnodes[SWNODE_SENSOR_ENDPOINT]),
};

static int read_acpi_block(struct device *dev, char *id, void *data, u32 size)
{
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_handle *handle;
	union acpi_object *obj;
	acpi_status status;
	int ret;

	handle = ACPI_HANDLE(dev);

	status = acpi_evaluate_object(handle, id, NULL, &buffer);
	if (ACPI_FAILURE(status)) {
		pr_info("%s: %d\n", __func__, 2);
		return -ENODEV;
	}

	obj = buffer.pointer;
	if (!obj) {
		dev_err(dev, "Couldn't locate ACPI buffer\n");
		return -ENODEV;
	}

	if (obj->type != ACPI_TYPE_BUFFER) {
		dev_err(dev, "Not an ACPI buffer\n");
		ret = -ENODEV;
		goto out_free_buff;
	}

	if (obj->buffer.length > size) {
		dev_err(dev, "Given buffer is too small\n");
		ret = -EINVAL;
		goto out_free_buff;
	}

	memcpy(data, obj->buffer.pointer, obj->buffer.length);
	ret = obj->buffer.length;

out_free_buff:
	kfree(buffer.pointer);
	return ret;
}

static int get_acpi_ssdb_sensor_data(struct device *dev,
				     struct sensor_bios_data *sensor)
{
	struct sensor_bios_data_packed sensor_data;
	int ret;

	ret = read_acpi_block(dev, "SSDB", &sensor_data, sizeof(sensor_data));
	if (ret < 0)
		return ret;

	sensor->link = sensor_data.link;
	sensor->lanes = sensor_data.lanes;
	sensor->mclkspeed = sensor_data.mclkspeed;
	sensor->degree = sensor_data.degree;

	return 0;
}

static int get_acpi_cldb_pmic_data(struct device *dev, struct pmic_cldb *cldb)
{
	int ret;

	ret = read_acpi_block(dev, "CLDB", cldb, sizeof(*cldb));
	if (ret < 0)
		return ret;

	return 0;
}

static int create_fwnode_properties(struct sensor *sensor,
				    struct sensor_bios_data *ssdb)
{
	struct property_entry *cio2_properties = sensor->cio2_properties;
	struct property_entry *dev_properties = sensor->dev_properties;
	struct property_entry *ep_properties = sensor->ep_properties;
	int i;

	/* device fwnode properties */
	memset(dev_properties, 0, sizeof(struct property_entry) * 3);

	dev_properties[0] = PROPERTY_ENTRY_U32("clock-frequency",
					       ssdb->mclkspeed);
	dev_properties[1] = PROPERTY_ENTRY_U8("rotation", ssdb->degree);

	/* endpoint fwnode properties */
	memset(ep_properties, 0, sizeof(struct property_entry) * 4);

	sensor->data_lanes = kmalloc_array(ssdb->lanes, sizeof(u32), GFP_KERNEL);
	if (!sensor->data_lanes)
		return -ENOMEM;

	for (i = 0; i < ssdb->lanes; i++)
		sensor->data_lanes[i] = i + 1;

	ep_properties[0] = PROPERTY_ENTRY_U32("bus-type", 5);
	ep_properties[1] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
							sensor->data_lanes,
							ssdb->lanes);
	ep_properties[2] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_SENSOR];

	/* cio2 endpoint props */
	memset(cio2_properties, 0, sizeof(struct property_entry) * 3);

	cio2_properties[0] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
							  sensor->data_lanes,
							  ssdb->lanes);
	cio2_properties[1] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_CIO2];

	return 0;
}

static int create_connection_swnodes(struct sensor *sensor,
				     struct sensor_bios_data *ssdb)
{
	struct software_node *nodes = sensor->swnodes;

	memset(nodes, 0, sizeof(*nodes) * 6);

	nodes[SWNODE_SENSOR_HID] = NODE_SENSOR(sensor->name,
					       sensor->dev_properties);
	nodes[SWNODE_SENSOR_PORT] = NODE_PORT("port0",
					      &nodes[SWNODE_SENSOR_HID]);
	nodes[SWNODE_SENSOR_ENDPOINT] = NODE_ENDPOINT("endpoint0",
						      &nodes[SWNODE_SENSOR_PORT],
						      sensor->ep_properties);
	nodes[SWNODE_CIO2_PORT] = NODE_PORT(port_names[ssdb->link],
					    &cio2_hid_node);
	nodes[SWNODE_CIO2_ENDPOINT] = NODE_ENDPOINT("endpoint0",
						    &nodes[SWNODE_CIO2_PORT],
						    sensor->cio2_properties);

	return 0;
}

static int cio2_bridge_find_sensor_pmic(struct sensor *sensor)
{
	struct acpi_handle_list dep_handles;
	acpi_handle handle;
	acpi_status status;
	unsigned int i;
	int ret;

	handle = sensor->adev->handle;

	if (!acpi_has_method(handle, "_DEP")) {
		dev_err(sensor->dev, "Sensor has no _DEP method\n");
		return -EINVAL;
	}

	status = acpi_evaluate_reference(handle, "_DEP", NULL, &dep_handles);
	if (ACPI_FAILURE(status)) {
		dev_err(sensor->dev, "Failed to evaluate _DEP reference\n");
		return -EINVAL;
	}

	for (i = 0; i < dep_handles.count; i++) {
		struct acpi_device_info *info;

		status = acpi_get_object_info(dep_handles.handles[i], &info);
		if (ACPI_FAILURE(status)) {
			dev_err(sensor->dev,
				"Failed to get _DEP device information\n");
			return -EINVAL;
		}

		if (info->valid & ACPI_VALID_HID &&
		    !strcmp(info->hardware_id.string, PMIC_HID)) {
			ret = acpi_bus_get_device(dep_handles.handles[i],
						  &sensor->pmic.adev);
			if (ret) {
				dev_err(sensor->dev,
					"Failed to get PMIC ACPI device\n");
				ret = -ENODEV;
			}
			kfree(info);
			break;
		}
		kfree(info);
	}

	if (!sensor->pmic.adev)
		return ret;

	sensor->pmic.dev = bus_find_device_by_acpi_dev(&platform_bus_type,
						   sensor->pmic.adev);

	if (!sensor->pmic.dev) {
		acpi_dev_put(sensor->pmic.adev);
		return -ENODEV;
	}

	INIT_LIST_HEAD(&sensor->pmic.regulators);
	INIT_LIST_HEAD(&sensor->pmic.gpios);

	return 0;
}

static struct cio2_sensor_regulator_map *
cio2_bridge_get_sensor_supply_map(struct sensor *sensor)
{
	struct cio2_sensor_regulator_map *ret;
	union acpi_object *obj;
	unsigned int i;

	obj = acpi_evaluate_dsm_typed(sensor->adev->handle,
				      &sensor_module_guid, 0x00,
				      0x00, NULL, ACPI_TYPE_STRING);

	if (!obj) {
		dev_err(sensor->dev,
			"Failed to get sensor module string from _DSM\n");
		return ERR_PTR(-ENODEV);
	}

	if (obj->string.type != ACPI_TYPE_STRING) {
		dev_err(sensor->dev, "Sensor _DSM returned a non-string value\n");
		ret = ERR_PTR(-EINVAL);
		goto out_free_obj;
	}

	ret = -ENODEV;
	for (i = 0; i < ARRAY_SIZE(cio2_sensor_regulator_maps); i++) {
		if (!strcmp(cio2_sensor_regulator_maps[i].sensor_module_name,
			    obj->string.pointer)) {
			ret = &cio2_sensor_regulator_maps[i];
			goto out_free_obj;
		}
	};

out_free_obj:
	ACPI_FREE(obj);
	return ret;
}

static int cio2_bridge_register_regulator(struct sensor *sensor,
					  struct acpi_resource *ares)
{
	struct cio2_sensor_regulator_map *regulator_map;
	struct regulator_init_data init_data = { };
	struct cio2_gpio_regulator *regulator;
	struct regulator_config cfg = { };
	struct pmic *pmic = &sensor->pmic;
	int ret;

	regulator_map = cio2_bridge_get_sensor_supply_map(sensor);
	if (IS_ERR_OR_NULL(regulator_map)) {
		dev_err(sensor->dev,
			"Found no supplies defined for this sensor\n");
		return PTR_ERR(regulator_map);
	}

	if (pmic->n_regulators >= regulator_map->n_supplies) {
		dev_err(sensor->dev, "All known supplies are already mapped\n");
		return -EINVAL;
	}

	init_data.supply_regulator = NULL;
	init_data.constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS;
	init_data.num_consumer_supplies = 1;
	init_data.consumer_supplies = &regulator_map->supplies[pmic->n_regulators];

	regulator = kmalloc(sizeof(*regulator), GFP_KERNEL);
	if (!regulator)
		return -ENOMEM;

	snprintf(regulator->regulator_name, GPIO_REGULATOR_NAME_LENGTH,
		 "gpio-regulator-%d", pmic->n_regulators);
	snprintf(regulator->supply_name, GPIO_REGULATOR_SUPPLY_NAME_LENGTH,
		 "supply-%d", pmic->n_regulators);

	regulator->rdesc = GPIO_REGULATOR(regulator->regulator_name,
					 regulator->supply_name,
					 pmic->n_regulators,
					 &cio2_bridge_gpio_regulator_ops);

	regulator->gpio = acpi_get_gpiod(ares->data.gpio.resource_source.string_ptr,
					 ares->data.gpio.pin_table[0]);
	if (IS_ERR(regulator->gpio)) {
		ret = PTR_ERR(regulator->gpio);
		goto err_free_regulator;
	}

	cfg.dev = pmic->dev;
	cfg.init_data = &init_data;
	cfg.ena_gpiod = regulator->gpio;
	cfg.driver_data = regulator->gpio;

	regulator->rdev = regulator_register(&regulator->rdesc, &cfg);
	if (IS_ERR(regulator->rdev)) {
		ret = PTR_ERR(regulator->rdev);
		goto err_free_gpio;
	}

	list_add(&regulator->list, &pmic->regulators);
	pmic->n_regulators++;

	return 0;

err_free_gpio:
	gpiod_put(regulator->gpio);
err_free_regulator:
	kfree(regulator);

	return PTR_ERR(regulator->rdev);
}

static int cio2_bridge_map_gpio_to_sensor(struct sensor *sensor, struct acpi_resource *ares,
					  char *func)
{
	struct pmic *pmic = &sensor->pmic;
	struct gpiod_lookup table_entry;
	struct cio2_gpio_pin *gpio;
	struct acpi_device *adev;
	acpi_handle handle;
	acpi_status status;
	int ret;

	gpio = kzalloc(sizeof(*gpio) + sizeof(struct gpiod_lookup) * 2, GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	status = acpi_get_handle(NULL, ares->data.gpio.resource_source.string_ptr, &handle);
	if (ACPI_FAILURE(status))
		return -EINVAL;

	ret = acpi_bus_get_device(handle, &adev);
	if (ret)
		return -ENODEV;

	gpio->lookup->dev_id = dev_name(sensor->dev);
	table_entry = (struct gpiod_lookup) GPIO_LOOKUP_IDX(acpi_dev_name(adev),
							    ares->data.gpio.pin_table[0], func, 0,
							    GPIO_ACTIVE_HIGH);
	memcpy(&gpio->lookup->table[0], &table_entry, sizeof(table_entry));

	gpiod_add_lookup_table(gpio->lookup);
	list_add(&gpio->list, &pmic->gpios);

	return 0;
}

static int cio2_bridge_map_pmic_gpio(struct acpi_resource *ares, void *data)
{
	struct sensor *sensor = data;
	union acpi_object *obj;
	int ret;

	if ((ares->type != ACPI_RESOURCE_TYPE_GPIO) ||
	   (ares->data.gpio.connection_type != ACPI_RESOURCE_GPIO_TYPE_IO))
		return EINVAL; /* Deliberately positive */

	/*
	 * gpio_n + 2 because the index of this _DSM function is 1-based and the
	 * first function is just a count.
	 */
	obj = acpi_evaluate_dsm_typed(sensor->pmic.adev->handle,
				      &pmic_gpio_data_guid, 0x00,
				      sensor->pmic.n_gpios + 2,
				      NULL, ACPI_TYPE_INTEGER);
	if (!obj) {
		dev_warn(sensor->dev, "No _DSM entry for this GPIO pin\n");
		return -ENODEV;
	}

	switch (obj->integer.value & 0xff) { /* low byte holds type information */
	case 0x00: /* Maybe optional regulator? */
		ret = cio2_bridge_map_gpio_to_sensor(sensor, ares, "xshutdn");
		//cio2_bridge_register_regulator(sensor, gpio_n);
		break;
	case 0x01: /* Power regulators */
	case 0x0c:
		ret = cio2_bridge_register_regulator(sensor, ares);
		break;
	case 0x0b: /* Power rail, but to a device separate to sensor */
		ret = cio2_bridge_register_regulator(sensor, ares);
		break;
	case 0x0d: /* Controls power to a privacy LED */
		ret = cio2_bridge_map_gpio_to_sensor(sensor, ares, "indicator-led");
		break;
	default:
		/* if we've gotten here, we're clueless */
		dev_warn(sensor->pmic.dev,
			"GPIO type 0x%02u not known; the sensor may not function\n",
			(obj->integer.value & 0xff));
		ret = -EINVAL;
	}

	sensor->pmic.n_gpios++;

	ACPI_FREE(obj);
	return ret;
}

static int cio2_bridge_parse_pmic_crs(struct sensor *sensor)
{
	struct pmic *pmic = &sensor->pmic;
	struct list_head resource_list;
	int ret = 0;

	INIT_LIST_HEAD(&resource_list);

	ret = acpi_dev_get_resources(pmic->adev, &resource_list,
				     cio2_bridge_map_pmic_gpio, sensor);

	acpi_dev_free_resource_list(&resource_list);

	return ret;
}

static void cio2_bridge_free_gpio_resources(struct sensor *sensor)
{
	struct cio2_gpio_regulator *regulator;
	struct pmic *pmic = &sensor->pmic;
	struct cio2_gpio_pin *gpio;

	list_for_each_entry(regulator, &pmic->regulators, list) {
		regulator_unregister(regulator->rdev);
		gpiod_put(regulator->gpio);
		kfree(regulator);
		pmic->n_regulators--;
	}

	list_for_each_entry(gpio, &pmic->gpios, list) {
		gpiod_remove_lookup_table(gpio->lookup);
		kfree(gpio->lookup);
		kfree(gpio);
		pmic->n_gpios--;
	}
}

static void cio2_bridge_unregister_sensors(void)
{
	struct sensor *sensor;
	int i;

	for (i = 0; i < bridge.n_sensors; i++) {
		sensor = &bridge.sensors[i];

		software_node_unregister_nodes(sensor->swnodes);

		kfree(sensor->data_lanes);

		put_device(sensor->dev);
		acpi_dev_put(sensor->adev);
	}
}

static int connect_supported_devices(struct pci_dev *cio2)
{
	struct sensor_bios_data ssdb;
	struct fwnode_handle *fwnode;
	struct acpi_device *adev;
	struct pmic_cldb cldb;
	struct sensor *sensor;
	char * this_device;
	struct device *dev;
	int i, ret;

	ret = 0;
	for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {
		this_device = supported_devices[i];

		adev = acpi_dev_get_first_match_dev(this_device, NULL, -1);
		if (!adev)
			continue;

		dev = bus_find_device_by_acpi_dev(&i2c_bus_type, adev);
		if (!dev) {
			ret = -EPROBE_DEFER;
			goto err_rollback;
		}

		sensor = &bridge.sensors[bridge.n_sensors];
		sensor->dev = dev;
		sensor->adev = adev;

		snprintf(sensor->name, ACPI_ID_LEN, "%s",
			 this_device);

		ret = get_acpi_ssdb_sensor_data(dev, &ssdb);
		if (ret)
			goto err_free_dev;

		ret = cio2_bridge_find_sensor_pmic(sensor);
		if (ret)
			goto err_free_dev;

		ret = get_acpi_cldb_pmic_data(sensor->pmic.dev, &cldb);
		if (ret)
			goto err_free_pmic;

		/*
		 * If the INT3472 device upon which this sensor is dependent
		 * has type "Discrete" then we have some setup work to do.
		 */
		if (cldb.control_logic_type == 1) {
			pr_info("cio2: parsing _CRS\n");
			ret = cio2_bridge_parse_pmic_crs(sensor);
			if (ret)
				goto err_free_pmic;
		} else {
			pr_info("cio2: control logic not equal to 1\n");
		}

		ret = create_fwnode_properties(sensor, &ssdb);
		if (ret)
			goto err_free_pmic;

		ret = create_connection_swnodes(sensor, &ssdb);
		if (ret)
			goto err_free_pmic;

		ret = software_node_register_nodes(sensor->swnodes);
		if (ret)
			goto err_free_pmic;

		fwnode = software_node_fwnode(&sensor->swnodes[SWNODE_SENSOR_HID]);
		if (!fwnode) {
			ret = -ENODEV;
			goto err_free_swnodes;
		}

		set_secondary_fwnode(dev, fwnode);

		dev_info(&cio2->dev, "Found supported device %s\n",
			 this_device);

		bridge.n_sensors++;
		continue;
	}

	return ret;

err_free_swnodes:
	software_node_unregister_nodes(sensor->swnodes);
err_free_gpio_resources:
	cio2_bridge_free_gpio_resources(sensor);
err_free_pmic:
	acpi_dev_put(sensor->pmic.adev);
	put_device(sensor->pmic.dev);
err_free_dev:
	put_device(dev);
err_rollback:
	acpi_dev_put(adev);

	/*
	 * If an iteration of this loop results in -EPROBE_DEFER then
	 * we need to roll back any sensors that were successfully
	 * registered. Any other error and we'll skip that step, as
	 * it seems better to have one successfully connected sensor.
	 */

	if (ret == -EPROBE_DEFER)
		cio2_bridge_unregister_sensors();

	return ret;
}

int cio2_bridge_init(struct pci_dev *cio2)
{
	struct device *dev = &cio2->dev;	
	struct fwnode_handle *fwnode;
	int ret;

	pci_dev_get(cio2);

	ret = software_node_register(&cio2_hid_node);
	if (ret < 0) {
		dev_err(dev, "Failed to register the CIO2 HID node\n");
		goto err_put_cio2;
	}

	ret = connect_supported_devices(cio2);
	if (ret == -EPROBE_DEFER)
		goto err_unregister_cio2;

	if (bridge.n_sensors == 0) {
		ret = -EPROBE_DEFER;
		goto err_unregister_cio2;
	}

	dev_info(dev, "Connected %d cameras\n", bridge.n_sensors);

	fwnode = software_node_fwnode(&cio2_hid_node);
	if (!fwnode) {
		dev_err(dev, "Error getting fwnode from cio2 software_node\n");
		ret = -ENODEV;
		goto err_unregister_sensors;
	}

	set_secondary_fwnode(dev, fwnode);

	return 0;

err_unregister_sensors:
	cio2_bridge_unregister_sensors();
err_unregister_cio2:
	software_node_unregister(&cio2_hid_node);
err_put_cio2:
	pci_dev_put(cio2);

	return ret;
}

void cio2_bridge_clean(struct pci_dev *cio2)
{
	pci_dev_put(cio2);

	cio2_bridge_unregister_sensors();

	software_node_unregister(&cio2_hid_node);
}
