// SPDX-License-Identifier: GPL-2.0
/* Author: Dan Scally <djrscally@gmail.com> */
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/property.h>
#include <linux/regulator/driver.h>
#include <media/v4l2-subdev.h>

#include "cio2-bridge.h"

/*
 * Extend this array with ACPI Hardware ID's of devices known to be working.
 * Do not add a HID for a sensor that is not actually supported.
 */
static const char * const cio2_supported_devices[] = {
	"INT33BE",
	"OVTI2680",
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

static int cio2_bridge_read_acpi_buffer(struct acpi_device *adev, char *id,
					void *data, u32 size)
{
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *obj;
	acpi_status status;
	int ret;

	status = acpi_evaluate_object(adev->handle, id, NULL, &buffer);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	obj = buffer.pointer;
	if (!obj) {
		dev_err(&adev->dev, "Couldn't locate ACPI buffer\n");
		return -ENODEV;
	}

	if (obj->type != ACPI_TYPE_BUFFER) {
		dev_err(&adev->dev, "Not an ACPI buffer\n");
		ret = -ENODEV;
		goto out_free_buff;
	}

	if (obj->buffer.length > size) {
		dev_err(&adev->dev, "Given buffer is too small\n");
		ret = -EINVAL;
		goto out_free_buff;
	}

	memcpy(data, obj->buffer.pointer, obj->buffer.length);
	ret = obj->buffer.length;

out_free_buff:
	kfree(buffer.pointer);
	return ret;
}

static void cio2_bridge_init_property_names(struct cio2_sensor *sensor)
{
        strscpy(sensor->prop_names.clock_frequency, "clock-frequency", 16);
        strscpy(sensor->prop_names.rotation, "rotation", 16);
        strscpy(sensor->prop_names.bus_type, "bus-type", 16);
        strscpy(sensor->prop_names.data_lanes, "data-lanes", 16);
        strscpy(sensor->prop_names.remote_endpoint, "remote-endpoint", 16);
}

static void cio2_bridge_create_fwnode_properties(struct cio2_sensor *sensor)
{
       unsigned int i;

       cio2_bridge_init_property_names(sensor);

       for (i = 0; i < 4; i++) {
               sensor->data_lanes[i] = i+1;
       }

        /*
         * Can't use PROPERTY_ENTRY_REF because it creates a new variable to
         * point to, which doesn't survive the function.
         */
        sensor->local_ref[0] = (struct software_node_ref_args){
                .node = &sensor->swnodes[SWNODE_CIO2_ENDPOINT]
                };
        sensor->remote_ref[0] = (struct software_node_ref_args){
                .node = &sensor->swnodes[SWNODE_SENSOR_ENDPOINT]
                };

	sensor->dev_properties[0] = PROPERTY_ENTRY_U32(
                                        sensor->prop_names.clock_frequency,
                                        sensor->ssdb.mclkspeed);
	sensor->dev_properties[1] = PROPERTY_ENTRY_U8(
                                        sensor->prop_names.rotation,
                                        sensor->ssdb.degree);

	sensor->ep_properties[0] = PROPERTY_ENTRY_U32(
                                        sensor->prop_names.bus_type, 5);
	sensor->ep_properties[1] = PROPERTY_ENTRY_U32_ARRAY_LEN(
                                        sensor->prop_names.data_lanes,
                                        sensor->data_lanes,
                                        sensor->ssdb.lanes);
	sensor->ep_properties[2] = PROPERTY_ENTRY_REF_ARRAY(
                                        sensor->prop_names.remote_endpoint,
                                        sensor->local_ref);

	sensor->cio2_properties[0] = PROPERTY_ENTRY_U32_ARRAY_LEN(
                                        sensor->prop_names.data_lanes,
                                        sensor->data_lanes,
                                        sensor->ssdb.lanes);
	sensor->cio2_properties[1] = PROPERTY_ENTRY_REF_ARRAY(
                                        sensor->prop_names.remote_endpoint,
                                        sensor->remote_ref);
}

static void cio2_bridge_init_swnode_names(struct cio2_sensor *sensor)
{
        snprintf(sensor->node_names.remote_port, 10, "port%u", sensor->ssdb.link);
        strscpy(sensor->node_names.port, "port0", 10);
        strscpy(sensor->node_names.endpoint, "endpoint0", 10);
}

static void cio2_bridge_create_connection_swnodes(struct cio2_bridge *bridge, struct cio2_sensor *sensor)
{
	struct software_node *nodes = sensor->swnodes;

        cio2_bridge_init_swnode_names(sensor);

	nodes[SWNODE_SENSOR_HID] = NODE_SENSOR(sensor->name,
					       sensor->dev_properties);
	nodes[SWNODE_SENSOR_PORT] = NODE_PORT(sensor->node_names.port,
					      &nodes[SWNODE_SENSOR_HID]);
	nodes[SWNODE_SENSOR_ENDPOINT] = NODE_ENDPOINT(sensor->node_names.endpoint,
						      &nodes[SWNODE_SENSOR_PORT],
						      sensor->ep_properties);
	nodes[SWNODE_CIO2_PORT] = NODE_PORT(
                                        sensor->node_names.remote_port,
					&bridge->cio2_hid_node);
	nodes[SWNODE_CIO2_ENDPOINT] = NODE_ENDPOINT(
                                        sensor->node_names.endpoint,
					&nodes[SWNODE_CIO2_PORT],
					sensor->cio2_properties);
}

static void cio2_bridge_unregister_sensors(struct cio2_bridge *bridge)
{
	struct cio2_sensor *sensor;
	unsigned int i;

	for (i = 0; i < bridge->n_sensors; i++) {
		sensor = &bridge->sensors[i];
		software_node_unregister_nodes(sensor->swnodes);
		acpi_dev_put(sensor->adev);
	}
}

static int cio2_bridge_find_sensor_pmic(struct cio2_sensor *sensor)
{
	struct acpi_handle_list dep_handles;
	acpi_handle handle;
	acpi_status status;
	unsigned int i;
	int ret;

	handle = sensor->adev->handle;

	if (!acpi_has_method(handle, "_DEP")) {
		dev_err(&sensor->adev->dev, "Sensor has no _DEP method\n");
		return -EINVAL;
	}

	status = acpi_evaluate_reference(handle, "_DEP", NULL, &dep_handles);
	if (ACPI_FAILURE(status)) {
		dev_err(&sensor->adev->dev, "Failed to evaluate _DEP reference\n");
		return -EINVAL;
	}

	for (i = 0; i < dep_handles.count; i++) {
		struct acpi_device_info *info;

		status = acpi_get_object_info(dep_handles.handles[i], &info);
		if (ACPI_FAILURE(status)) {
			dev_err(&sensor->adev->dev,
				"Failed to get _DEP device information\n");
			return -EINVAL;
		}

		if (info->valid & ACPI_VALID_HID &&
		    !strcmp(info->hardware_id.string, CIO2_PMIC_HID)) {
			ret = acpi_bus_get_device(dep_handles.handles[i],
						  &sensor->pmic.adev);
			if (ret) {
				dev_err(&sensor->adev->dev,
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

	INIT_LIST_HEAD(&sensor->pmic.regulators);
	INIT_LIST_HEAD(&sensor->pmic.gpios);

	return 0;
}

static int cio2_bridge_map_gpio_to_sensor(struct cio2_sensor *sensor,
                                          struct acpi_resource *ares, char *func)
{
	char *path = ares->data.gpio.resource_source.string_ptr;
        struct gpiod_lookup table_entry;
	struct acpi_device *adev;
	acpi_handle handle;
	acpi_status status;
	int ret;

        if (!sensor->gpios) {
                pr_info("Creating GPIOS table %s\n", i2c_acpi_dev_name(sensor->adev));
                sensor->gpios = kzalloc(sizeof(struct gpiod_lookup_table) + (4 * sizeof(struct gpiod_lookup)), GFP_KERNEL);
                sensor->gpios->dev_id = i2c_acpi_dev_name(sensor->adev);
        }

        /* Fetch ACPI handle for the GPIO chip */
	status = acpi_get_handle(NULL, path, &handle);
	if (ACPI_FAILURE(status))
		return -EINVAL;

	ret = acpi_bus_get_device(handle, &adev);
	if (ret)
		return -ENODEV;

	table_entry = (struct gpiod_lookup) GPIO_LOOKUP_IDX(acpi_dev_name(adev),
							    ares->data.gpio.pin_table[0], func, 0,
							    GPIO_ACTIVE_HIGH);

	memcpy(&sensor->gpios->table[sensor->n_gpios], &table_entry, sizeof(table_entry));
        sensor->n_gpios++;

	return 0;
}

static struct cio2_sensor_regulator_map *
cio2_bridge_get_sensor_supply_map(struct cio2_sensor *sensor)
{
	struct cio2_sensor_regulator_map *ret;
	union acpi_object *obj;
	unsigned int i;

	obj = acpi_evaluate_dsm_typed(sensor->adev->handle,
				      &cio2_sensor_module_guid, 0x00,
				      0x01, NULL, ACPI_TYPE_STRING);

	if (!obj) {
		dev_err(&sensor->adev->dev,
			"Failed to get sensor module string from _DSM\n");
		return ERR_PTR(-ENODEV);
	}

	if (obj->string.type != ACPI_TYPE_STRING) {
		dev_err(&sensor->adev->dev,
                        "Sensor _DSM returned a non-string value\n");
		ret = ERR_PTR(-EINVAL);
		goto out_free_obj;
	}
        pr_info("Found sensor module string: %s\n", obj->string.pointer);

	ret = ERR_PTR(-ENODEV);
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

static int cio2_bridge_register_regulator(struct cio2_sensor *sensor,
					  struct acpi_resource *ares)
{
        char *path = ares->data.gpio.resource_source.string_ptr;
	struct cio2_sensor_regulator_map *regulator_map;
	struct regulator_init_data init_data = { };
	struct cio2_gpio_regulator *regulator;
	struct regulator_config cfg = { };
	int ret;

	regulator_map = cio2_bridge_get_sensor_supply_map(sensor);
	if (IS_ERR_OR_NULL(regulator_map)) {
		dev_err(&sensor->adev->dev,
			"Found no supplies defined for this sensor\n");
		return PTR_ERR(regulator_map);
	}

	if (sensor->pmic.n_regulators >= regulator_map->n_supplies) {
		dev_err(&sensor->adev->dev, "All known supplies are already mapped\n");
		return -EINVAL;
	}

	init_data.supply_regulator = NULL;
	init_data.constraints.valid_ops_mask = REGULATOR_CHANGE_STATUS;
	init_data.num_consumer_supplies = 1;
	init_data.consumer_supplies = &regulator_map->supplies[sensor->pmic.n_regulators];

	regulator = kmalloc(sizeof(*regulator), GFP_KERNEL);
	if (!regulator)
		return -ENOMEM;

	snprintf(regulator->regulator_name, GPIO_REGULATOR_NAME_LENGTH,
		 "gpio-regulator-%d", sensor->pmic.n_regulators);
	snprintf(regulator->supply_name, GPIO_REGULATOR_SUPPLY_NAME_LENGTH,
		 "supply-%d", sensor->pmic.n_regulators);

	regulator->rdesc = GPIO_REGULATOR(regulator->regulator_name,
					 regulator->supply_name,
					 sensor->pmic.n_regulators,
					 &cio2_bridge_gpio_regulator_ops);

	regulator->gpio = acpi_get_gpiod(path, ares->data.gpio.pin_table[0]);
	if (IS_ERR(regulator->gpio)) {
		ret = PTR_ERR(regulator->gpio);
		goto err_free_regulator;
	}

	cfg.dev = &sensor->pmic.adev->dev;
	cfg.init_data = &init_data;
	cfg.ena_gpiod = regulator->gpio;
	cfg.driver_data = regulator->gpio;

	regulator->rdev = regulator_register(&regulator->rdesc, &cfg);
	if (IS_ERR(regulator->rdev)) {
		ret = PTR_ERR(regulator->rdev);
		goto err_free_gpio;
	}

	list_add(&regulator->list, &sensor->pmic.regulators);
	sensor->pmic.n_regulators++;

	return 0;

err_free_gpio:
	gpiod_put(regulator->gpio);
err_free_regulator:
	kfree(regulator);

	return PTR_ERR(regulator->rdev);
}

static int cio2_bridge_handle_pmic_gpio(struct acpi_resource *ares, void *data)
{
	struct cio2_sensor *sensor = data;
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
				      &cio2_pmic_gpio_guid, 0x00,
				      sensor->pmic.n_gpios + 2,
				      NULL, ACPI_TYPE_INTEGER);

        pr_info("Found GPIO with DSM data 0x%04x\n", obj->integer.value);

	if (!obj) {
		dev_warn(&sensor->adev->dev, "No _DSM entry for this GPIO pin\n");
		return -ENODEV;
	}

	switch (obj->integer.value & 0xff) { /* low byte holds type information */
	case 0x00: /* Purpose unclear, possibly a reset GPIO pin */
		ret = cio2_bridge_map_gpio_to_sensor(sensor, ares, "reset");
		break;
	case 0x01: /* Power regulators (we think) for the main sensor */
	case 0x0c:
		ret = cio2_bridge_register_regulator(sensor, ares);
		break;
	case 0x0b: /* Power regulators, but to a device separate to sensor */
		ret = cio2_bridge_register_regulator(sensor, ares);
		break;
	case 0x0d: /* Indicator LEDs */
		ret = cio2_bridge_map_gpio_to_sensor(sensor, ares, "indicator-led");
		break;
	default:
		/* if we've gotten here, we're clueless */
		dev_warn(&sensor->pmic.adev->dev,
			"GPIO type 0x%02llu not known; the sensor may not function\n",
			(obj->integer.value & 0xff));
		ret = EINVAL;
	}

	sensor->pmic.n_gpios++;

	ACPI_FREE(obj);
	return ret;
}

static int cio2_bridge_parse_pmic_crs(struct cio2_sensor *sensor)
{
	struct list_head resource_list;
	int ret = 0;

	INIT_LIST_HEAD(&resource_list);

	ret = acpi_dev_get_resources(sensor->pmic.adev, &resource_list,
				     cio2_bridge_handle_pmic_gpio, sensor);

	acpi_dev_free_resource_list(&resource_list);

        gpiod_add_lookup_table(sensor->gpios);

	return ret;
}

static int cio2_bridge_connect_sensors(struct cio2_bridge *bridge)
{
	struct fwnode_handle *fwnode;
	struct cio2_sensor *sensor;
	struct acpi_device *adev;
        unsigned int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(cio2_supported_devices); i++) {
		const char *this_device = cio2_supported_devices[i];

		for_each_acpi_dev_match(adev, this_device, NULL, -1) {
			if (!adev || !(adev->status.present && adev->status.enabled))
				continue;

			sensor = &bridge->sensors[bridge->n_sensors];
			sensor->adev = adev;
                        strscpy(sensor->name, this_device, sizeof(sensor->name));

			ret = cio2_bridge_read_acpi_buffer(adev, "SSDB",
                                                           &sensor->ssdb,
                                                           sizeof(sensor->ssdb));
			if (ret < 0)
				goto err_put_adev;

			if (sensor->ssdb.lanes > 4) {
				dev_err(&adev->dev, "Number of lanes in SSDB is invalid\n");
				goto err_put_adev;
			}

                        ret = cio2_bridge_find_sensor_pmic(sensor);
                        if (ret)
                                goto err_put_adev;

                        ret = cio2_bridge_read_acpi_buffer(sensor->pmic.adev, "CLDB", &sensor->pmic.cldb, sizeof(sensor->pmic.cldb));
                        if (ret < 0)
                                goto err_put_pmic;

                        /*
                         * If the INT3472 device upon which this sensor depends
                         * has type "Discrete" then we have some setup work to
                         * do.
                         */
                        if (sensor->pmic.cldb.control_logic_type == 1) {
                                ret = cio2_bridge_parse_pmic_crs(sensor);
                                if (ret)
                                        goto err_put_pmic;
                        }

			cio2_bridge_create_fwnode_properties(sensor);
			cio2_bridge_create_connection_swnodes(bridge, sensor);

			ret = software_node_register_nodes(sensor->swnodes);
			if (ret)
				goto err_put_adev;

			fwnode = software_node_fwnode(&sensor->swnodes[SWNODE_SENSOR_HID]);
			if (!fwnode) {
				ret = -ENODEV;
				goto err_free_swnodes;
			}

			adev->fwnode.secondary = fwnode;

			dev_info(&bridge->cio2->dev,
                                 "Found supported sensor %s\n",
                                 acpi_dev_name(adev));

			bridge->n_sensors++;
		}
	}

	return ret;

err_free_swnodes:
	software_node_unregister_nodes(sensor->swnodes);
err_put_pmic:
        acpi_dev_put(sensor->pmic.adev);
err_put_adev:
        acpi_dev_put(sensor->adev);

	return ret;
}

int cio2_bridge_init(struct pci_dev *cio2)
{
	struct device *dev = &cio2->dev;
	struct fwnode_handle *fwnode;
        struct cio2_bridge *bridge;
	int ret;

        bridge = kzalloc(sizeof(*bridge), GFP_KERNEL);
        if (!bridge)
                return -ENOMEM;

        strscpy(bridge->cio2_node_name, CIO2_HID, sizeof(bridge->cio2_node_name));
        bridge->cio2_hid_node = (const struct software_node) { bridge->cio2_node_name };
	bridge->cio2 = pci_dev_get(cio2);

	ret = software_node_register(&bridge->cio2_hid_node);
	if (ret < 0) {
		dev_err(dev, "Failed to register the CIO2 HID node\n");
		goto err_put_cio2;
	}

	ret = cio2_bridge_connect_sensors(bridge);
	if (ret || bridge->n_sensors == 0)
		goto err_unregister_cio2;

	dev_info(dev, "Connected %d cameras\n", bridge->n_sensors);

	fwnode = software_node_fwnode(&bridge->cio2_hid_node);
	if (!fwnode) {
		dev_err(dev, "Error getting fwnode from cio2 software_node\n");
		ret = -ENODEV;
		goto err_unregister_sensors;
	}

	set_secondary_fwnode(dev, fwnode);

	return 0;

err_unregister_sensors:
	cio2_bridge_unregister_sensors(bridge);
err_unregister_cio2:
	software_node_unregister(&bridge->cio2_hid_node);
err_put_cio2:
	pci_dev_put(bridge->cio2);

        kfree(bridge);
	return ret;
}
