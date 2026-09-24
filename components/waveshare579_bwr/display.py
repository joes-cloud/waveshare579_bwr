import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import display, spi
from esphome.const import (
    CONF_BUSY_PIN,
    CONF_DC_PIN,
    CONF_ID,
    CONF_LAMBDA,
    CONF_RESET_PIN,
    CONF_ROTATION,
    CONF_UPDATE_INTERVAL,
)

DEPENDENCIES = ["spi"]
AUTO_LOAD = ["display"]

waveshare579_bwr_ns = cg.esphome_ns.namespace("waveshare579_bwr")
Waveshare579BWR = waveshare579_bwr_ns.class_(
    "Waveshare579BWR",
    display.DisplayBuffer,
    spi.SPIDevice,
    cg.PollingComponent,
)

CONFIG_SCHEMA = cv.All(
    display.FULL_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(Waveshare579BWR),
            cv.Required(CONF_DC_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_BUSY_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
        }
    ).extend(spi.spi_device_schema(cs_pin_required=True)),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await display.register_display(var, config)
    await spi.register_spi_device(var, config)

    if CONF_LAMBDA in config:
        writer = await cg.process_lambda(
            config[CONF_LAMBDA],
            [(display.Display.operator("ref"), "it")],
            return_type=cg.void,
        )
        cg.add(var.set_writer(writer))

    if CONF_ROTATION in config:
        cg.add(var.set_rotation(config[CONF_ROTATION]))

    dc_pin = await cg.gpio_pin_expression(config[CONF_DC_PIN])
    cg.add(var.set_dc_pin(dc_pin))
    busy_pin = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    cg.add(var.set_busy_pin(busy_pin))
    reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(reset_pin))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

