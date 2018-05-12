"""Implementation of I/O Board Binary Sensor device"""
import asyncio
import logging

from homeassistant.components.rpi_ioboard import OneButton
from homeassistant.components.binary_sensor import BinarySensorDevice

DEPENDENCIES = ['rpi_ioboard']

_LOGGER = logging.getLogger(__name__)

@asyncio.coroutine
def async_setup_platform(hass, config, async_add_devices, discovery_info=None): # pylint: disable=W0613
    """Setup of I/O Board Binary Sensor device."""

#    _LOGGER.info("Setup of I/O Board Binary Sensor device")
    if discovery_info is None:
        return False
    async_add_devices([IOBoardBinarySensor(hass, discovery_info)], True)
    return True

class IOBoardBinarySensor(OneButton, BinarySensorDevice):
    """Implementation of Binary Sensor device"""
