"""Implementation of I/O Board Light device"""
import asyncio
import logging

from homeassistant.components.rpi_ioboard import OneRelay
from homeassistant.components.light import Light

DEPENDENCIES = ['rpi_ioboard']

_LOGGER = logging.getLogger(__name__)

@asyncio.coroutine
def async_setup_platform(hass, config, async_add_devices, discovery_info=None): # pylint: disable=W0613
    """Setup of I/O Board Light device."""

#    _LOGGER.debug("Setup of I/O Board Light device")
    if discovery_info is None:
        return

    async_add_devices([IOBoardLight(hass, discovery_info)], True)
    return True

class IOBoardLight(OneRelay, Light):
    """Implementation of Light device class"""
