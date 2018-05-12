"""Implementation of I/O Board Switch device"""
import asyncio
import logging

from homeassistant.components.rpi_ioboard import OneRelay
from homeassistant.components.switch import SwitchDevice

DEPENDENCIES = ['rpi_ioboard']

_LOGGER = logging.getLogger(__name__)

@asyncio.coroutine
def async_setup_platform(hass, config, async_add_devices, discovery_info=None): # pylint: disable=W0613
    """Setup of I/O Board Switch device."""

#    _LOGGER.debug("Setup of I/O Board Switch device")
    if discovery_info is None:
        return

    async_add_devices([IOBoardSwitch(hass, discovery_info)], True)
    return True

class IOBoardSwitch(OneRelay, SwitchDevice):
    """Implementation of Switch device class"""
