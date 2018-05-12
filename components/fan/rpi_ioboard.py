"""Implementation of I/O Board Fan device"""
import asyncio
import logging

from homeassistant.components.rpi_ioboard import OneRelay
from homeassistant.components.fan import (SPEED_HIGH, FanEntity, SUPPORT_SET_SPEED)
from homeassistant.const import STATE_OFF

DEPENDENCIES = ['rpi_ioboard']

_LOGGER = logging.getLogger(__name__)

@asyncio.coroutine
def async_setup_platform(hass, config, async_add_devices, discovery_info=None): # pylint: disable=W0613
    """Setup of I/O Board Fan device."""

#    _LOGGER.debug("Setup of I/O Board Fan device")
    if discovery_info is None:
        return

    async_add_devices([IOBoardFan(hass, discovery_info)], True)
    return True

class IOBoardFan(OneRelay, FanEntity):
    """Implementation of Fan device class"""

    @property
    def speed(self) -> str:
        """Return the current speed."""
        if self.is_on:
            return SPEED_HIGH
        else:
            return STATE_OFF

    @property
    def speed_list(self) -> list:
        """Get the list of available speeds."""
        return [STATE_OFF, SPEED_HIGH]

    def turn_on(self, speed: str=None) -> None:
        """Turn on the entity."""
        super().turn_on()

    def turn_off(self) -> None:
        """Turn off the entity."""
        super().turn_off()

    def set_speed(self, speed: str) -> None:
        """Set the speed of the fan."""
        if speed == SPEED_HIGH:
            self.turn_on()
        else:
            self.turn_off()

    @property
    def supported_features(self) -> int:
        """Flag supported features."""
        return SUPPORT_SET_SPEED
