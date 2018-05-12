"""
Implementation of I/O Board base suppert
"""

import logging
import voluptuous as vol

from homeassistant.const import (CONF_NAME, ATTR_ENTITY_ID, EVENT_HOMEASSISTANT_START, EVENT_HOMEASSISTANT_STOP)
from homeassistant.exceptions import HomeAssistantError
import homeassistant.helpers.config_validation as cv
from homeassistant.helpers.discovery import load_platform
from homeassistant.helpers.entity import Entity, ToggleEntity

REQUIREMENTS = ['i2csense==0.0.3',
                'smbus-cffi==0.5.1',
                'RPi.GPIO==0.6.1']

_LOGGER = logging.getLogger(__name__)

CONF_I2C_BUS = 'i2c_bus'
CONF_BOARDS = 'boards'
CONF_I2C_ADDRESS = 'i2c_address'
CONF_INT_PIN = 'int_pin'
CONF_INPUTS = 'inputs'
CONF_OUTPUTS = 'outputs'
CONF_TYPE = 'type'
CONF_ACTIONS = 'actions'
CONF_PIN = 'pin'
CONF_IOBOARD = 'ioboard'
CONF_BINARY_SENSOR = 'binary_sensor'

DEFAULT_I2C_BUS = 1
DEFAULT_I2C_ADDRESS = 0x20
DEFAULT_OUTPUT_TYPE = 'light'
DEFAULT_INPUT_TYPE = 'button'

EVENT_NAME = 'ioboard_event'
EVENT_CLICK_ACTION = 'click_action'

CLICK_ACTION_DOWN = 'down'
CLICK_ACTION_UP = 'up'
CLICK_ACTION_SINGLE = 'single'
CLICK_ACTION_DOUBLE = 'double'
CLICK_ACTION_HOLD = 'hold'
BUTTON_CLICK_ACTIONS = [CLICK_ACTION_SINGLE, CLICK_ACTION_HOLD]
BINARY_CLICK_ACTIONS = [CLICK_ACTION_DOWN, CLICK_ACTION_UP]
ALL_CLICK_ACTIONS = [CLICK_ACTION_DOWN, CLICK_ACTION_UP, CLICK_ACTION_SINGLE,
                     CLICK_ACTION_DOUBLE, CLICK_ACTION_HOLD]

DOMAIN = 'rpi_ioboard'

INPUT_TYPES = {'button': BUTTON_CLICK_ACTIONS,
               'power': BINARY_CLICK_ACTIONS,
               'gas': BINARY_CLICK_ACTIONS,
               'motion': BINARY_CLICK_ACTIONS,
               'moisture': BINARY_CLICK_ACTIONS,
               'alarm': BINARY_CLICK_ACTIONS,}

OUTPUT_TYPES = {'light': ['light', 'light'],
                'switch': ['switch', 'switch'],
                'fan': ['fan', 'fan'],}

OUTPUT_SCHEMA = vol.Schema({
    vol.Required(CONF_NAME): cv.string,
    vol.Optional(CONF_TYPE, default=DEFAULT_OUTPUT_TYPE):
        vol.In(OUTPUT_TYPES),
})

INPUT_SCHEMA = vol.Schema({
    vol.Required(CONF_NAME): cv.string,
    vol.Optional(CONF_TYPE, default=DEFAULT_INPUT_TYPE):
        vol.In(INPUT_TYPES),
    vol.Optional(CONF_ACTIONS):
        vol.All(cv.ensure_list, [vol.In(ALL_CLICK_ACTIONS)]),
})

BOARD_SCHEMA = vol.Schema({
    vol.Required(CONF_I2C_ADDRESS): vol.Coerce(int),
    vol.Required(CONF_INT_PIN): vol.Coerce(int),
    vol.Required(CONF_INPUTS, default={}):
        vol.Schema({cv.positive_int: INPUT_SCHEMA}),
    vol.Required(CONF_OUTPUTS, default={}):
        vol.Schema({cv.positive_int: OUTPUT_SCHEMA}),
})

CONFIG_SCHEMA = vol.Schema({
    DOMAIN: vol.Schema({
        vol.Optional(CONF_I2C_BUS, default=DEFAULT_I2C_BUS): vol.In([0, 1]),
        vol.Required(CONF_BOARDS, default=[]): vol.All(
            cv.ensure_list, [BOARD_SCHEMA]),
    }),
}, extra=vol.ALLOW_EXTRA)

def setup(hass, config, session=None): # pylint: disable=W0613
    """Set up for the I/O board devices."""
    import RPi.GPIO as GPIO
    import threading
    import smbus

    def cleanup_gpio(event):
        """Stuff to do before stopping."""
        GPIO.cleanup()

    def prepare_gpio(event):
        """Stuff to do when home assistant starts."""
        hass.bus.listen_once(EVENT_HOMEASSISTANT_STOP, cleanup_gpio)

    conf = config.get(DOMAIN)
    i2c_bus = conf.get(CONF_I2C_BUS)
    smbus = smbus.SMBus(i2c_bus)
    if smbus is None:
        raise HomeAssistantError("i2c bus could not be opened")

    hass.bus.listen_once(EVENT_HOMEASSISTANT_START, prepare_gpio)
    GPIO.setmode(GPIO.BCM)
    boards = conf.get(CONF_BOARDS)
    lock = threading.RLock()

    for board in boards:
        IOBoard(hass, smbus, board, lock)
    return True


class OneButton(Entity):
    """IOPin class"""

    DEBOUNCE_TIME = 0.050
    HOLD_TIME = 1.0
    CLICK_TIME = 0.6

    BUTTON_RELEASED = 0

    _pressed_time = None
    _released_time = None
    _timer = None
    _state = 0
    _click_actions = []
    _pin_mask = 0

    def __init__(self, hass, config):
        """Initialize the IOPin."""
        self.hass = hass
        self._sensor_type = config[CONF_TYPE]
        self._name = config[CONF_NAME]
        self._pin = config[CONF_PIN]
        self._click_actions = config[CONF_ACTIONS]
        ioboard_name = config[CONF_IOBOARD]
        self._ioboard = hass.data.get(ioboard_name)
        if self._ioboard is None:
            raise HomeAssistantError("I/O Board {0} not registered".format(ioboard_name))
        self._pin_mask = 1 << (self._pin - 1)
        self._ioboard.add_input_pin(self)
        self.set_value(self._ioboard.old_port)
        self._negative = self._sensor_type == 'alarm'

    @property
    def is_on(self):
        """Return if device is on"""
#        _LOGGER.critical("OneButton %s is_on %s", self.name, self._state in [1, 3])
        result = self._state in [1, 3]
        if self._negative:
            return not result
        else:
            return result

    @property
    def device_class(self):
        """Return device type"""
        return self._sensor_type

    @property
    def name(self):
        """Return the name of the device."""
        return self._name #'input_{}_{}'.format(self._ioboard.board_id + 1, self._pin)

    @property
    def should_poll(self):
        """No polling needed."""
        return False

    @property
    def pin_mask(self):
        """Pin mask"""
        return self._pin_mask

    def _fire_event(self, click_action):
        """Fire event"""
        #import time
        if click_action in [CLICK_ACTION_DOWN, CLICK_ACTION_UP]:
#            _LOGGER.critical("OneButton %s state changed %s", self.name, self._state)
            self.schedule_update_ha_state() #does not work -> so this is hack
#            _LOGGER.critical("OneButton 2 %s state changed %s", self.name, self._state)
#            self.hass.states.async_set(self.entity_id, state, {}, False)
        else:
#            _LOGGER.critical("OneButton %s fire event %s {%s}", self.name,
#                          click_action, self.entity_id)
            self.hass.bus.fire(EVENT_NAME, {
                ATTR_ENTITY_ID: self.entity_id,
                EVENT_CLICK_ACTION: click_action
            })

    def _timer_func(self, click_action, expected_state):
        """Timer event"""
#        _LOGGER.debug("OneButton %s timer event %d == %d action %s", self.name,
#                      self._state, expected_state, click_action)
        if self._state == expected_state:
            self._state = 0
            self._fire_event(click_action)

    def _waiting_for_first_press(self, now, pin_level):
        """State machine for state 0"""
        import threading

        if pin_level != self.BUTTON_RELEASED: #waiting for button being pressed.
            self._pressed_time = now
#            _LOGGER.critical("OneButton %s pressed, time=%s", self.name, now)
            if (now - self._released_time) > self.DEBOUNCE_TIME:
                self._state = 1
                if CLICK_ACTION_DOWN in self._click_actions:
                    self._fire_event(CLICK_ACTION_DOWN)
                if CLICK_ACTION_HOLD in self._click_actions:
                    self._timer = threading.Timer(self.HOLD_TIME, self._timer_func,
                                                  [CLICK_ACTION_HOLD, 1])
                    self._timer.start()

    def _waiting_for_first_release(self, now, pin_level):
        """State machine for state 1"""
        import threading

        if pin_level == self.BUTTON_RELEASED: #waiting for button being released.
#            _LOGGER.critical("OneButton %s released, time=%s", self.name, now)
            if CLICK_ACTION_HOLD in self._click_actions and self._timer:
                self._timer.cancel()
            self._released_time = now
            if (now - self._pressed_time) > self.DEBOUNCE_TIME:
                self._state = 0
                if CLICK_ACTION_UP in self._click_actions:
                    self._fire_event(CLICK_ACTION_UP)
                if CLICK_ACTION_DOUBLE in self._click_actions:
                    self._state = 2
                    self._timer = threading.Timer(self.CLICK_TIME - (now - self._pressed_time),
                                                  self._timer_func, [CLICK_ACTION_SINGLE, 2])
                    self._timer.start()
                else:
                    if CLICK_ACTION_SINGLE in self._click_actions:
                        self._fire_event(CLICK_ACTION_SINGLE)
            else:
                _LOGGER.debug('debounce state=1')
                self._state = 0

    def _waiting_for_second_press(self, now, pin_level):
        """"""
        if pin_level != self.BUTTON_RELEASED:
            self._timer.cancel()
            if (now - self._released_time) > self.DEBOUNCE_TIME:
                self._state = 3
                self._pressed_time = now
                if CLICK_ACTION_DOWN in self._click_actions:
                    self._fire_event(CLICK_ACTION_DOWN)
            else:
                _LOGGER.debug('debounce state=2')
                self._state = 0

    def _waiting_for_second_release(self, now, pin_level):
        """"""
        if pin_level == self.BUTTON_RELEASED:
            self._state = 0
            if  (now - self._pressed_time) > self.DEBOUNCE_TIME:
                if CLICK_ACTION_UP in self._click_actions:
                    self._fire_event(CLICK_ACTION_UP)
                self._fire_event(CLICK_ACTION_DOUBLE)
            else:
                _LOGGER.debug('debounce state=3')

    def on_pin_change(self, value):
        """Callback when pin changed"""
        import time

        now = time.time()
        pin_level = value & self._pin_mask
#        _LOGGER.critical("OneButton %s on_pin_change, value=%s, pin_mask=%s, pin_level=%s, _state=%s, time=%s", self.name, value, self._pin_mask, pin_level, self._state, now)
        if self._state == 0:
            self._waiting_for_first_press(now, pin_level)
        elif self._state == 1:
            self._waiting_for_first_release(now, pin_level)
        elif self._state == 2:
            self._waiting_for_second_press(now, pin_level)
        elif self._state == 3:
            self._waiting_for_second_release(now, pin_level)

    def set_value(self, value):
        """Set pin value after create"""
        import time

        pin_level = value & self._pin_mask
        if pin_level != self.BUTTON_RELEASED and CLICK_ACTION_UP in self._click_actions:
            self._state = 1
            self._pressed_time = time.time()
        else:
            self._state = 0
            self._released_time = time.time()


class OneRelay(ToggleEntity):
    """Implementation of relay device"""

    def __init__(self, hass, config):
        """Initialize the relay device"""
#        _LOGGER.debug("OneRelay __init__ 1")
        self.hass = hass
        self._sensor_type = config[CONF_TYPE]
        self._pin = config[CONF_PIN]
        self._name = config[CONF_NAME]
        ioboard_name = config[CONF_IOBOARD]
        self._ioboard = hass.data.get(ioboard_name)
        self._state = None
        if self._ioboard is None:
            raise HomeAssistantError("I/O Board {0} not registered".format(ioboard_name))
#        self._ioboard.add_output_pin(self)

    @property
    def device_class(self):
        """Return the class of this device, from DEVICE_CLASSES."""
        return self._sensor_type

    @property
    def should_poll(self):
        """No polling needed."""
        return False

    @property
    def name(self):
        """Return the name of the device if any."""
        return self._name

    @property
    def is_on(self):
        """Return true if device is on."""
        #import time
        with self._ioboard.lock:
            if self._state is None:
#                _LOGGER.critical("OneRelay %s is_on", self.name)
                self._state = self._ioboard.read_output_pin(self._pin)
#                _LOGGER.critical("OneRelay 2 %s is_on=%s", self.name, self._state)
        return self._state

    def turn_on(self, **kwargs): # pylint: disable=W0613
        """Turn the device on."""
#        import time
        with self._ioboard.lock:
#            _LOGGER.critical("OneRelay %s turn_on", self.name)
            self._ioboard.write_output_pin(self._pin, 1)
#            _LOGGER.critical("OneRelay 2 %s turn_on", self.name)
            self._state = True
        self.schedule_update_ha_state()

    def turn_off(self, **kwargs): # pylint: disable=W0613
        """Turn the device off."""
#        import time
        with self._ioboard.lock:
#            _LOGGER.critical("OneRelay %s turn_off", self.name)
            self._ioboard.write_output_pin(self._pin, 0)
#            _LOGGER.critical("OneRelay 2 %s turn_off", self.name)
            self._state = False
        self.schedule_update_ha_state()


class IOBoardHW(object):
    """
    The MCP23017 port 0 pin controler
    #
    """
    # Define registers values from datasheet
    IODIRA = 0x00  # IO direction A - 1= input 0 = output
    IODIRB = 0x01  # IO direction B - 1= input 0 = output
    # The GPINTEN register controls the interrupt-onchange feature for each
    # pin on port B.
    GPINTENB = 0x05
    # Default value for port B - These bits set the compare value for pins
    # configured for interrupt-on-change.  If the associated pin level is the
    # opposite from the register bit, an interrupt occurs.
    DEFVALB = 0x07
    # Interrupt control register for port B.  If 1 interrupt is fired when the
    # pin matches the default value, if 0 the interrupt is fired on state
    # change
    INTCONB = 0x09
    GPPUB = 0x0D  # pull-up resistors for port B
    IOCONA = 0x0A  # see datasheet for configuration register
    IOCONB = 0x0B  # see datasheet for configuration register
    # The INTF register reflects the interrupt condition on the port B pins of
    # any pin that is enabled for interrupts.  A set bit indicates that the
    # associated pin caused the interrupt.
    INTFB = 0x0F
    # The INTCAP register captures the GPIO port B value at the time the
    # interrupt occurred.
    INTCAPB = 0x11
    GPIOA = 0x12  # data port A
    GPIOB = 0x13  # data port B
    OLATA = 0x14  # output latches A
    OLATB = 0x15  # output latches B

    _input_port_value = 0x00
    _output_port_value = 0x00
    _ioconfig = 0x00

    def __init__(self, smbus, i2c_address, lock):
        """
        init object with i2c address, default is 0x20, 0x21 for IOBoard board,
        load default configuration
        """
#        _LOGGER.debug("IOBoardHW __init__ 1")
        self._lock = lock
        self._bus = smbus
        self._i2c_address = i2c_address
        self._initialize();

    # local methods

    def _initialize(self):

#        _LOGGER.critical("initialize {0}".format(self.name))
        with self._lock:
            #self._bus.write_byte_data(self._i2c_address, self.IOCON, self.__ioconfig)
            iocon = self._bus.read_byte_data(self._i2c_address, self.IOCONB)
            self._updatebyte(iocon, 1, 0) 
            self._updatebyte(iocon, 2, 0) 
            self._updatebyte(iocon, 6, 0) 
            self._bus.write_byte_data(self._i2c_address, self.IOCONB, iocon)
            self._bus.write_byte_data(self._i2c_address, self.IODIRB, 0xFF)  #Port B as input
            self._input_port_value = self._bus.read_byte_data(self._i2c_address, self.GPIOB)
            self._bus.write_byte_data(self._i2c_address, self.GPINTENB, 0xFF)

            self._bus.write_byte_data(self._i2c_address, self.IODIRA, 0x00)  #Port A as output
            self._output_port_value = self._bus.read_byte_data(self._i2c_address, self.GPIOA)


    @staticmethod
    def checkbit(byte, bit):
        """ internal method for reading the value of a single bit
        within a byte
        """
        value = 0
        if byte & (1 << bit):
            value = 1
        return value

    @staticmethod
    def _updatebyte(byte, bit, value):
        """
        internal method for setting the value of a single bit within a byte
        """
        if value == 0:
            return byte & ~(1 << bit)
        elif value == 1:
            return byte | (1 << bit)

    # public methods

#    def set_port_pullups(self, value):
#        """
#        set the internal 100K pull-up resistors for the selected IO port
#        """
#        self._bus.write_byte_data(self._i2c_address, self.GPPUB, value)
#        return
#
    def read_input_port(self):
        """
        read all pins on the selected port
        port 0 = pins 1 to 8
        returns number between 0 and 255 or 0x00 and 0xFF
        """
#        _LOGGER.critical("read_input_port {0}".format(self.name))
        try:
            self._input_port_value = self._bus.read_byte_data(self._i2c_address, self.GPIOB)
        except OSError as err:
            _LOGGER.critical("read_input_port: OS error: {0}".format(err))
            self._initialize()
            self._input_port_value = self._bus.read_byte_data(self._i2c_address, self.GPIOB)
        return self._input_port_value

    def read_output_port(self):
        """
        read all pins on the selected port
        port 0 = pins 1 to 8
        returns number between 0 and 255 or 0x00 and 0xFF
        """
#        _LOGGER.critical("read_output_port {0}".format(self.name))
        try:
            self._output_port_value = self._bus.read_byte_data(self._i2c_address, self.GPIOA)
        except OSError as err:
            _LOGGER.critical("read_output_port: OS error: {0}".format(err))
        return self._output_port_value

    def read_output_pin(self, pin):
        """
        read individual output pin 1 - 8
        """
#        _LOGGER.critical("read_output_pin {0} {1}".format(self.name, pin))
        port = self.read_output_port()
        pin_mask = 1 << (pin - 1)
        if port & pin_mask != 0:
            return 1
        return 0

    def write_output_pin(self, pin, value):
        """
        write to an individual pin 1 - 8
        """
#        _LOGGER.critical("write_output_pin {0} {1}".format(self.name, pin))
        try:
            pin = pin - 1
            self._output_port_value = self._updatebyte(self._output_port_value, pin, value)
            self._bus.write_byte_data(self._i2c_address, self.GPIOA, self._output_port_value)
        except OSError as err:
            _LOGGER.critical("write_output_pin: OS error: {0}".format(err))
        return

    @property
    def lock(self):
        """Return lock object."""
        return self._lock

    @property
    def name(self):
        """Return the name of the sensor."""
        return 'ioboard{}'.format(self.board_id + 1)

    @property
    def board_id(self):
        """Return the id of the ioboard."""
        return self._i2c_address - DEFAULT_I2C_ADDRESS


#    def set_interrupt_polarity(self, value):
#        """
#        This sets the polarity of the INT output pins
#        1 = Active-high.
#        0 = Active-low.
#        """
#        if value == 0:
#            self._ioconfig = self._updatebyte(self._ioconfig, 1, 0)
#            self._bus.write_byte_data(self._i2c_address, self.IOCON, self._ioconfig)
#        if value == 1:
#            self._ioconfig = self._updatebyte(self._ioconfig, 1, 1)
#            self._bus.write_byte_data(self._i2c_address, self.IOCON, self._ioconfig)
#        return
#
#    def set_interrupt_type(self, value):
#        """
#        Sets the type of interrupt for each pin on the selected port
#        1 = interrupt is fired when the pin matches the default value, 0 =
#        the interrupt is fired on state change
#        """
#        self._bus.write_byte_data(self._i2c_address, self.INTCONB, value)
#        return
#
#    def set_interrupt_defaults(self, value):
#        """
#        These bits set the compare value for pins configured for
#        interrupt-on-change on the selected port.
#        If the associated pin level is the opposite from the register bit, an
#        interrupt occurs.
#        """
#        self._bus.write_byte_data(self._i2c_address, self.DEFVALB, value)
#        return
#
    def read_interrupt_status(self):
        """
        read the interrupt status for the pins on the selected port
        port 0 = pins 1 to 8, port 1 = pins 9 to 16
        """
        return self._bus.read_byte_data(self._i2c_address, self.INTFB)

    def read_interrupt_capture(self):
        """
        read the value from the selected port at the time of the last
        interrupt trigger
        """
        return self._bus.read_byte_data(self._i2c_address, self.INTCAPB)

#    def reset_interrupts(self):
#        """
#        Reset the interrupts to 0
#        """
#        self.read_interrupt_capture()
#        return


class IOBoard(IOBoardHW):
    """IOBoard class"""

    def __init__(self, hass, smbus, config, lock):
        """Initialize the IOBoard."""
        import RPi.GPIO as GPIO

#        _LOGGER.debug("IOBoard __init__ 1")
        super(IOBoard, self).__init__(smbus, config.get(CONF_I2C_ADDRESS), lock)
        self._int_pin = config.get(CONF_INT_PIN)
        self._old_port = self.read_input_port()
        self._inputs = []
#        self._outputs = []

        hass.data[self.name] = self

        inputs = config.get(CONF_INPUTS)
        for pin_num, input_pin in inputs.items():
            pin_type = input_pin.get(CONF_TYPE)
            input_actions = input_pin.get(CONF_ACTIONS)
            if input_actions is None:
                input_actions = INPUT_TYPES.get(pin_type)
            load_platform(hass, CONF_BINARY_SENSOR, DOMAIN, {
                CONF_PIN: pin_num,
                CONF_NAME: input_pin.get(CONF_NAME),
                CONF_TYPE: pin_type,
                CONF_ACTIONS: input_actions,
                CONF_IOBOARD: self.name
                }, config)

        outputs = config.get(CONF_OUTPUTS)
        for pin_num, output_pin in outputs.items():
            pin_type = output_pin.get(CONF_TYPE)
            output_type = OUTPUT_TYPES.get(pin_type)
            load_platform(hass, output_type[0], DOMAIN, {
                CONF_PIN: pin_num,
                CONF_NAME: output_pin.get(CONF_NAME),
                CONF_TYPE: output_type[1],
                CONF_IOBOARD: self.name
                }, config)

        def int_callback(pin): # pylint: disable=W0613
            """Interrupt callback"""
#            import time
            with self.lock:
                try:
#                    _LOGGER.critical("IOBoard %s GPIO interrupt", self.name)
                    status = self.read_interrupt_status()
                    capture = self.read_interrupt_capture()
                    port = self.read_input_port()
#                    _LOGGER.critical("IOBoard 2 %s GPIO Interrupt, status=%s, capture=%s, port=%s", self.name, status, capture, port)
                    for input_pin in self._inputs:
                        if port & input_pin.pin_mask != self._old_port & input_pin.pin_mask:
                            input_pin.on_pin_change(port)
                    self._old_port = port
                except OSError as err:
                    _LOGGER.critical("int_callback({0}): OS error: {1}".format(pin, err))
                except:
                    _LOGGER.critical("int_callback({0}): Unexpected error: {0}}".format(pin, sys.exc_info()[0]))
#            _LOGGER.critical("IOBoard 3 %s GPIO interrupt", self.name)

#        _LOGGER.critical("IOBoard %s __init__", self.name)
        GPIO.setup(self._int_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self._int_pin, GPIO.FALLING, callback=int_callback)

    def add_input_pin(self, input_pin):
        """Register new input pin."""
        self._inputs.append(input_pin)

#    def add_output_pin(self, output_pin):
#        """Register new output pin."""
#        self._outputs.append(output_pin)

    @property
    def should_poll(self):
        """No polling needed."""
        return False

    @property
    def old_port(self):
        """Return previous port value"""
        return self._old_port
