################################################################
#
# A Simple output logger
#
###################################################
import utime
from micropython import const
from machine import RTC

CRITICAL = const(50)
ERROR = const(40)
WARNING = const(30)
INFO = const(20)
DEBUG = const(10)
NOTSET = const(0)

_level_dict = {
    CRITICAL: "CRIT",
    ERROR: "ERRO",
    WARNING: "WARN",
    INFO: "INFO",
    DEBUG: "DBUG",
}
_level = ERROR
_loggers = {}


class SimpleLogger(object):
    LOGGER_NAME = "LOG"

    # stack inspection is not possible on micropython
    # might want to be able to add a socket as print target for remote / headless logging

    def __init__(self, logger_name="LOG"):
        self.level = _level
        self.LOGGER_NAME = logger_name
        self.rtc = RTC()

    def _level_str(self, level):
        if level in _level_dict:
            return _level_dict[level]
        return "LVL" + str(level)

    def log_hex(self, caller, message, data):
        if isinstance(data, bytearray):
            hexstr = [hex(c) for c in data]
        elif isinstance(data, bytes):
            hexstr = [hex(int(c)) for c in data]
        elif hasattr(data, '__iter__'):
            hexstr = [hex(c) for c in data]
        else:
            hexstr = '0x{0:02x}'.format(data)

        self.log(caller, DEBUG, "{} :{}".format(message, hexstr))

    def debug(self, caller, msg):
        self.log(caller, DEBUG, msg)

    def info(self, caller, msg):
        self.log(caller, INFO, msg)

    def warning(self, caller, msg):
        self.log(caller, WARNING, msg)

    def error(self, caller, msg):
        self.log(caller, ERROR, msg)

    def critical(self, caller, msg):
        self.log(caller, CRITICAL, msg)

    def log(self, caller, level, message):
        if level >= self.level:
            # t = utime.localtime()
            t = self.rtc.now()
            print(
                "[{}:{}]{}-{:02d}-{:02d} {:02d}:{:02d}:{:02d},{:04d} - {}: {}".format(self.LOGGER_NAME,
                                                                                      self._level_str(level),
                                                                                      t[0], t[1], t[2], t[3], t[4],
                                                                                      t[5], int(t[6]/1000),
                                                                                      caller,
                                                                                      message))


def get_logger(name):
    if name in _loggers:
        return _loggers[name]
    new_logger = SimpleLogger(name)
    _loggers[name] = new_logger
    return new_logger


def info(caller, msg):
    get_logger(None).info(caller, msg)


def debug(caller, msg):
    get_logger(None).debug(caller, msg)


def basic_config(level=INFO, filename=None, stream=None, format=None):
    global _level, _stream
    _level = level
    if stream:
        _stream = stream
    if filename is not None:
        print("logging.basicConfig: filename arg is not supported")
    if format is not None:
        print("logging.basicConfig: format arg is not supported")


if __name__ == "__main__":
    basic_config(level=NOTSET)
    logger = get_logger("MAIN")
    logger.log("main", NOTSET, "Log Test")
    logger.warning("main", "Warn Test")
    logger.info("main", "Info Test")
    logger.debug("main", "Debug Test")
    logger.error("main", "Error Test")
    logger.critical("main", "Critical Test")
    logger.log_hex("main", "log Hex", 0x1010)
    logger.log_hex("main", "log Hex", bytearray([0x0001, 0x0002]))
    logger.log_hex("main", "log Hex", bytes([1, 10, 100]))
