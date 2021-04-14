import os
import sys
import logging
import logging.handlers
import datetime
from aidoop.log.custom_timed_rotating_handler import CustomTimedRotatingFileHandler


class LoggerObject:
    LOG_FILE_PATH = "./logs"

    def __init__(self, name, level=logging.INFO, stream=sys.stdout):
        # create logger
        self.logger = logging.getLogger(name)

        # set level
        self.level = level
        self.logger.setLevel(level)

        # create console handler with level
        self.console_handler = logging.StreamHandler()
        self.console_handler.setLevel(level)

        # create rotating file hander with level
        log_file_path = LoggerObject.LOG_FILE_PATH
        os.makedirs(log_file_path, exist_ok=True)
        self.rotating_handler = CustomTimedRotatingFileHandler(
            filename=f"{log_file_path}/object-tracking",
            when="H",
            interval=1,
            encoding="utf-8",
        )
        self.rotating_handler.suffix = "-%Y-%m-%d-%H.log"
        self.rotating_handler.setLevel(level)

        # create formatter
        self.formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        self.console_handler.setFormatter(self.formatter)
        self.rotating_handler.setFormatter(self.formatter)

        # add console handler to logger
        self.logger.addHandler(self.console_handler)
        self.logger.addHandler(self.rotating_handler)

    @property
    def level(self):
        return self.__real_attr

    @level.setter
    def level(self, level):
        self.__real_attr = level
        self.logger.setLevel(level)

    def debug(self, message):
        return self.logger.debug(message)

    def info(self, message):
        return self.logger.info(message)

    def warning(self, message):
        return self.logger.warning(message)

    def error(self, message):
        return self.logger.error(message)

    def critical(self, message):
        return self.logger.critical(message)


class Logger:
    LOGGER_DICT = dict()

    class Level:
        DEBUG = logging.DEBUG
        INFO = logging.INFO
        WARNING = logging.WARNING
        ERROR = logging.ERROR
        CRITICAL = logging.CRITICAL

    @staticmethod
    def get(name, level=logging.INFO, stream=sys.stdout):
        try:
            logger = Logger.LOGGER_DICT[name]
        except KeyError:
            logger = LoggerObject(name, level, stream)
            Logger.LOGGER_DICT[name] = logger

        return logger


if __name__ == "__main__":
    cl = LoggerObject("classifier")

    cl.debug("test")
    cl.info("test")
    cl.warning("test")
    cl.error("test")
    cl.critical("test")

    dl = LoggerObject("detector", logging.INFO)
    dl.debug("test")
    dl.info("test")
    dl.warning("test")
    dl.error("test")
    dl.critical("test")

    edgeDebug = Logger.get("edge").debug
    edgeDebug("hi")

    edgeDebug = Logger.get("edge").debug
    edgeDebug("hi twice")

    import time

    while True:
        dl.info("test")
        time.sleep(5)
