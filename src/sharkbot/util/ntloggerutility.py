from typing import Union

from ntcore import NetworkTableInstance, Topic, GenericEntry
import logging

class NTLoggerUtility:
    def __init__(self, table_name):
        """
        Initializes a logging utility for a specific table.

        Parameters:
        - table_name: str - The name of the NetworkTable to log to.
        """
        nt = NetworkTableInstance.getDefault()
        self.nt_table = nt.getTable(table_name)
        self.log_level = "INFO"
        self.entries = {}

    def setLevel(self, level: str):
        """
        Set the logging level for this logger.

        Parameters:
        - level (str): The logging level ('DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL').
        """
        valid_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if level not in valid_levels:
            raise ValueError(f"Invalid log level: {level}. Must be one of {valid_levels}")
        self.log_level = level
        self.nt_table.putString("LogLevel", self.log_level)  # Optionally log the level to the table

    def log(self, level: str, key: str, value : any):
        """
        Logs a key-value pair to the NetworkTable.

        Parameters:
        - key: str - The key to log.
        - value: Any - The value to log (Number, String, etc.).
        """
        if not self._shouldLog(level):
            return  # Skip logging if the level is lower than the current log level

        # Lazy initialize the key in the table
        if key not in self.entries:
            topic = self.nt_table.getTopic(key)
            self.entries[key] = topic.getGenericEntry()
        entry = self.entries[key]

        if isinstance(value, int):
            entry.setInteger(value)
        elif isinstance(value, float):
            entry.setDouble(value)
        elif isinstance(value, str):
            entry.setString(value)
        elif isinstance(value, bool):
            entry.setBoolean(value)
        else:
            entry.setString(str(value))
            logging.warning(f"Unsupported type for NetworkTables logging; {type(value)}")

    def get(self, key: str, default=None):
        """
        Retrieves a value from the NetworkTable.

        Parameters:
        - key: str - The key to retrieve.
        - default: Any - The default value if the key doesn't exist.

        Returns:
        - The value associated with the key or the default.
        """
        if key in self.entries:
            entry = self.entries[key]
            # Generic entries have a typed get operations; a default must be provided
            value = entry.get()
            if value and value.isDouble():
                return entry.getDouble(0.0)
            if value and value.isBoolean():
                return entry.getBoolean(False)
            if value and value.isString():
                return entry.getString("")
            if value and value.isInteger():
                return entry.getInteger(0)
        return None

    def _shouldLog(self, level: str) -> bool:
        """
        Determine whether a message at the specified level should be logged.

        Parameters:
        - level (str): The logging level of the message.

        Returns:
        - bool: True if the message should be logged, False otherwise.
        """
        level_order = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40, "CRITICAL": 50}
        return level_order[level] >= level_order[self.log_level]

    def debug(self, key: str, value: str | float | bool) -> None:
        """Log a DEBUG level message."""
        self.log("DEBUG", key, value)

    def info(self, key: str, value: str | float | bool) -> None:
        """Log an INFO level message."""
        self.log("INFO", key, value)

    def warning(self, key: str, value: str | float | bool) -> None:
        """Log a WARNING level message."""
        self.log("WARNING", key, value)

    def error(self, key: str, value: str | float | bool) -> None:
        """Log an ERROR level message."""
        self.log("ERROR", key, value)

    def critical(self, key: str, value: str | float | bool) -> None:
        """Log a CRITICAL level message."""
        self.log("CRITICAL", key, value)