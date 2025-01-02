import datetime

from colorama import Fore, Style, init

# Initialize colorama for colored output in the console
init(autoreset=True)

# Log type constants
LOG_TYPE_DEBUG = "DEBUG"
LOG_TYPE_INFO = "INFO"
LOG_TYPE_SUCCESS = "SUCCESS"
LOG_TYPE_WARNING = "WARNING"
LOG_TYPE_ERROR = "ERROR"

# Log level numeric values for comparison
LOG_LEVELS = {
    LOG_TYPE_DEBUG: 10,
    LOG_TYPE_INFO: 20,
    LOG_TYPE_SUCCESS: 30,
    LOG_TYPE_WARNING: 30,
    LOG_TYPE_ERROR: 40
}

# Log color map for console output
LOG_COLORS = {
    LOG_TYPE_DEBUG: Fore.WHITE,
    LOG_TYPE_INFO: Fore.BLUE,
    LOG_TYPE_SUCCESS: Fore.GREEN,
    LOG_TYPE_WARNING: Fore.YELLOW,
    LOG_TYPE_ERROR: Fore.RED
}

# Global variables to configure where logs are written
log_to_console = True
console_log_level = LOG_LEVELS[LOG_TYPE_INFO]


def configure_logging(console=True, min_console_level=LOG_TYPE_DEBUG):
    """
    Configures the logging output.

    :param console: Whether to log to the console (default is True)
    :param min_console_level: The minimum log level to display in the console (default is DEBUG)
    """
    global log_to_console, console_log_level
    log_to_console = console
    console_log_level = LOG_LEVELS[min_console_level]


def log(log_type, message, additional_info=None):
    """
    Logs a message with a specific log type and additional information.

    :param log_type: The type of log (e.g., DEBUG, INFO, WARNING, ERROR)
    :param message: The main log message
    :param additional_info: Additional information (can be a string or list of strings) to include in the log (optional)
    """
    # Get the current timestamp
    timestamp = datetime.datetime.now().strftime("%d.%m.%Y %H:%M:%S")

    # Format the log message
    log_msg = f"{timestamp}\t\t{log_type}\t{message}"

    print(log_msg)
    
    # Handle multiple additional_info strings
    if additional_info:
        if isinstance(additional_info, list):
            additional_info = " | ".join(additional_info)  # Join list of strings with a separator
        log_msg += f". Additional Info: {additional_info}"

    # Log to console if enabled and meets the minimum log level
    if log_to_console and LOG_LEVELS[log_type] >= console_log_level:
        color = LOG_COLORS.get(log_type, Fore.WHITE)
        print(f"{color}{log_msg}{Style.RESET_ALL}")
