"""
Example implementation of the serial communication protocol that enables testing.
"""

import logging
from typing import Callable, Tuple

import click
import serial

logging.basicConfig(
    level=logging.DEBUG,
    format="[%(asctime)s] - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

LOGGER = logging.getLogger(__name__)


COMMAND_HEARTBEAT = b"\x01"
COMMAND_SERVICE_STARTED = b"\x02"
COMMAND_SERVICE_CRASHED = b"\x03"
COMMAND_PULSE_LED = b"\x04"
COMMAND_PULSE_NO_LED = b"\x05"

_ALL_COMMANDS = [
    COMMAND_HEARTBEAT,
    COMMAND_SERVICE_STARTED,
    COMMAND_SERVICE_CRASHED,
    COMMAND_PULSE_LED,
    COMMAND_PULSE_NO_LED,
]

COMMAND_SIZE_BYTES = 1


CallbackCallables = Tuple[
    Callable[[], None],
    Callable[[], None],
    Callable[[], None],
    Callable[[bytes], None],
    Callable[[], None],
]


def _write_command(serial_port: serial.Serial, command: bytes) -> bool:
    """
    Write a command to the serial port. Log and raise a ValueError if something goes wrong.
    :param command: The single byte command to write to the serial port.
    :raises: ValueError if there is a problem writing to the port
    :return: None
    """
    LOGGER.info(f"Writing [{command}] to the serial port...")
    serial_port.write(command)

    read = serial_port.read(COMMAND_SIZE_BYTES)
    if read != command:
        LOGGER.error(
            f"Something has gone wrong with the serial connection. "
            f"Expected to read [{command}], got [{read}]."
        )
        return False
    else:
        LOGGER.info("command written correctly.")
        return True


def configure_serial(port_name: str, baud_rate: int, led_on_trigger: bool) -> CallbackCallables:
    """
    Configure the serial port to be able to send commands to the arduino using our thin protocol.
    :param port_name: The name of the serial port to connect to.
    :param baud_rate: The baud rate to talk at
    :param led_on_trigger: Blink the indicator LED on the arduino's PCB when an actuator trigger
    is sent if this is enabled. Otherwise it will remain dark.
    :return: Callables used by consumers.
    """

    pulse_command = COMMAND_PULSE_LED if led_on_trigger else COMMAND_PULSE_NO_LED

    LOGGER.info("Configuring serial port...")
    port = serial.Serial(port=port_name, baudrate=baud_rate, timeout=5)

    def flush_all_waiting() -> None:
        """
        Flush all waiting bytes in both directions.
        :return: None
        """
        port.flushOutput()
        port.flushInput()
        port.flush()

    flush_all_waiting()

    while not _write_command(port, COMMAND_HEARTBEAT):
        pass

    LOGGER.info("Serial port configured!")

    def write_message_to_port(message: bytes) -> None:
        """
        Write a command to the serial port.
        :param message: The message to write
        :return: None
        """
        _write_command(port, message)

    def send_trigger_solenoid() -> None:
        """
        Sends the command to pulse the solenoid.
        :return: None
        """
        write_message_to_port(pulse_command)

    def send_connected() -> None:
        """
        Sends the command to tell the Arduino that we have connected to twitch and will
        now be processing commands.
        :return: None
        """
        write_message_to_port(COMMAND_SERVICE_STARTED)

    def send_stopped() -> None:
        """
        Sends the command to tell the Arduino that we have crashed and not to expect any more
        commands for some time.
        :return: None
        """
        write_message_to_port(COMMAND_SERVICE_CRASHED)

    def disconnect_arduino() -> None:
        """
        Flush bytes and then safely disconnect the Arduino.
        :return: None
        """
        flush_all_waiting()
        port.close()

    return (
        send_trigger_solenoid,
        send_connected,
        send_stopped,
        write_message_to_port,
        disconnect_arduino,
    )


@click.command()
@click.option(
    "--serial-port",
    type=click.Path(exists=True, dir_okay=False),
    required=False,
    default="/dev/ttyACM0",
    show_default=True,
    help="The serial port to write commands to.",
)
@click.option(
    "--baud",
    type=click.IntRange(min=0),
    required=False,
    default=115200,
    show_default=True,
    help="The baud rate to communicate with the arduino at.",
)
@click.option(
    "--command-index",
    type=click.IntRange(min=0, max=2),
    required=False,
    default=0,
    show_default=True,
    help=(
        "The index of the command to send to the arduino, "
        "0 for `send_trigger_solenoid`, 1 for `send_connected`, 2 for `send_stopped`."
    ),
)
@click.option(
    "--led-on-trigger",
    type=click.BOOL,
    required=False,
    default=False,
    show_default=True,
    help=(
        "Blink the indicator LED on the arduino's PCB when an actuator trigger is sent if this is "
        "enabled. Otherwise it will remain dark."
    ),
)
def main(serial_port: str, baud: int, command_index: int, led_on_trigger: bool) -> None:
    """
    Write a given command to the arduino as often as possible to test that it is received correctly.
    \f
    :param serial_port: The serial port to write commands to.
    :param baud: The baud rate to communicate with the arduino at.
    :param command_index: The index of the command to send to the arduino, 0 for
    `send_trigger_solenoid`, 1 for `send_connected`, 2 for `send_stopped`.
    :param led_on_trigger: Blink the indicator LED on the arduino's PCB when an actuator trigger is
    sent if this is enabled. Otherwise it will remain dark.
    :return: None
    """

    # For this example we only care about the first 3 commands.
    commands = configure_serial(
        port_name=serial_port, baud_rate=baud, led_on_trigger=led_on_trigger
    )[0:2]

    while True:
        commands[command_index]()


if __name__ == "__main__":
    main()  # pylint: disable=no-value-for-parameter
