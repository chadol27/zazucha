import json
import threading
import time
from dataclasses import asdict, dataclass

import serial
from flask import Flask, jsonify, render_template
from serial.tools import list_ports

try:
    from pyhuskylens import HuskyLens, ALGORITHM_OBJECT_TRACKING
except ImportError:
    HuskyLens = None
    ALGORITHM_OBJECT_TRACKING = 2


ARDUINO_BAUD = 9600
HUSKYLENS_PORT = "/dev/ttyAMA0"
HUSKYLENS_BAUD = 9600

TARGET_ID = 1
FRAME_CENTER_X = 160
DEAD_ZONE = 25
SAFE_DISTANCE_CM = 20
DRIVE_SPEED = 125
TURN_SPEED = 175

TRACK_INTERVAL_SEC = 0.1
TRACK_TURN_DURATION_SEC = 0.1
TRACK_TURN_WAIT_SEC = 0.3
STATUS_POLL_INTERVAL_MS = 500
RECONNECT_INTERVAL_SEC = 2
OBSTACLE_LED_BLINK_INTERVAL_SEC = 0.5

CMD_FORWARD = "w"
CMD_BACKWARD = "s"
CMD_LEFT = "a"
CMD_RIGHT = "d"
CMD_STOP = "x"

MOTOR_COMMANDS = {
    CMD_FORWARD: ("f", DRIVE_SPEED, "f", DRIVE_SPEED),
    CMD_BACKWARD: ("b", DRIVE_SPEED, "b", DRIVE_SPEED),
    CMD_LEFT: ("b", TURN_SPEED, "f", TURN_SPEED),
    CMD_RIGHT: ("f", TURN_SPEED, "b", TURN_SPEED),
    CMD_STOP: ("s", 0, "s", 0),
}

LED_RED = (150, 0, 0)
LED_GREEN = (0, 150, 0)
LED_BLUE = (0, 0, 150)
LED_OFF = (0, 0, 0)


@dataclass
class RobotState:
    mode: str = "stopped"
    last_command: str = "m:s,0,s,0"
    distance: int | None = None
    obstacle: bool = False
    arduino_connected: bool = False
    arduino_port: str | None = None
    huskylens_connected: bool = False
    huskylens_error: str | None = None
    arduino_error: str | None = None
    target_detected: bool = False
    target_id: int | None = None
    target_x: int | None = None


app = Flask(__name__)
state = RobotState()
state_lock = threading.Lock()
arduino_lock = threading.Lock()

arduino = None
huskylens = None
connection_lock = threading.Lock()


def update_state(**kwargs):
    with state_lock:
        for key, value in kwargs.items():
            setattr(state, key, value)


def get_state_snapshot():
    with state_lock:
        return asdict(state)


def get_obstacle():
    with state_lock:
        return state.obstacle


def get_mode():
    with state_lock:
        return state.mode


def build_motor_protocol(command):
    left_direction, left_speed, right_direction, right_speed = MOTOR_COMMANDS[command]
    return f"m:{left_direction},{left_speed},{right_direction},{right_speed}"


def build_led_protocol(color):
    red, green, blue = color
    return f"l:{red},{green},{blue}"


def write_arduino(command):
    if arduino is None:
        return False

    line = f"{command}\n"

    try:
        with arduino_lock:
            arduino.write(line.encode("ascii"))
            arduino.flush()
        return True
    except (OSError, serial.SerialException) as exc:
        disconnect_arduino(str(exc))
        return False


def send_motor(command, mode=None, ignore_obstacle=False):
    if command != CMD_STOP and get_obstacle() and not ignore_obstacle:
        command = CMD_STOP
        mode = "stopped"

    protocol_command = build_motor_protocol(command)
    ok = write_arduino(protocol_command)
    if ok:
        update_state(last_command=protocol_command)
        if mode:
            update_state(mode=mode)
    return ok


def set_led(color):
    write_arduino(build_led_protocol(color))


def stop_robot(reason="stopped"):
    send_motor(CMD_STOP, reason)
    set_led(LED_RED)


def stop_for_obstacle():
    send_motor(CMD_STOP, "obstacle")


def set_manual_command(command):
    if command == CMD_STOP:
        stop_robot("stopped")
        return

    if send_motor(command, "manual", ignore_obstacle=True):
        set_led(LED_BLUE)


def arduino_port_candidates():
    ports = list(list_ports.comports())
    preferred = []
    fallback = []

    for port in ports:
        device = port.device
        if device == HUSKYLENS_PORT:
            continue

        if device.startswith("/dev/ttyACM") or device.startswith("/dev/ttyUSB"):
            preferred.append(device)
        else:
            fallback.append(device)

    return preferred + fallback


def disconnect_arduino(error=None):
    global arduino

    with arduino_lock:
        if arduino is not None:
            try:
                arduino.close()
            except (OSError, serial.SerialException):
                pass
        arduino = None

    update_state(
        arduino_connected=False,
        arduino_port=None,
        arduino_error=error,
    )


def disconnect_huskylens(error=None):
    global huskylens

    huskylens = None
    update_state(
        huskylens_connected=False,
        huskylens_error=error,
        target_detected=False,
        target_id=None,
        target_x=None,
    )


def connect_arduino():
    global arduino

    with connection_lock:
        if arduino is not None:
            return

    for port in arduino_port_candidates():
        try:
            candidate = serial.Serial(port, ARDUINO_BAUD, timeout=0.2)
            time.sleep(2)
            deadline = time.time() + 3

            while time.time() < deadline:
                line = candidate.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if "distance" in data:
                    arduino = candidate
                    update_state(
                        distance=int(data["distance"]),
                        arduino_connected=True,
                        arduino_port=port,
                        arduino_error=None,
                    )
                    stop_robot("stopped")
                    return

            candidate.close()
        except (OSError, serial.SerialException) as exc:
            update_state(arduino_error=str(exc))

    update_state(
        arduino_connected=False,
        arduino_error="Arduino port not found",
    )


def connect_huskylens():
    global huskylens

    with connection_lock:
        if huskylens is not None:
            return

    if HuskyLens is None:
        update_state(
            huskylens_connected=False,
            huskylens_error="pyhuskylens is not installed",
        )
        return

    try:
        huskylens = HuskyLens(HUSKYLENS_PORT, baud=HUSKYLENS_BAUD)
        if hasattr(huskylens, "knock") and not huskylens.knock():
            disconnect_huskylens("HuskyLens did not respond")
            return

        huskylens.set_alg(ALGORITHM_OBJECT_TRACKING)
        update_state(
            huskylens_connected=True,
            huskylens_error=None,
        )
    except Exception as exc:
        disconnect_huskylens(str(exc))


def arduino_reader_loop():
    while True:
        if arduino is None:
            time.sleep(0.5)
            continue

        try:
            current = arduino
            if current is None:
                time.sleep(0.5)
                continue
            line = current.readline().decode("utf-8", errors="ignore").strip()
        except (OSError, serial.SerialException) as exc:
            disconnect_arduino(str(exc))
            time.sleep(0.5)
            continue

        if not line:
            continue

        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            continue

        if "distance" not in data:
            continue

        distance = int(data["distance"])
        previous_obstacle = get_obstacle()
        obstacle = 0 < distance <= SAFE_DISTANCE_CM
        update_state(distance=distance, obstacle=obstacle)

        if obstacle and get_mode() != "manual":
            stop_for_obstacle()
        elif previous_obstacle and get_mode() != "manual":
            stop_robot("stopped")


def get_target_block():
    if huskylens is None:
        return None

    try:
        try:
            blocks = huskylens.get_blocks(
                algorithm=ALGORITHM_OBJECT_TRACKING,
                ID=TARGET_ID,
            )
        except TypeError:
            blocks = huskylens.get_blocks()

        for block in blocks:
            if getattr(block, "ID", None) == TARGET_ID:
                return block
    except Exception as exc:
        disconnect_huskylens(str(exc))

    return None


def connection_manager_loop():
    while True:
        snapshot = get_state_snapshot()

        if not snapshot["arduino_connected"] or arduino is None:
            connect_arduino()

        if not snapshot["huskylens_connected"] or huskylens is None:
            connect_huskylens()

        time.sleep(RECONNECT_INTERVAL_SEC)


def led_blink_loop():
    led_on = False

    while True:
        if not get_obstacle():
            led_on = False
            time.sleep(OBSTACLE_LED_BLINK_INTERVAL_SEC)
            continue

        led_on = not led_on
        set_led(LED_RED if led_on else LED_OFF)
        time.sleep(OBSTACLE_LED_BLINK_INTERVAL_SEC)


def tracking_loop():
    while True:
        if get_mode() != "tracking":
            time.sleep(TRACK_INTERVAL_SEC)
            continue

        if get_obstacle():
            stop_for_obstacle()
            time.sleep(TRACK_INTERVAL_SEC)
            continue

        block = get_target_block()
        if block is None:
            update_state(
                target_detected=False,
                target_id=None,
                target_x=None,
            )
            stop_robot("tracking")
            time.sleep(TRACK_INTERVAL_SEC)
            continue

        target_x = int(block.x)
        update_state(
            target_detected=True,
            target_id=TARGET_ID,
            target_x=target_x,
            huskylens_connected=True,
            huskylens_error=None,
        )

        if target_x < FRAME_CENTER_X - DEAD_ZONE:
            send_motor(CMD_LEFT, "tracking")
            set_led(LED_GREEN)
            time.sleep(TRACK_TURN_DURATION_SEC)
            send_motor(CMD_STOP, "tracking")
            time.sleep(TRACK_TURN_WAIT_SEC)
            continue

        if target_x > FRAME_CENTER_X + DEAD_ZONE:
            send_motor(CMD_RIGHT, "tracking")
            set_led(LED_GREEN)
            time.sleep(TRACK_TURN_DURATION_SEC)
            send_motor(CMD_STOP, "tracking")
            time.sleep(TRACK_TURN_WAIT_SEC)
            continue

        send_motor(CMD_FORWARD, "tracking")
        set_led(LED_GREEN)
        time.sleep(TRACK_INTERVAL_SEC)


@app.route("/")
def index():
    return render_template(
        "index.html",
        status_poll_interval_ms=STATUS_POLL_INTERVAL_MS,
    )


@app.route("/api/manual/<command>", methods=["POST"])
def manual(command):
    command = command.lower()
    if command not in {CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP}:
        return jsonify({"ok": False, "error": "invalid command"}), 400

    set_manual_command(command)
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/tracking/start", methods=["POST"])
def tracking_start():
    update_state(mode="tracking")
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/tracking/stop", methods=["POST"])
def tracking_stop():
    stop_robot("stopped")
    update_state(
        target_detected=False,
        target_id=None,
        target_x=None,
    )
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/status")
def status():
    return jsonify(get_state_snapshot())


def start_background_threads():
    threading.Thread(target=connection_manager_loop, daemon=True).start()
    threading.Thread(target=arduino_reader_loop, daemon=True).start()
    threading.Thread(target=led_blink_loop, daemon=True).start()
    threading.Thread(target=tracking_loop, daemon=True).start()


if __name__ == "__main__":
    connect_arduino()
    connect_huskylens()
    start_background_threads()
    app.run(host="0.0.0.0", port=5000, threaded=True)
