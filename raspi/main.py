import json
import random
import shutil
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path

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

MANUAL_DRIVE_SPEED = 125
MANUAL_TURN_SPEED = 175
HUSKY_DRIVE_SPEED = 125
HUSKY_TURN_SPEED = 175
LINE_LEARNING_DRIVE_SPEED = 150
LINE_LEARNING_TURN_SPEED = 135
LINE_RUNNING_DRIVE_SPEED = 150
LINE_RUNNING_TURN_SPEED = 150

TRACK_INTERVAL_SEC = 0.1
TRACK_TURN_DURATION_SEC = 0.1
TRACK_TURN_WAIT_SEC = 0.3
STATUS_POLL_INTERVAL_MS = 500
RECONNECT_INTERVAL_SEC = 2
OBSTACLE_LED_BLINK_INTERVAL_SEC = 0.5
COLLECT_LOOP_INTERVAL_SEC = 0.02
FORWARD_COLLECT_FPS = 2
TURN_COLLECT_FPS = 10
LINE_INTERVAL_SEC = 0.1
LINE_CONFIDENCE_THRESHOLD = 0.45
TRAIN_EPOCHS = 12
TRAIN_BATCH_SIZE = 16
MIN_IMAGES_PER_LABEL = 10
CAMERA_WIDTH = 160
CAMERA_HEIGHT = 90
CAMERA_INDEX_CANDIDATES = (0, 1, 2, 3, 4)

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
MODEL_DIR = BASE_DIR / "models"
MODEL_PATH = MODEL_DIR / "line_cnn.keras"

CMD_FORWARD = "w"
CMD_BACKWARD = "s"
CMD_LEFT = "a"
CMD_RIGHT = "d"
CMD_STOP = "x"

MOTOR_DIRECTIONS = {
    CMD_FORWARD: ("f", "f", "drive"),
    CMD_BACKWARD: ("b", "b", "drive"),
    CMD_LEFT: ("b", "f", "turn"),
    CMD_RIGHT: ("f", "b", "turn"),
    CMD_STOP: ("s", 0, "s", 0),
}

LEARNING_LABELS = {
    CMD_FORWARD: "forward",
    CMD_LEFT: "left",
    CMD_RIGHT: "right",
}
LABEL_NAMES = ("forward", "left", "right")
LABEL_TO_INDEX = {label: index for index, label in enumerate(LABEL_NAMES)}
INDEX_TO_COMMAND = {
    LABEL_TO_INDEX["forward"]: CMD_FORWARD,
    LABEL_TO_INDEX["left"]: CMD_LEFT,
    LABEL_TO_INDEX["right"]: CMD_RIGHT,
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
    manual_command: str = CMD_STOP
    collecting: bool = False
    training: bool = False
    imitation_running: bool = False
    camera_connected: bool = False
    camera_index: str | None = None
    camera_error: str | None = None
    dataset_counts: dict | None = None
    model_ready: bool = False
    train_message: str | None = None
    train_accuracy: float | None = None
    train_progress: float = 0.0
    train_epoch: int = 0
    train_total_epochs: int = TRAIN_EPOCHS
    train_eta_seconds: int | None = None
    last_prediction: str | None = None
    last_prediction_confidence: float | None = None


app = Flask(__name__)
state = RobotState()
state_lock = threading.Lock()
arduino_lock = threading.Lock()

arduino = None
huskylens = None
connection_lock = threading.Lock()
camera = None
camera_index = None
camera_lock = threading.Lock()
training_lock = threading.Lock()
line_model = None
last_collection_times = {
    CMD_FORWARD: 0.0,
    CMD_LEFT: 0.0,
    CMD_RIGHT: 0.0,
}


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


def get_manual_command():
    with state_lock:
        return state.manual_command


def get_collecting():
    with state_lock:
        return state.collecting


def ensure_learning_dirs():
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    for label in LABEL_NAMES:
        (DATA_DIR / label).mkdir(parents=True, exist_ok=True)


def count_dataset_images():
    ensure_learning_dirs()
    return {
        label: len(list((DATA_DIR / label).glob("*.jpg")))
        for label in LABEL_NAMES
    }


def update_dataset_state():
    update_state(
        dataset_counts=count_dataset_images(),
        model_ready=MODEL_PATH.exists(),
    )


def import_cv2():
    try:
        import cv2
    except ImportError:
        update_state(camera_error="opencv-python is not installed")
        return None
    return cv2


def import_tensorflow():
    try:
        import tensorflow as tf
    except ImportError:
        update_state(train_message="tensorflow is not installed")
        return None
    return tf


def camera_source_candidates():
    sources = []

    for path in sorted(Path("/dev").glob("video*")):
        sources.append(str(path))

    for index in CAMERA_INDEX_CANDIDATES:
        sources.append(index)

    deduped = []
    for source in sources:
        if source not in deduped:
            deduped.append(source)
    return deduped


def open_camera_source(cv2, source):
    backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
    errors = []

    for backend in backends:
        try:
            candidate = cv2.VideoCapture(source, backend)
            if not candidate.isOpened():
                candidate.release()
                errors.append(f"{source} backend {backend}: open failed")
                continue

            candidate.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            candidate.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            candidate.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            frame = None
            ok = False
            for _ in range(5):
                ok, frame = candidate.read()
                if ok and frame is not None:
                    break
                time.sleep(0.05)

            if ok and frame is not None:
                return candidate, None

            candidate.release()
            errors.append(f"{source} backend {backend}: read failed")
        except Exception as exc:
            errors.append(f"{source} backend {backend}: {exc}")

    return None, "; ".join(errors)


def connect_camera(force_reconnect=False):
    global camera, camera_index

    cv2 = import_cv2()
    if cv2 is None:
        update_state(camera_connected=False, camera_index=None)
        return None

    with camera_lock:
        if camera is not None:
            if not force_reconnect:
                return camera
            camera.release()
            camera = None
            camera_index = None

        errors = []
        for source in camera_source_candidates():
            candidate, error = open_camera_source(cv2, source)
            if candidate is None:
                if error:
                    errors.append(error)
                continue

            camera = candidate
            camera_index = str(source)
            update_state(
                camera_connected=True,
                camera_index=str(source),
                camera_error=None,
            )
            return camera

    detail = " | ".join(errors[-4:]) if errors else "no /dev/video* or camera index opened"
    update_state(
        camera_connected=False,
        camera_index=None,
        camera_error=f"Camera not found ({detail})",
    )
    return None


def disconnect_camera(error=None):
    global camera, camera_index

    with camera_lock:
        if camera is not None:
            camera.release()
        camera = None
        camera_index = None

    update_state(
        camera_connected=False,
        camera_index=None,
        camera_error=error,
    )


def read_camera_frame():
    cam = connect_camera()
    if cam is None:
        return None

    with camera_lock:
        ok, frame = cam.read()

    if not ok or frame is None:
        disconnect_camera("Camera read failed")
        return None

    return frame


def preprocess_frame(frame):
    cv2 = import_cv2()
    if cv2 is None:
        return None

    height = frame.shape[0]
    cropped = frame[height // 2 :, :]
    resized = cv2.resize(cropped, (CAMERA_WIDTH, CAMERA_HEIGHT))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    return rgb


def frame_to_model_input(frame):
    processed = preprocess_frame(frame)
    if processed is None:
        return None

    import numpy as np

    return processed.astype("float32") / 255.0


def save_collection_frame(command):
    cv2 = import_cv2()
    if cv2 is None:
        return False

    label = LEARNING_LABELS.get(command)
    if label is None:
        return False

    frame = read_camera_frame()
    if frame is None:
        return False

    processed = preprocess_frame(frame)
    if processed is None:
        return False

    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"{timestamp}-{time.time_ns()}.jpg"
    path = DATA_DIR / label / filename
    cv2.imwrite(str(path), cv2.cvtColor(processed, cv2.COLOR_RGB2BGR))
    update_dataset_state()
    return True


def collect_fps_for_command(command):
    if command == CMD_FORWARD:
        return FORWARD_COLLECT_FPS

    if command in {CMD_LEFT, CMD_RIGHT}:
        return TURN_COLLECT_FPS

    return 0


def should_collect_frame(command, now):
    fps = collect_fps_for_command(command)
    if fps <= 0:
        return False

    min_interval = 1.0 / fps
    elapsed = now - last_collection_times.get(command, 0.0)
    if elapsed < min_interval:
        return False

    last_collection_times[command] = now
    return True


def load_training_data():
    cv2 = import_cv2()
    if cv2 is None:
        return None, None, "opencv-python is not installed"

    import numpy as np

    images = []
    labels = []
    paths_by_label = {
        label: sorted((DATA_DIR / label).glob("*.jpg"))
        for label in LABEL_NAMES
    }
    counts = {label: len(paths) for label, paths in paths_by_label.items()}
    sample_count = min(counts.values())

    if sample_count < MIN_IMAGES_PER_LABEL:
        too_few = ", ".join(
            f"{label} {count}/{MIN_IMAGES_PER_LABEL}"
            for label, count in counts.items()
            if count < MIN_IMAGES_PER_LABEL
        )
        return None, None, f"Images are too few ({too_few})"

    for label in LABEL_NAMES:
        for path in random.sample(paths_by_label[label], sample_count):
            image = cv2.imread(str(path))
            if image is None:
                continue
            image = cv2.resize(image, (CAMERA_WIDTH, CAMERA_HEIGHT))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            images.append(image.astype("float32") / 255.0)
            labels.append(LABEL_TO_INDEX[label])

    if not images:
        return None, None, "No training images found"

    update_state(train_message=f"Training with {sample_count} images per label")
    return np.array(images), np.array(labels), None


def build_line_model(tf):
    model = tf.keras.Sequential(
        [
            tf.keras.layers.Input(shape=(CAMERA_HEIGHT, CAMERA_WIDTH, 3)),
            tf.keras.layers.Conv2D(16, 3, activation="relu"),
            tf.keras.layers.MaxPooling2D(),
            tf.keras.layers.Conv2D(32, 3, activation="relu"),
            tf.keras.layers.MaxPooling2D(),
            tf.keras.layers.Conv2D(48, 3, activation="relu"),
            tf.keras.layers.GlobalAveragePooling2D(),
            tf.keras.layers.Dense(32, activation="relu"),
            tf.keras.layers.Dropout(0.15),
            tf.keras.layers.Dense(len(LABEL_NAMES), activation="softmax"),
        ]
    )
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )
    return model


def build_training_progress_callback(tf):
    class TrainingProgressCallback(tf.keras.callbacks.Callback):
        def on_train_begin(self, logs=None):
            self.started_at = time.time()
            update_state(
                train_progress=0.0,
                train_epoch=0,
                train_total_epochs=TRAIN_EPOCHS,
                train_eta_seconds=None,
                train_message=f"Training epoch 0/{TRAIN_EPOCHS}",
            )

        def on_epoch_end(self, epoch, logs=None):
            completed = epoch + 1
            elapsed = time.time() - self.started_at
            avg_epoch_sec = elapsed / completed if completed else 0
            remaining = max(TRAIN_EPOCHS - completed, 0)
            eta = int(avg_epoch_sec * remaining) if remaining else 0
            progress = completed / TRAIN_EPOCHS
            accuracy = logs.get("accuracy") if logs else None

            update_state(
                train_progress=progress,
                train_epoch=completed,
                train_eta_seconds=eta,
                train_accuracy=float(accuracy) if accuracy is not None else None,
                train_message=f"Training epoch {completed}/{TRAIN_EPOCHS}",
            )

    return TrainingProgressCallback()


def train_line_model():
    global line_model

    with training_lock:
        update_state(
            training=True,
            train_message="Training started",
            train_accuracy=None,
            train_progress=0.0,
            train_epoch=0,
            train_total_epochs=TRAIN_EPOCHS,
            train_eta_seconds=None,
        )
        try:
            tf = import_tensorflow()
            if tf is None:
                update_state(training=False)
                return

            images, labels, error = load_training_data()
            if error:
                update_state(training=False, train_message=error)
                return

            model = build_line_model(tf)
            history = model.fit(
                images,
                labels,
                epochs=TRAIN_EPOCHS,
                batch_size=TRAIN_BATCH_SIZE,
                validation_split=0.2,
                shuffle=True,
                verbose=0,
                callbacks=[build_training_progress_callback(tf)],
            )
            MODEL_DIR.mkdir(parents=True, exist_ok=True)
            model.save(MODEL_PATH)
            line_model = model
            accuracy = float(history.history["accuracy"][-1])
            update_state(
                training=False,
                model_ready=True,
                train_accuracy=accuracy,
                train_progress=1.0,
                train_epoch=TRAIN_EPOCHS,
                train_total_epochs=TRAIN_EPOCHS,
                train_eta_seconds=0,
                train_message=f"Training complete: accuracy {accuracy:.3f}",
            )
        except Exception as exc:
            update_state(
                training=False,
                train_eta_seconds=None,
                train_message=f"Training failed: {exc}",
            )


def load_line_model():
    global line_model

    if line_model is not None:
        return line_model

    if not MODEL_PATH.exists():
        update_state(train_message="Model file not found", model_ready=False)
        return None

    tf = import_tensorflow()
    if tf is None:
        return None

    line_model = tf.keras.models.load_model(MODEL_PATH)
    update_state(model_ready=True)
    return line_model


def build_motor_protocol(command, drive_speed, turn_speed):
    if command == CMD_STOP:
        left_direction, left_speed, right_direction, right_speed = MOTOR_DIRECTIONS[command]
        return f"m:{left_direction},{left_speed},{right_direction},{right_speed}"

    left_direction, right_direction, speed_kind = MOTOR_DIRECTIONS[command]
    speed = drive_speed if speed_kind == "drive" else turn_speed
    left_speed = speed
    right_speed = speed
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


def send_motor(
    command,
    mode=None,
    ignore_obstacle=False,
    drive_speed=MANUAL_DRIVE_SPEED,
    turn_speed=MANUAL_TURN_SPEED,
):
    if command != CMD_STOP and get_obstacle() and not ignore_obstacle:
        command = CMD_STOP
        mode = "stopped"

    protocol_command = build_motor_protocol(command, drive_speed, turn_speed)
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
    update_state(manual_command=command)
    if command == CMD_STOP:
        stop_robot("stopped")
        return

    drive_speed = LINE_LEARNING_DRIVE_SPEED if get_collecting() else MANUAL_DRIVE_SPEED
    turn_speed = LINE_LEARNING_TURN_SPEED if get_collecting() else MANUAL_TURN_SPEED

    if send_motor(
        command,
        "manual",
        ignore_obstacle=True,
        drive_speed=drive_speed,
        turn_speed=turn_speed,
    ):
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
            send_motor(
                CMD_LEFT,
                "tracking",
                drive_speed=HUSKY_DRIVE_SPEED,
                turn_speed=HUSKY_TURN_SPEED,
            )
            set_led(LED_GREEN)
            time.sleep(TRACK_TURN_DURATION_SEC)
            send_motor(
                CMD_STOP,
                "tracking",
                drive_speed=HUSKY_DRIVE_SPEED,
                turn_speed=HUSKY_TURN_SPEED,
            )
            time.sleep(TRACK_TURN_WAIT_SEC)
            continue

        if target_x > FRAME_CENTER_X + DEAD_ZONE:
            send_motor(
                CMD_RIGHT,
                "tracking",
                drive_speed=HUSKY_DRIVE_SPEED,
                turn_speed=HUSKY_TURN_SPEED,
            )
            set_led(LED_GREEN)
            time.sleep(TRACK_TURN_DURATION_SEC)
            send_motor(
                CMD_STOP,
                "tracking",
                drive_speed=HUSKY_DRIVE_SPEED,
                turn_speed=HUSKY_TURN_SPEED,
            )
            time.sleep(TRACK_TURN_WAIT_SEC)
            continue

        send_motor(
            CMD_FORWARD,
            "tracking",
            drive_speed=HUSKY_DRIVE_SPEED,
            turn_speed=HUSKY_TURN_SPEED,
        )
        set_led(LED_GREEN)
        time.sleep(TRACK_INTERVAL_SEC)


def collection_loop():
    while True:
        snapshot = get_state_snapshot()
        if not snapshot["collecting"]:
            time.sleep(COLLECT_LOOP_INTERVAL_SEC)
            continue

        command = snapshot["manual_command"]
        now = time.monotonic()
        if (
            snapshot["mode"] == "manual"
            and command in LEARNING_LABELS
            and should_collect_frame(command, now)
        ):
            save_collection_frame(command)

        time.sleep(COLLECT_LOOP_INTERVAL_SEC)


def imitation_loop():
    while True:
        if get_mode() != "imitation":
            time.sleep(LINE_INTERVAL_SEC)
            continue

        model = load_line_model()
        if model is None:
            stop_robot("stopped")
            time.sleep(LINE_INTERVAL_SEC)
            continue

        if get_obstacle():
            stop_for_obstacle()
            time.sleep(LINE_INTERVAL_SEC)
            continue

        frame = read_camera_frame()
        if frame is None:
            stop_robot("stopped")
            time.sleep(LINE_INTERVAL_SEC)
            continue

        model_input = frame_to_model_input(frame)
        if model_input is None:
            stop_robot("stopped")
            time.sleep(LINE_INTERVAL_SEC)
            continue

        import numpy as np

        predictions = model.predict(np.expand_dims(model_input, axis=0), verbose=0)[0]
        prediction_index = int(np.argmax(predictions))
        confidence = float(predictions[prediction_index])
        command = INDEX_TO_COMMAND[prediction_index]
        label = LABEL_NAMES[prediction_index]
        update_state(
            last_prediction=label,
            last_prediction_confidence=confidence,
            imitation_running=True,
        )

        if confidence < LINE_CONFIDENCE_THRESHOLD:
            send_motor(
                CMD_STOP,
                "imitation",
                drive_speed=LINE_RUNNING_DRIVE_SPEED,
                turn_speed=LINE_RUNNING_TURN_SPEED,
            )
            set_led(LED_RED)
            time.sleep(LINE_INTERVAL_SEC)
            continue

        send_motor(
            command,
            "imitation",
            drive_speed=LINE_RUNNING_DRIVE_SPEED,
            turn_speed=LINE_RUNNING_TURN_SPEED,
        )
        set_led(LED_GREEN)
        time.sleep(LINE_INTERVAL_SEC)


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
    update_state(mode="tracking", imitation_running=False)
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/tracking/stop", methods=["POST"])
def tracking_stop():
    stop_robot("stopped")
    update_state(
        target_detected=False,
        target_id=None,
        target_x=None,
        imitation_running=False,
    )
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/learning/collect/start", methods=["POST"])
def collection_start():
    ensure_learning_dirs()
    update_dataset_state()
    if connect_camera(force_reconnect=True) is None:
        update_state(collecting=False, train_message="Camera connection failed")
        return jsonify({"ok": False, "error": "camera connection failed", "status": get_state_snapshot()}), 400

    for command in last_collection_times:
        last_collection_times[command] = 0.0

    update_state(collecting=True, train_message="Collection started")
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/learning/collect/stop", methods=["POST"])
def collection_stop():
    update_dataset_state()
    update_state(collecting=False, train_message="Collection stopped")
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/learning/data/delete", methods=["POST"])
def learning_data_delete():
    ensure_learning_dirs()
    for label in LABEL_NAMES:
        label_dir = DATA_DIR / label
        if label_dir.exists():
            shutil.rmtree(label_dir)
        label_dir.mkdir(parents=True, exist_ok=True)

    update_dataset_state()
    update_state(
        collecting=False,
        train_message="Collected images deleted",
        train_accuracy=None,
        last_prediction=None,
        last_prediction_confidence=None,
    )
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/learning/train", methods=["POST"])
def learning_train():
    if get_state_snapshot()["training"]:
        return jsonify({"ok": False, "error": "training already running"}), 409

    threading.Thread(target=train_line_model, daemon=True).start()
    update_state(train_message="Training queued")
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/imitation/start", methods=["POST"])
def imitation_start():
    if not MODEL_PATH.exists():
        update_state(model_ready=False, train_message="Model file not found")
        return jsonify({"ok": False, "error": "model file not found"}), 400

    if connect_camera(force_reconnect=True) is None:
        update_state(imitation_running=False, train_message="Camera connection failed")
        return jsonify({"ok": False, "error": "camera connection failed", "status": get_state_snapshot()}), 400

    update_state(
        mode="imitation",
        collecting=False,
        imitation_running=True,
        target_detected=False,
        target_id=None,
        target_x=None,
    )
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/imitation/stop", methods=["POST"])
def imitation_stop():
    stop_robot("stopped")
    update_state(
        imitation_running=False,
        last_prediction=None,
        last_prediction_confidence=None,
    )
    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/camera/reconnect", methods=["POST"])
def camera_reconnect():
    if connect_camera(force_reconnect=True) is None:
        return jsonify({"ok": False, "error": "camera connection failed", "status": get_state_snapshot()}), 400

    return jsonify({"ok": True, "status": get_state_snapshot()})


@app.route("/api/status")
def status():
    update_dataset_state()
    return jsonify(get_state_snapshot())


def start_background_threads():
    threading.Thread(target=connection_manager_loop, daemon=True).start()
    threading.Thread(target=arduino_reader_loop, daemon=True).start()
    threading.Thread(target=led_blink_loop, daemon=True).start()
    threading.Thread(target=tracking_loop, daemon=True).start()
    threading.Thread(target=collection_loop, daemon=True).start()
    threading.Thread(target=imitation_loop, daemon=True).start()


if __name__ == "__main__":
    ensure_learning_dirs()
    update_dataset_state()
    connect_arduino()
    connect_huskylens()
    start_background_threads()
    app.run(host="0.0.0.0", port=5000, threaded=True)
