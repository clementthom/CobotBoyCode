import tkinter as tk
from tkinter import ttk
import threading
import time

import serial
import serial.tools.list_ports

# =====================================================
# CONNECTION SETTINGS
# =====================================================
USE_REAL_ARDUINO = True
SERIAL_PORT = "COM6"
BAUD_RATE = 9600

# =====================================================
# CONSTANTS
# =====================================================
objects = ["Gomme", "Gobelet", "Cylindre"]
modes = ["ECO", "Normal", "SPEED"]

cycle_names = {
    0: "Cycle 1",
    1: "Cycle 2",
}

# Positions available for position testing in Tab 4.
# These are display names only. The index sent to Arduino still selects
# the correct basePositions[cycle][position].
# 0=Entree, 1=Machine A, 2=Machine B, 3=Sortie.
position_names = [
    "Entree",
    "Machine A",
    "Machine B",
    "Sortie",
]

position_code_names = [
    "POS_convoyeurEntree",
    "POS_machineA",
    "POS_machineB",
    "POS_convoyeurSortie",
]

position_code_names = [
    "POS_convoyeurEntree",
    "POS_machineA",
    "POS_machineB",
    "POS_convoyeurSortie",
]

# =====================================================
# GLOBAL STATE
# =====================================================
arduino = None
reader_running = False

state = {
    "connection": "DISCONNECTED",
    "robot_state": "NOT_INIT",

    "init": 0,
    "initializing": 0,
    "run": 0,
    "pause": 0,

    "cycle_index": 0,
    "cycle_name": "Cycle 1",

    "object_index": 0,
    "object_name": "Gomme",

    "mode_index": 1,
    "mode_name": "Normal",

    "pieces": 1,
    "current_piece": 0,
    "speed_factor": 1.00,

    "servo_a": 90.0,
    "servo_b": 90.0,
    "servo_z": 90.0,

    "current_x": "?",
    "current_y": "?",
    "current_z": "?",
    "last_position": "None",
}

# =====================================================
# SERIAL HELPERS
# =====================================================
def find_arduino_port():
    ports = serial.tools.list_ports.comports()

    for port in ports:
        desc = port.description.lower()
        device = port.device

        if (
            "arduino" in desc
            or "mega" in desc
            or "ch340" in desc
            or "usb serial" in desc
        ):
            return device

    return None


def resolve_serial_port():
    if SERIAL_PORT.upper() != "AUTO":
        return SERIAL_PORT

    return find_arduino_port()


def connect_arduino():
    global arduino, reader_running

    if not USE_REAL_ARDUINO:
        state["connection"] = "SIMULATION"
        update_dashboard()
        log("[APP] Simulation mode active.")
        return

    try:
        port = resolve_serial_port()

        if port is None:
            raise RuntimeError("No Arduino port found.")

        arduino = serial.Serial(port, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        arduino.reset_input_buffer()

        # Tell the Arduino that the Python maintenance terminal is active.
        # The Arduino will lock its physical OLED to "MAINTENANCE TERMINAL".
        arduino.write(b"TERMINAL_ON\n")
        arduino.flush()
        time.sleep(0.1)

        arduino.write(b"PING\n")
        arduino.flush()

        response = arduino.readline().decode(errors="ignore").strip()

        if "OK;TERMINAL_ON" in response or "PONG" in response or "STATUS" in response:
            log("[APP] Terminal mode requested on Arduino.")
        else:
            log("[APP] Warning: terminal handshake response unclear: " + response)

        state["connection"] = f"CONNECTED {port}"
        update_dashboard()

        reader_running = True
        threading.Thread(target=read_arduino, daemon=True).start()

        log(f"[APP] Connected to Arduino on {port}")

        send_command("GET_STATUS")
        send_command("GET_CONFIG")

    except Exception as e:
        state["connection"] = "CONNECTION ERROR"
        update_dashboard()
        log("[APP] Connection error: " + repr(e))


def disconnect_arduino():
    global arduino, reader_running

    reader_running = False

    try:
        if arduino is not None and arduino.is_open:
            arduino.close()
    except Exception:
        pass

    arduino = None
    state["connection"] = "DISCONNECTED"
    update_dashboard()
    log("[APP] Disconnected.")


def reconnect_arduino():
    disconnect_arduino()
    connect_arduino()


def send_command(cmd):
    log("[APP → ARDUINO] " + cmd)

    if not USE_REAL_ARDUINO:
        simulate_response(cmd)
        return

    if arduino is None or not arduino.is_open:
        log("[APP] Arduino not connected.")
        return

    try:
        arduino.write((cmd.strip() + "\n").encode("utf-8"))
        arduino.flush()
    except Exception as e:
        log("[APP] Send error: " + str(e))


def read_arduino():
    global reader_running

    while reader_running:
        try:
            raw = arduino.readline().decode(errors="ignore").strip()
            if raw:
                root.after(0, lambda line=raw: handle_arduino_line(line))
        except Exception as e:
            root.after(0, lambda err=e: log("[APP] Read error: " + str(err)))
            break


# =====================================================
# SIMULATION MODE
# =====================================================
def simulate_response(cmd):
    parts = cmd.strip().split()
    if not parts:
        return

    c0 = parts[0].upper()

    if c0 == "TERMINAL_ON":
        handle_arduino_line("OK;TERMINAL_ON")
        emit_status()

    elif c0 == "TERMINAL_OFF":
        handle_arduino_line("OK;TERMINAL_OFF")
        emit_status()

    elif c0 == "PING":
        handle_arduino_line("PONG")

    elif c0 == "GET_STATUS":
        emit_status()

    elif c0 == "GET_CONFIG":
        emit_config()

    elif c0 == "SET_CYCLE":
        state["cycle_index"] = int(parts[1])
        state["cycle_name"] = cycle_names[state["cycle_index"]]
        handle_arduino_line("OK;SET_CYCLE")
        emit_status()

    elif c0 == "SET_OBJECT":
        state["object_index"] = int(parts[1])
        state["object_name"] = objects[state["object_index"]]
        handle_arduino_line("OK;SET_OBJECT")
        emit_status()

    elif c0 == "SET_MODE":
        state["mode_index"] = int(parts[1])
        state["mode_name"] = modes[state["mode_index"]]
        handle_arduino_line("OK;SET_MODE")
        emit_status()

    elif c0 == "SET_PIECES":
        state["pieces"] = int(parts[1])
        handle_arduino_line("OK;SET_PIECES")
        emit_status()

    elif c0 == "SET_SPEED":
        state["speed_factor"] = float(parts[1])
        handle_arduino_line("OK;SET_SPEED")
        emit_status()

    elif c0 == "INITIALIZE":
        state["robot_state"] = "INITIALIZING"
        state["init"] = 0
        state["run"] = 0
        state["pause"] = 0
        handle_arduino_line("OK;INITIALIZE_START")
        emit_status()
        root.after(800, finish_sim_init)


    elif c0 == "RUN_CYCLE":

        if state["init"] == 0:
            handle_arduino_line("ERR;NOT_INITIALIZED")

            emit_status()

            return

        state["robot_state"] = "RUNNING"

        state["run"] = 1

        state["pause"] = 0

        state["current_piece"] = 0

        handle_arduino_line("OK;RUN_CYCLE_START")

        emit_status()

        run_sim_cycle()


    elif c0 == "PAUSE_TOGGLE":

        if state["run"] == 1 and state["pause"] == 0:

            state["robot_state"] = "PAUSED"

            state["run"] = 0

            state["pause"] = 1

            handle_arduino_line("OK;PAUSED")

            emit_status()


        elif state["pause"] == 1:

            state["robot_state"] = "RUNNING"

            state["run"] = 1

            state["pause"] = 0

            handle_arduino_line("OK;RESUMED")

            emit_status()

            run_sim_cycle()


        else:

            handle_arduino_line("ERR;PAUSE_NOT_AVAILABLE")

            emit_status()


    elif c0 == "GOTO_POSITION":
        if len(parts) >= 4:
            cycle_index = int(parts[1])
            object_index = int(parts[2])
            position_index = int(parts[3])

            state["cycle_index"] = cycle_index
            state["cycle_name"] = cycle_names.get(cycle_index, "Cycle ?")
            state["object_index"] = object_index
            state["object_name"] = objects[object_index] if 0 <= object_index < len(objects) else "Object ?"

            position_name = position_names[position_index] if 0 <= position_index < len(position_names) else "Position ?"
            handle_arduino_line(f"OK;GOTO_POSITION;CYCLE={cycle_index};OBJECT={object_index};POSITION={position_index};POSITION_NAME={position_name}")
            emit_status()
        else:
            handle_arduino_line("ERR;GOTO_POSITION_BAD_ARGS")

    elif c0 == "GO_NAMED_POSITION":
        if len(parts) >= 4:
            cycle_index = int(parts[1])
            object_index = int(parts[2])
            position_name = parts[3]

            state["cycle_index"] = cycle_index
            state["cycle_name"] = cycle_names.get(cycle_index, "Cycle ?")
            state["object_index"] = object_index
            state["object_name"] = objects[object_index] if 0 <= object_index < len(objects) else "Object ?"

            handle_arduino_line(f"OK;GO_NAMED_POSITION;CYCLE={cycle_index};OBJECT={object_index};POSITION_NAME={position_name}")
            emit_status()
        else:
            handle_arduino_line("ERR;GO_NAMED_POSITION_BAD_ARGS")

    else:

        handle_arduino_line("ERR;UNKNOWN_COMMAND")

def finish_sim_init():
    state["robot_state"] = "READY"
    state["init"] = 1
    state["run"] = 0
    state["pause"] = 0
    handle_arduino_line("OK;INITIALIZE_DONE")
    emit_status()

def run_sim_cycle():
    def loop():
        for p in range(state["current_piece"], state["pieces"]):
            if state["pause"] == 1 or state["run"] == 0:
                return

            state["current_piece"] = p + 1
            emit_status()

            for _ in range(10):
                if state["pause"] == 1 or state["run"] == 0:
                    return
                time.sleep(0.2)

        state["robot_state"] = "READY"
        state["run"] = 0
        state["pause"] = 0
        state["current_piece"] = 0

        handle_arduino_line("OK;RUN_CYCLE_DONE")
        emit_status()

    threading.Thread(target=loop, daemon=True).start()



def emit_status():
    line = (
        f"STATUS;"
        f"STATE={state['robot_state']};"
        f"INIT={state['init']};"
        f"RUN={state['run']};"
        f"PAUSE={state['pause']};"
        f"CYCLE={state['cycle_index']};"
        f"CYCLE_NAME={state['cycle_name']};"
        f"OBJECT={state['object_index']};"
        f"OBJECT_NAME={state['object_name']};"
        f"MODE={state['mode_index']};"
        f"MODE_NAME={state['mode_name']};"
        f"PIECES={state['pieces']};"
        f"CURRENT_PIECE={state['current_piece']};"
        f"SPEED_FACTOR={state['speed_factor']:.2f};"
        f"SERVO_A={state['servo_a']:.1f};"
        f"SERVO_B={state['servo_b']:.1f};"
        f"SERVO_Z={state['servo_z']:.1f}"
    )
    handle_arduino_line(line)


def emit_config():
    line = (
        f"CONFIG;"
        f"CYCLE={state['cycle_index']};"
        f"OBJECT={state['object_index']};"
        f"MODE={state['mode_index']};"
        f"PIECES={state['pieces']};"
        f"SPEED_FACTOR={state['speed_factor']:.2f};"
        f"SERVO_A={state['servo_a']:.1f};"
        f"SERVO_B={state['servo_b']:.1f};"
        f"SERVO_Z={state['servo_z']:.1f}"
    )
    handle_arduino_line(line)


# =====================================================
# PARSER
# =====================================================
def parse_semicolon_line(line):
    parts = line.split(";")
    msg_type = parts[0]

    data = {}
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            data[k] = v

    return msg_type, data


def handle_arduino_line(line):
    log("[ARDUINO] " + line)

    if line == "PONG":
        return

    msg_type, data = parse_semicolon_line(line)

    if msg_type == "STATUS":
        state["robot_state"] = data.get("STATE", state["robot_state"])
        state["init"] = int(data.get("INIT", state["init"]))
        state["initializing"] = int(data.get("INITIALIZING", state.get("initializing", 0)))
        state["run"] = int(data.get("RUN", state["run"]))
        state["pause"] = int(data.get("PAUSE", state["pause"]))

        state["cycle_index"] = int(data.get("CYCLE", state["cycle_index"]))
        state["cycle_name"] = data.get("CYCLE_NAME", cycle_names.get(state["cycle_index"], "Cycle ?"))

        state["object_index"] = int(data.get("OBJECT", state["object_index"]))
        state["object_name"] = data.get("OBJECT_NAME", objects[state["object_index"]])

        state["mode_index"] = int(data.get("MODE", state["mode_index"]))
        state["mode_name"] = data.get("MODE_NAME", modes[state["mode_index"]])

        state["pieces"] = int(data.get("PIECES", state["pieces"]))
        state["current_piece"] = int(data.get("CURRENT_PIECE", state["current_piece"]))
        state["speed_factor"] = float(data.get("SPEED_FACTOR", state["speed_factor"]))

        state["servo_a"] = float(data.get("SERVO_A", state["servo_a"]))
        state["servo_b"] = float(data.get("SERVO_B", state["servo_b"]))
        state["servo_z"] = float(data.get("SERVO_Z", state["servo_z"]))

    elif msg_type == "CONFIG":
        state["cycle_index"] = int(data.get("CYCLE", state["cycle_index"]))
        state["cycle_name"] = cycle_names.get(state["cycle_index"], "Cycle ?")

        state["object_index"] = int(data.get("OBJECT", state["object_index"]))
        state["object_name"] = objects[state["object_index"]]

        state["mode_index"] = int(data.get("MODE", state["mode_index"]))
        state["mode_name"] = modes[state["mode_index"]]

        state["pieces"] = int(data.get("PIECES", state["pieces"]))
        state["speed_factor"] = float(data.get("SPEED_FACTOR", state["speed_factor"]))

    elif msg_type == "OK":
        ok_text = line.split(";", 1)[1] if ";" in line else ""

        if ok_text.startswith("GOTO_POSITION_START"):
            position_index = int(data.get("POSITION", "0"))
            cycle_index = int(data.get("CYCLE", state["cycle_index"]))
            object_index = int(data.get("OBJECT", state["object_index"]))

            state["cycle_index"] = cycle_index
            state["cycle_name"] = cycle_names.get(cycle_index, "Cycle ?")
            state["object_index"] = object_index
            state["object_name"] = objects[object_index] if 0 <= object_index < len(objects) else "Object ?"
            state["last_position"] = position_names[position_index] if 0 <= position_index < len(position_names) else data.get("POSITION_NAME", "?")

            try:
                position_status_var.set(
                    f"Moving: Cycle {cycle_index + 1} / {state['object_name']} / {state['last_position']}"
                )
            except Exception:
                pass

        elif ok_text.startswith("GOTO_POSITION_DONE"):
            position_index = int(data.get("POSITION", "0"))
            cycle_index = int(data.get("CYCLE", state["cycle_index"]))
            object_index = int(data.get("OBJECT", state["object_index"]))

            state["cycle_index"] = cycle_index
            state["cycle_name"] = cycle_names.get(cycle_index, "Cycle ?")
            state["object_index"] = object_index
            state["object_name"] = objects[object_index] if 0 <= object_index < len(objects) else "Object ?"
            state["last_position"] = position_names[position_index] if 0 <= position_index < len(position_names) else data.get("POSITION_NAME", "?")
            state["current_x"] = data.get("X", state["current_x"])
            state["current_y"] = data.get("Y", state["current_y"])
            state["current_z"] = data.get("Z", state["current_z"])

            try:
                position_status_var.set(
                    f"Arrived: Cycle {cycle_index + 1} / {state['object_name']} / {state['last_position']}"
                )
            except Exception:
                pass

    elif msg_type == "EVENT":
        if data.get("TYPE") == "BUTTON":
            name = data.get("NAME", "?")
            status = data.get("STATUS", "RECEIVED")
            message = f"Physical {name} button: {status}"
            log("[BUTTON TEST] " + message)
            try:
                button_test_status_var.set(message)
            except Exception:
                pass

    update_dashboard()


# =====================================================
# GUI ACTIONS
# =====================================================
def apply_config():
    cycle_index = int(config_cycle.get()) - 1
    object_index = objects.index(config_object.get())
    mode_index = modes.index(config_mode.get())
    pieces_value = int(config_pieces.get())
    speed_value = float(config_speed.get())

    state["cycle_index"] = cycle_index
    state["cycle_name"] = cycle_names[cycle_index]
    state["object_index"] = object_index
    state["object_name"] = objects[object_index]
    state["mode_index"] = mode_index
    state["mode_name"] = modes[mode_index]
    state["pieces"] = pieces_value
    state["speed_factor"] = speed_value
    update_dashboard()

    send_command(f"SET_CYCLE {cycle_index}")
    send_command(f"SET_OBJECT {object_index}")
    send_command(f"SET_MODE {mode_index}")
    send_command(f"SET_PIECES {pieces_value}")
    send_command(f"SET_SPEED {speed_value:.2f}")
    send_command("GET_STATUS")
    send_command("GET_CONFIG")


def ihm_init():
    log("[BUTTON] INIT clicked")
    apply_config()
    # Give Arduino a short moment to process the config commands first.
    # Then INITIALIZE uses the same action as the physical INIT button.
    root.after(500, lambda: send_command("INITIALIZE"))


def ihm_play():
    log("[BUTTON] PLAY / START clicked")
    # Do not resend all config here. PLAY should behave like the physical START button.
    # Use APPLY CONFIG before INIT if you changed cycle/object/mode/pieces/speed.
    send_command("RUN_CYCLE")


def ihm_pause():
    log("[BUTTON] PAUSE clicked")
    send_command("PAUSE_TOGGLE")


def test_serial_link():
    log("[TEST] Sending PING + GET_STATUS")
    send_command("PING")
    send_command("GET_STATUS")


def test_init_led():
    log("[TEST] Asking Arduino to blink INIT LED")
    send_command("TEST_INIT_LED")


# =====================================================
# LED AND BUTTON TEST ACTIONS
# =====================================================
def start_button_test_mode():
    log("[BUTTON TEST] Button test mode ON. Press physical INIT / START / PAUSE.")
    try:
        button_test_status_var.set("Button test active. Press a physical button.")
    except Exception:
        pass
    send_command("BUTTON_TEST_ON")


def stop_button_test_mode():
    log("[BUTTON TEST] Button test mode OFF")
    send_command("BUTTON_TEST_OFF")


def led_set(led_name, value):
    send_command(f"LED_SET {led_name} {1 if value else 0}")


def led_pulse(led_name):
    send_command(f"LED_PULSE {led_name}")


def led_auto():
    send_command("LED_AUTO")


# =====================================================
# SERVO TEST ACTIONS
# =====================================================
def go_repos_from_terminal():
    log("[SERVO TEST] Going to repos before manual servo test")
    send_command("GO_REPOS")


def set_servo(servo_name, value):
    value = float(value)
    value = max(0.0, min(180.0, value))
    send_command(f"SERVO_SET {servo_name} {value:.1f}")


def step_servo(servo_name, delta):
    send_command(f"SERVO_STEP {servo_name} {float(delta):.1f}")


# =====================================================
# POSITION CHECK ACTIONS
# =====================================================
def update_position_command_preview(*args):
    try:
        cycle_index = int(position_cycle_var.get()) - 1
        object_index = objects.index(position_object_var.get())
        position_name = position_name_var.get()
        position_index = position_names.index(position_name)

        position_command_var.set(f"GOTO_POSITION {cycle_index} {object_index} {position_index}")
        code_name = position_code_names[position_index]
        position_status_var.set(
            f"Selected: Cycle {cycle_index + 1} / {position_object_var.get()} / {position_name} ({code_name})"
        )
    except Exception:
        pass


def go_to_selected_position():
    try:
        cycle_index = int(position_cycle_var.get()) - 1
        object_index = objects.index(position_object_var.get())
        position_name = position_name_var.get()
        position_index = position_names.index(position_name)

        position_command_var.set(f"GOTO_POSITION {cycle_index} {object_index} {position_index}")
        code_name = position_code_names[position_index]
        position_status_var.set(
            f"Moving to Cycle {cycle_index + 1} / {position_object_var.get()} / {position_name} ({code_name})"
        )

        log(
            f"[POSITION CHECK] Cycle={cycle_index + 1}, "
            f"Object={position_object_var.get()}, Position={position_name}, Code={code_name}, Index={position_index}"
        )

        # Arduino command format:
        # GOTO_POSITION cycle_index object_index position_index
        # position_index maps to:
        # 0 Entree  -> POS_convoyeurEntree
        # 1 Machine A -> POS_machineA
        # 2 Machine B -> POS_machineB
        # 3 Sortie -> POS_convoyeurSortie
        send_command(f"GOTO_POSITION {cycle_index} {object_index} {position_index}")

    except Exception as e:
        position_status_var.set("Position command error: " + str(e))
        log("[POSITION CHECK] Error: " + str(e))


def go_to_repos_from_position_tab():
    position_status_var.set("Going to REPOS")
    send_command("GO_REPOS")

def on_tab_changed(event):
    selected_tab = notebook.index(notebook.select())

    if selected_tab == 1:
        stop_button_test_mode()
        led_auto()
        go_repos_from_terminal()
    elif selected_tab == 2:
        go_repos_from_terminal()
        start_button_test_mode()
    elif selected_tab == 3:
        stop_button_test_mode()
        led_auto()
        update_position_command_preview()
    else:
        stop_button_test_mode()
        led_auto()

# =====================================================
# GUI DISPLAY
# =====================================================
def sync_servo_sliders_from_state():
    # When the Arduino reports current servo angles, keep Tab 2 starting values
    # matched to the real REPOS/current angles.
    try:
        servo_a_var.set(state["servo_a"])
        servo_b_var.set(state["servo_b"])
        servo_z_var.set(state["servo_z"])
    except Exception:
        # Sliders may not exist during startup.
        pass

def update_dashboard():
    connection_var.set(state["connection"])
    robot_state_var.set(state["robot_state"])

    init_var.set(str(state["init"]))
    run_var.set(str(state["run"]))
    pause_var.set(str(state["pause"]))

    cycle_var.set(state["cycle_name"])
    object_var.set(state["object_name"])
    mode_var.set(state["mode_name"])
    pieces_var.set(str(state["pieces"]))
    current_piece_var.set(str(state["current_piece"]))
    speed_var.set(f"{state['speed_factor']:.2f}")

    try:
        current_x_var.set(str(state.get("current_x", "?")))
        current_y_var.set(str(state.get("current_y", "?")))
        current_z_var.set(str(state.get("current_z", "?")))
        last_position_var.set(str(state.get("last_position", "None")))
        servo_a_live_var.set(f"{state['servo_a']:.1f}")
        servo_b_live_var.set(f"{state['servo_b']:.1f}")
        servo_z_live_var.set(f"{state['servo_z']:.1f}")
    except Exception:
        pass

    sync_servo_sliders_from_state()

    draw_oled()


def draw_oled():
    oled_canvas.delete("all")
    oled_canvas.create_rectangle(0, 0, 300, 170, fill="black", outline="#444444")

    # This is the PYTHON fake OLED/status preview.
    # The real Arduino OLED is locked to MAINTENANCE TERMINAL MODE,
    # but this preview should show the live robot state from Arduino STATUS lines.
    oled_text(0, 0, "PYTHON TERMINAL STATUS")
    oled_text(0, 16, "----------------------")
    oled_text(0, 34, f"Cycle : {state['cycle_name']}")
    oled_text(0, 50, f"Objet : {state['object_name']}")
    oled_text(0, 66, f"Mode  : {state['mode_name']}")
    oled_text(0, 82, f"Pieces: {state['pieces']}")
    oled_text(0, 98, f"Speed : {state['speed_factor']:.2f}")
    oled_text(0, 114, f"State : {state['robot_state']}")
    oled_text(0, 130, f"Init:{state['init']} Initing:{state.get('initializing',0)}")
    oled_text(0, 146, f"Run:{state['run']} Pause:{state['pause']} Piece:{state['current_piece']}/{state['pieces']}")


def oled_text(x, y, text):
    oled_canvas.create_text(
        x,
        y,
        text=text,
        fill="white",
        anchor="nw",
        font=("Consolas", 9)
    )


def log(message):
    log_box.insert(tk.END, message + "\n")
    log_box.see(tk.END)


# =====================================================
# WINDOW
# =====================================================
root = tk.Tk()
root.title("cobot_boy_terminal_25")
root.geometry("1180x760")
root.minsize(1100, 700)

# =====================================================
# VARIABLES
# =====================================================
connection_var = tk.StringVar(value="DISCONNECTED")
robot_state_var = tk.StringVar(value="NOT_INIT")

init_var = tk.StringVar(value="0")
run_var = tk.StringVar(value="0")
pause_var = tk.StringVar(value="0")

cycle_var = tk.StringVar(value="Cycle 1")
object_var = tk.StringVar(value="Gomme")
mode_var = tk.StringVar(value="Normal")
pieces_var = tk.StringVar(value="1")
current_piece_var = tk.StringVar(value="0")
speed_var = tk.StringVar(value="1.00")

current_x_var = tk.StringVar(value="?")
current_y_var = tk.StringVar(value="?")
current_z_var = tk.StringVar(value="?")
last_position_var = tk.StringVar(value="None")
servo_a_live_var = tk.StringVar(value="90.0")
servo_b_live_var = tk.StringVar(value="90.0")
servo_z_live_var = tk.StringVar(value="90.0")

# =====================================================
# HEADER
# =====================================================
tk.Label(
    root,
    text="COBOT BOY - MINIMAL IHM TERMINAL",
    font=("Arial", 20, "bold")
).pack(pady=10)

# =====================================================
# DASHBOARD
# =====================================================
dash = tk.LabelFrame(root, text="Live State", padx=10, pady=10)
dash.pack(fill="x", padx=15, pady=5)

tk.Label(dash, text="Connection:", font=("Arial", 9, "bold")).grid(row=0, column=0, sticky="e")
tk.Label(dash, textvariable=connection_var, width=22, relief="sunken", anchor="w").grid(row=0, column=1, padx=5)

tk.Label(dash, text="State:", font=("Arial", 9, "bold")).grid(row=0, column=2, sticky="e")
tk.Label(dash, textvariable=robot_state_var, width=16, relief="sunken", anchor="w").grid(row=0, column=3, padx=5)

tk.Label(dash, text="Init:", font=("Arial", 9, "bold")).grid(row=0, column=4, sticky="e")
tk.Label(dash, textvariable=init_var, width=8, relief="sunken", anchor="w").grid(row=0, column=5, padx=5)

tk.Label(dash, text="Run:", font=("Arial", 9, "bold")).grid(row=0, column=6, sticky="e")
tk.Label(dash, textvariable=run_var, width=8, relief="sunken", anchor="w").grid(row=0, column=7, padx=5)

tk.Label(dash, text="Pause:", font=("Arial", 9, "bold")).grid(row=0, column=8, sticky="e")
tk.Label(dash, textvariable=pause_var, width=8, relief="sunken", anchor="w").grid(row=0, column=9, padx=5)

tk.Label(dash, text="Cycle:", font=("Arial", 9, "bold")).grid(row=1, column=0, sticky="e")
tk.Label(dash, textvariable=cycle_var, width=16, relief="sunken", anchor="w").grid(row=1, column=1, padx=5)

tk.Label(dash, text="Object:", font=("Arial", 9, "bold")).grid(row=1, column=2, sticky="e")
tk.Label(dash, textvariable=object_var, width=16, relief="sunken", anchor="w").grid(row=1, column=3, padx=5)

tk.Label(dash, text="Mode:", font=("Arial", 9, "bold")).grid(row=1, column=4, sticky="e")
tk.Label(dash, textvariable=mode_var, width=16, relief="sunken", anchor="w").grid(row=1, column=5, padx=5)

tk.Label(dash, text="Pieces:", font=("Arial", 9, "bold")).grid(row=1, column=6, sticky="e")
tk.Label(dash, textvariable=pieces_var, width=8, relief="sunken", anchor="w").grid(row=1, column=7, padx=5)

tk.Label(dash, text="Current:", font=("Arial", 9, "bold")).grid(row=1, column=8, sticky="e")
tk.Label(dash, textvariable=current_piece_var, width=8, relief="sunken", anchor="w").grid(row=1, column=9, padx=5)

tk.Label(dash, text="Speed:", font=("Arial", 9, "bold")).grid(row=2, column=0, sticky="e")
tk.Label(dash, textvariable=speed_var, width=16, relief="sunken", anchor="w").grid(row=2, column=1, padx=5)

# =====================================================
# MAIN AREA WITH TABS
# =====================================================
main = tk.Frame(root)
main.pack(fill="both", expand=True, padx=15, pady=10)

notebook = ttk.Notebook(main)
notebook.pack(side="left", fill="both", expand=True, padx=(0, 10))
notebook.bind("<<NotebookTabChanged>>", on_tab_changed)

# =====================================================
# TAB 1 - CONFIGURATION
# =====================================================
config_tab = tk.Frame(notebook)
notebook.add(config_tab, text="Tab 1 - Cycle Tests")

config_box = tk.LabelFrame(config_tab, text="Cycle Test Configuration", padx=20, pady=20)
config_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(config_box, text="Cycle").grid(row=0, column=0, sticky="e", padx=10, pady=10)
config_cycle = tk.IntVar(value=1)
ttk.Combobox(config_box, textvariable=config_cycle, values=[1, 2], width=20, state="readonly").grid(row=0, column=1)

tk.Label(config_box, text="Object").grid(row=1, column=0, sticky="e", padx=10, pady=10)
config_object = tk.StringVar(value="Gomme")
ttk.Combobox(config_box, textvariable=config_object, values=objects, width=20, state="readonly").grid(row=1, column=1)

tk.Label(config_box, text="Mode").grid(row=2, column=0, sticky="e", padx=10, pady=10)
config_mode = tk.StringVar(value="Normal")
ttk.Combobox(config_box, textvariable=config_mode, values=modes, width=20, state="readonly").grid(row=2, column=1)

tk.Label(config_box, text="Pieces").grid(row=3, column=0, sticky="e", padx=10, pady=10)
config_pieces = tk.IntVar(value=1)
tk.Spinbox(config_box, from_=1, to=10, textvariable=config_pieces, width=20).grid(row=3, column=1)

tk.Label(config_box, text="Speed factor").grid(row=4, column=0, sticky="e", padx=10, pady=10)
config_speed = tk.DoubleVar(value=1.00)
tk.Scale(config_box, from_=0.5, to=3.0, resolution=0.05, orient="horizontal", length=250, variable=config_speed).grid(row=4, column=1)

tk.Button(config_box, text="APPLY CONFIG", width=25, height=2, command=apply_config).grid(row=5, column=0, columnspan=2, pady=15)

tk.Button(config_box, text="GET STATUS", width=25, command=lambda: send_command("GET_STATUS")).grid(row=6, column=0, columnspan=2, pady=5)
tk.Button(config_box, text="GET CONFIG", width=25, command=lambda: send_command("GET_CONFIG")).grid(row=7, column=0, columnspan=2, pady=5)
tk.Button(config_box, text="RECONNECT", width=25, command=reconnect_arduino).grid(row=8, column=0, columnspan=2, pady=5)
tk.Button(config_box, text="TEST SERIAL LINK", width=25, command=test_serial_link).grid(row=9, column=0, columnspan=2, pady=5)
tk.Button(config_box, text="TEST INIT LED", width=25, command=test_init_led).grid(row=10, column=0, columnspan=2, pady=5)

# Fake OLED is only shown in Tab 1 because this tab is the cycle/config test page.
oled_box = tk.LabelFrame(config_box, text="Fake OLED Preview", padx=15, pady=15)
oled_box.grid(row=0, column=2, rowspan=11, padx=(35, 5), pady=5, sticky="n")

oled_canvas = tk.Canvas(
    oled_box,
    width=300,
    height=170,
    bg="black",
    highlightthickness=2,
    highlightbackground="#444444"
)
oled_canvas.pack()

# IHM cycle control buttons are only shown in Tab 1 because this tab is for cycle tests.
ihm_box = tk.LabelFrame(config_box, text="Cycle Control Buttons", padx=16, pady=16)
ihm_box.grid(row=0, column=3, rowspan=11, padx=(25, 5), pady=5, sticky="n")

tk.Button(
    ihm_box,
    text="INIT",
    width=16,
    height=2,
    bg="#72d572",
    font=("Arial", 12, "bold"),
    command=ihm_init
).pack(pady=8)

tk.Button(
    ihm_box,
    text="PLAY / START",
    width=16,
    height=2,
    bg="#75bfff",
    font=("Arial", 12, "bold"),
    command=ihm_play
).pack(pady=8)

tk.Button(
    ihm_box,
    text="PAUSE",
    width=16,
    height=2,
    bg="#ffd966",
    font=("Arial", 12, "bold"),
    command=ihm_pause
).pack(pady=8)

tk.Label(
    ihm_box,
    text="Use INIT first.\nThen PLAY to run.\nPAUSE toggles pause/resume.",
    fg="red",
    font=("Arial", 10, "bold"),
    justify="left"
).pack(pady=10)

# =====================================================
# TAB 2 - SINGLE SERVO TESTS
# =====================================================
servo_tab = tk.Frame(notebook)
notebook.add(servo_tab, text="Tab 2 - Servo Tests")

servo_box = tk.LabelFrame(servo_tab, text="Single Servo Tests", padx=20, pady=20)
servo_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(
    servo_box,
    text="When this tab opens, the Arduino is sent to REPOS first.\nUse these controls only when the robot is not running.",
    fg="red",
    font=("Arial", 10, "bold"),
    justify="left"
).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 15))

def add_servo_control(row, servo_name, label):
    var = tk.DoubleVar(value=90.0)

    tk.Label(servo_box, text=label, font=("Arial", 11, "bold")).grid(row=row, column=0, sticky="e", padx=10, pady=10)

    scale = tk.Scale(
        servo_box,
        from_=0,
        to=180,
        resolution=1,
        orient="horizontal",
        length=260,
        variable=var
    )
    scale.grid(row=row, column=1, padx=10, pady=10)

    tk.Button(
        servo_box,
        text="APPLY",
        width=10,
        command=lambda: set_servo(servo_name, var.get())
    ).grid(row=row, column=2, padx=5)

    step_box = tk.Frame(servo_box)
    step_box.grid(row=row, column=3, padx=5)

    tk.Button(step_box, text="-5", width=5, command=lambda: step_servo(servo_name, -5)).pack(side="left", padx=2)
    tk.Button(step_box, text="+5", width=5, command=lambda: step_servo(servo_name, 5)).pack(side="left", padx=2)

    return var

servo_a_var = add_servo_control(1, "A", "Servo A")
servo_b_var = add_servo_control(2, "B", "Servo B")
servo_z_var = add_servo_control(3, "Z", "Servo Z")

tk.Button(
    servo_box,
    text="GO TO REPOS",
    width=25,
    height=2,
    command=go_repos_from_terminal
).grid(row=4, column=0, columnspan=4, pady=20)

# =====================================================
# TAB 3 - BUTTON AND LED TESTS
# =====================================================
button_led_tab = tk.Frame(notebook)
notebook.add(button_led_tab, text="Tab 3 - Button / LED Tests")

button_led_box = tk.LabelFrame(button_led_tab, text="Button and LED Tests", padx=20, pady=20)
button_led_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(
    button_led_box,
    text="When this tab opens, the Arduino is sent to REPOS and button test mode is activated.\nPress physical INIT / START / PAUSE: the terminal should show RECEIVED.",
    fg="red",
    font=("Arial", 10, "bold"),
    justify="left"
).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 15))

button_test_status_var = tk.StringVar(value="Button test inactive.")

tk.Label(button_led_box, text="Physical button test", font=("Arial", 12, "bold")).grid(row=1, column=0, sticky="w", pady=10)
tk.Label(
    button_led_box,
    textvariable=button_test_status_var,
    width=45,
    relief="sunken",
    anchor="w",
    font=("Arial", 10)
).grid(row=1, column=1, columnspan=3, sticky="w", padx=10)

tk.Button(button_led_box, text="BUTTON TEST ON", width=18, command=start_button_test_mode).grid(row=2, column=1, padx=5, pady=5, sticky="w")
tk.Button(button_led_box, text="BUTTON TEST OFF", width=18, command=stop_button_test_mode).grid(row=2, column=2, padx=5, pady=5, sticky="w")

tk.Label(button_led_box, text="LED tests", font=("Arial", 12, "bold")).grid(row=3, column=0, sticky="w", pady=(25, 10))

def add_led_control(row, led_name, label):
    tk.Label(button_led_box, text=label, font=("Arial", 11, "bold")).grid(row=row, column=0, sticky="e", padx=10, pady=8)
    tk.Button(button_led_box, text="ON", width=10, command=lambda: led_set(led_name, True)).grid(row=row, column=1, padx=5, pady=8)
    tk.Button(button_led_box, text="OFF", width=10, command=lambda: led_set(led_name, False)).grid(row=row, column=2, padx=5, pady=8)
    tk.Button(button_led_box, text="PULSE", width=10, command=lambda: led_pulse(led_name)).grid(row=row, column=3, padx=5, pady=8)

add_led_control(4, "INIT", "INIT LED")
add_led_control(5, "RUN", "RUN LED")
add_led_control(6, "PAUSE", "PAUSE LED")
add_led_control(7, "POWER", "POWER LED")

tk.Button(
    button_led_box,
    text="RESTORE AUTOMATIC LED STATE",
    width=28,
    height=2,
    command=led_auto
).grid(row=8, column=0, columnspan=4, pady=(20, 5))

tk.Button(
    button_led_box,
    text="GO TO REPOS",
    width=25,
    height=2,
    command=go_repos_from_terminal
).grid(row=9, column=0, columnspan=4, pady=5)

# =====================================================
# TAB 4 - POSITION CHECK
# =====================================================
position_tab = tk.Frame(notebook)
notebook.add(position_tab, text="Tab 4 - Position Check")

position_box = tk.LabelFrame(position_tab, text="Position Tests", padx=16, pady=16)
position_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(
    position_box,
    text=(
        "Test one stored position without running the full cycle.\n"
        "Choose cycle, object and one of the four named positions."
    ),
    fg="red",
    font=("Arial", 10, "bold"),
    justify="left"
).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 14))

position_cycle_var = tk.IntVar(value=1)
position_object_var = tk.StringVar(value=objects[0])
position_name_var = tk.StringVar(value=position_names[0])
position_command_var = tk.StringVar(value="GOTO_POSITION 0 0 0")
position_status_var = tk.StringVar(value="Select a cycle, object and position.")

position_cycle_var.trace_add("write", update_position_command_preview)
position_object_var.trace_add("write", update_position_command_preview)
position_name_var.trace_add("write", update_position_command_preview)

left_panel = tk.Frame(position_box)
left_panel.grid(row=1, column=0, sticky="nw", padx=(0, 28))

right_panel = tk.LabelFrame(position_box, text="Live position returned by Arduino", padx=12, pady=10)
right_panel.grid(row=1, column=1, sticky="nw", padx=(10, 0))

tk.Label(left_panel, text="Cycle", font=("Arial", 11, "bold")).grid(row=0, column=0, sticky="e", padx=10, pady=8)
ttk.Combobox(
    left_panel,
    textvariable=position_cycle_var,
    values=[1, 2],
    width=22,
    state="readonly"
).grid(row=0, column=1, sticky="w", padx=10, pady=8)

tk.Label(left_panel, text="Object", font=("Arial", 11, "bold")).grid(row=1, column=0, sticky="e", padx=10, pady=8)
ttk.Combobox(
    left_panel,
    textvariable=position_object_var,
    values=objects,
    width=22,
    state="readonly"
).grid(row=1, column=1, sticky="w", padx=10, pady=8)

tk.Label(left_panel, text="Position", font=("Arial", 11, "bold")).grid(row=2, column=0, sticky="e", padx=10, pady=8)
ttk.Combobox(
    left_panel,
    textvariable=position_name_var,
    values=position_names,
    width=22,
    state="readonly"
).grid(row=2, column=1, sticky="w", padx=10, pady=8)

tk.Label(left_panel, text="Command", font=("Arial", 11, "bold")).grid(row=3, column=0, sticky="e", padx=10, pady=8)
tk.Entry(
    left_panel,
    textvariable=position_command_var,
    width=34,
    state="readonly"
).grid(row=3, column=1, sticky="w", padx=10, pady=8)

tk.Label(left_panel, text="Status", font=("Arial", 11, "bold")).grid(row=4, column=0, sticky="e", padx=10, pady=8)
tk.Label(
    left_panel,
    textvariable=position_status_var,
    width=46,
    relief="sunken",
    anchor="w"
).grid(row=4, column=1, sticky="w", padx=10, pady=8)

quick_box = tk.LabelFrame(left_panel, text="Quick position buttons", padx=10, pady=10)
quick_box.grid(row=5, column=0, columnspan=2, sticky="w", padx=10, pady=(16, 10))

def add_position_quick_button(position_name, row, column):
    tk.Button(
        quick_box,
        text=position_name,
        width=14,
        command=lambda name=position_name: (
            position_name_var.set(name),
            update_position_command_preview()
        )
    ).grid(row=row, column=column, padx=5, pady=5)

add_position_quick_button("Entree", 0, 0)
add_position_quick_button("Machine A", 0, 1)
add_position_quick_button("Machine B", 0, 2)
add_position_quick_button("Sortie", 0, 3)

button_row = tk.Frame(left_panel)
button_row.grid(row=6, column=0, columnspan=2, sticky="w", padx=10, pady=(18, 8))

tk.Button(
    button_row,
    text="GO TO POSITION",
    width=22,
    height=2,
    bg="#90ee90",
    font=("Arial", 11, "bold"),
    command=go_to_selected_position
).pack(side="left", padx=(0, 12))

tk.Button(
    button_row,
    text="GO TO REPOS",
    width=18,
    height=2,
    command=go_to_repos_from_position_tab
).pack(side="left", padx=12)

tk.Button(
    button_row,
    text="GET STATUS",
    width=16,
    height=2,
    command=lambda: send_command("GET_STATUS")
).pack(side="left", padx=12)

tk.Label(right_panel, text="Last position", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="e", padx=5, pady=5)
tk.Label(right_panel, textvariable=last_position_var, width=20, relief="sunken", anchor="w").grid(row=0, column=1, padx=5, pady=5)

tk.Label(right_panel, text="X", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="e", padx=5, pady=5)
tk.Label(right_panel, textvariable=current_x_var, width=20, relief="sunken", anchor="w").grid(row=1, column=1, padx=5, pady=5)

tk.Label(right_panel, text="Y", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="e", padx=5, pady=5)
tk.Label(right_panel, textvariable=current_y_var, width=20, relief="sunken", anchor="w").grid(row=2, column=1, padx=5, pady=5)

tk.Label(right_panel, text="Z", font=("Arial", 10, "bold")).grid(row=3, column=0, sticky="e", padx=5, pady=5)
tk.Label(right_panel, textvariable=current_z_var, width=20, relief="sunken", anchor="w").grid(row=3, column=1, padx=5, pady=5)

tk.Label(right_panel, text="Servo A", font=("Arial", 10, "bold")).grid(row=4, column=0, sticky="e", padx=5, pady=5)
tk.Label(right_panel, textvariable=servo_a_live_var, width=20, relief="sunken", anchor="w").grid(row=4, column=1, padx=5, pady=5)

tk.Label(right_panel, text="Servo B", font=("Arial", 10, "bold")).grid(row=5, column=0, sticky="e", padx=5, pady=5)
tk.Label(right_panel, textvariable=servo_b_live_var, width=20, relief="sunken", anchor="w").grid(row=5, column=1, padx=5, pady=5)

tk.Label(right_panel, text="Servo Z", font=("Arial", 10, "bold")).grid(row=6, column=0, sticky="e", padx=5, pady=5)
tk.Label(right_panel, textvariable=servo_z_live_var, width=20, relief="sunken", anchor="w").grid(row=6, column=1, padx=5, pady=5)

update_position_command_preview()

# =====================================================
# LOG
# =====================================================
log_frame = tk.LabelFrame(root, text="Serial Log", padx=5, pady=5)
log_frame.pack(fill="both", expand=False, padx=15, pady=10)

log_box = tk.Text(log_frame, height=9)
log_box.pack(fill="both", expand=True)

# =====================================================
# STARTUP
# =====================================================
draw_oled()
connect_arduino()
update_dashboard()

def on_close():
    try:
        send_command("BUTTON_TEST_OFF")
        send_command("LED_AUTO")
        send_command("TERMINAL_OFF")
        time.sleep(0.1)
    except Exception:
        pass
    disconnect_arduino()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()