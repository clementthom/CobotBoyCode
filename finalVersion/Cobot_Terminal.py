import os
import sys
import tkinter as tk
from tkinter import ttk
import threading
import time
import serial
import serial.tools.list_ports

# =====================================================
# CONNECTION SETTINGS (CROSS-PLATFORM)
# =====================================================
# Use "AUTO" for automatic port detection (recommended for cross-platform use)
SERIAL_PORT = "AUTO"
BAUD_RATE = 9600

# Command sent when the terminal must force the gripper open.
GRIPPER_OPEN_COMMAND = "GRIPPER_OPEN"
GRIPPER_CLOSE_COMMAND = "GRIPPER_CLOSE"
GRIPPER_STOP_COMMAND = "GRIPPER_STOP"
GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT_COMMAND = "GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT"
GRIPPER_AUTO_TEST_COMMAND = "GRIPPER_AUTO_TEST"

# =====================================================
# CONSTANTS
# =====================================================
REAL_OBJECTS = ["Gomme", "Gobelet", "Cylindre"]
OBJECT_CHOICES = ["Gomme", "Gobelet", "Cylindre", "Aucun"]
MODES = ["ÉCO", "Normal", "RAPIDE"]
CYCLES = ["Cycle 1", "Cycle 2"]
PREHENSEURS = ["Préhenseur 1 - offset", "Préhenseur 2 - centré"]
POSITION_NAMES = ["Entrée", "Machine A", "Machine B", "Sortie"]
POSITION_CODE_NAMES = [
    "POS_convoyeurEntree",
    "POS_machineA",
    "POS_machineB",
    "POS_convoyeurSortie",
]

SELECTED_BG = "#b8b8b8"
NORMAL_BG = "#f0f0f0"

# =====================================================
# GLOBAL STATE
# =====================================================
arduino = None
reader_running = False

state = {
    "connection": "DÉCONNECTÉ",
    "serial_link": "NON TESTÉ",
    "robot_state": "NOT_INIT",
    "init": 0,
    "initializing": 0,
    "run": 0,
    "pause": 0,
    "cycle_index": 0,
    "cycle_name": "Cycle 1",
    "prehenseur_index": 0,
    "prehenseur_name": "Préhenseur 1 - offset",
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
    "last_cycle_time": "--",
}

# =====================================================
# CROSS-PLATFORM SERIAL PORT HANDLING
# =====================================================
def find_arduino_port():
    """Detect Arduino port across Windows, Linux, and macOS."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        if any(keyword in desc for keyword in ["arduino", "mega", "ch340", "usb serial", "ttyacm", "ttyusb"]):
            return port.device
    return None

def resolve_serial_port():
    """Resolve the serial port, using auto-detection if SERIAL_PORT is 'AUTO'."""
    if SERIAL_PORT.upper() != "AUTO":
        return SERIAL_PORT
    port = find_arduino_port()
    if port:
        return port
    print("[WARNING] No Arduino port detected. Set SERIAL_PORT manually or connect an Arduino.")
    return None

# =====================================================
# SERIAL CONNECTION FUNCTIONS
# =====================================================
def connect_arduino():
    global arduino, reader_running

    try:
        port = resolve_serial_port()
        if port is None:
            raise RuntimeError("No Arduino port found. Check connection and permissions.")

        arduino = serial.Serial(port, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        arduino.reset_input_buffer()

        arduino.write(b"TERMINAL_ON\n")
        arduino.flush()
        time.sleep(0.1)
        arduino.write(b"PING\n")
        arduino.flush()

        state["connection"] = f"CONNECTED {port}"
        state["serial_link"] = "PORT OPEN - TEST IN PROGRESS"
        update_dashboard()

        reader_running = True
        threading.Thread(target=read_arduino, daemon=True).start()

        log(f"[APP] Connected to Arduino on {port}")
        send_command("GET_STATUS")
        send_command("GET_CONFIG")

    except Exception as e:
        state["connection"] = "CONNECTION ERROR"
        state["serial_link"] = "CONNECTION FAILED"
        update_dashboard()
        log(f"[APP] Connection error: {repr(e)}")

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
    state["serial_link"] = "DISCONNECTED"
    update_dashboard()
    log("[APP] Disconnected.")

def reconnect_arduino():
    disconnect_arduino()
    connect_arduino()

def send_command(cmd):
    log(f"[APP → ARDUINO] {cmd}")

    if arduino is None or not arduino.is_open:
        state["serial_link"] = "NOT CONNECTED"
        update_dashboard()
        log("[APP] Arduino not connected.")
        return

    try:
        arduino.write((cmd.strip() + "\n").encode("utf-8"))
        arduino.flush()
    except Exception as e:
        state["serial_link"] = "SEND ERROR"
        update_dashboard()
        log(f"[APP] Send error: {str(e)}")

def read_arduino():
    global reader_running
    while reader_running:
        try:
            raw = arduino.readline().decode(errors="ignore").strip()
            if raw:
                root.after(0, lambda line=raw: handle_arduino_line(line))
        except Exception as e:
            root.after(0, lambda err=e: log(f"[APP] Read error: {str(err)}"))
            break

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
    log(f"[ARDUINO] {line}")

    if line == "PONG":
        state["serial_link"] = "OK - PONG RECEIVED"
        try:
            serial_link_var.set(state["serial_link"])
        except Exception:
            pass
        return

    msg_type, data = parse_semicolon_line(line)

    if msg_type == "STATUS":
        state["robot_state"] = data.get("STATE", state["robot_state"])
        state["init"] = int(data.get("INIT", state["init"]))
        state["initializing"] = int(data.get("INITIALIZING", state.get("initializing", 0)))
        state["run"] = int(data.get("RUN", state["run"]))
        state["pause"] = int(data.get("PAUSE", state["pause"]))

        state["cycle_index"] = int(data.get("CYCLE", state["cycle_index"]))
        state["cycle_name"] = data.get("CYCLE_NAME", CYCLES[state["cycle_index"]])

        state["prehenseur_index"] = int(data.get("PREHENSEUR", state["prehenseur_index"]))
        state["prehenseur_name"] = data.get("PREHENSEUR_NAME", prehenseur_name_from_index(state["prehenseur_index"]))

        state["object_index"] = int(data.get("OBJECT", state["object_index"]))
        state["object_name"] = data.get("OBJECT_NAME", object_name_from_index(state["object_index"]))

        state["mode_index"] = int(data.get("MODE", state["mode_index"]))
        state["mode_name"] = data.get("MODE_NAME", MODES[state["mode_index"]])

        state["pieces"] = int(data.get("PIECES", state["pieces"]))
        state["current_piece"] = int(data.get("CURRENT_PIECE", state["current_piece"]))
        state["speed_factor"] = float(data.get("SPEED_FACTOR", state["speed_factor"]))
        state["servo_a"] = float(data.get("SERVO_A", state["servo_a"]))
        state["servo_b"] = float(data.get("SERVO_B", state["servo_b"]))
        state["servo_z"] = float(data.get("SERVO_Z", state["servo_z"]))

    elif msg_type == "CONFIG":
        state["cycle_index"] = int(data.get("CYCLE", state["cycle_index"]))
        state["cycle_name"] = CYCLES[state["cycle_index"]]
        state["prehenseur_index"] = int(data.get("PREHENSEUR", state["prehenseur_index"]))
        state["prehenseur_name"] = prehenseur_name_from_index(state["prehenseur_index"])
        state["object_index"] = int(data.get("OBJECT", state["object_index"]))
        state["object_name"] = object_name_from_index(state["object_index"])
        state["mode_index"] = int(data.get("MODE", state["mode_index"]))
        state["mode_name"] = MODES[state["mode_index"]]
        state["pieces"] = int(data.get("PIECES", state["pieces"]))
        state["speed_factor"] = float(data.get("SPEED_FACTOR", state["speed_factor"]))

    elif msg_type == "OK":
        ok_text = line.split(";", 1)[1] if ";" in line else ""

        if ok_text.startswith("RUN_CYCLE_DONE"):
            seconds = data.get("CYCLE_TIME_S")
            millis = data.get("CYCLE_TIME_MS")
            if seconds is not None:
                state["last_cycle_time"] = f"{float(seconds):.2f} s"
            elif millis is not None:
                state["last_cycle_time"] = f"{int(millis) / 1000.0:.2f} s"
            else:
                state["last_cycle_time"] = "cycle terminé"
            log(f"[CYCLE] Cycle time: {state['last_cycle_time']}")

        elif ok_text.startswith("GOTO_POSITION_DONE"):
            position_index = int(data.get("POSITION", "0"))
            cycle_index = int(data.get("CYCLE", state["cycle_index"]))
            prehenseur_index = int(data.get("PREHENSEUR", state["prehenseur_index"]))
            object_index = int(data.get("OBJECT", state["object_index"]))
            state["cycle_index"] = cycle_index
            state["cycle_name"] = CYCLES[cycle_index]
            state["prehenseur_index"] = prehenseur_index
            state["prehenseur_name"] = prehenseur_name_from_index(prehenseur_index)
            state["object_index"] = object_index
            state["object_name"] = object_name_from_index(object_index)
            state["last_position"] = POSITION_NAMES[position_index] if 0 <= position_index < len(POSITION_NAMES) else data.get("POSITION_NAME", "?")
            state["current_x"] = data.get("X", state["current_x"])
            state["current_y"] = data.get("Y", state["current_y"])
            state["current_z"] = data.get("Z", state["current_z"])
            position_status_var.set(f"Arrived: {state['cycle_name']} / {state['object_name']} / {state['last_position']}")

        elif ok_text.startswith("GOTO_XYZ_DONE") or ok_text.startswith("GO_REPOS_DONE"):
            state["last_position"] = data.get("POSITION_NAME", state["last_position"])
            state["current_x"] = data.get("X", state["current_x"])
            state["current_y"] = data.get("Y", state["current_y"])
            state["current_z"] = data.get("Z", state["current_z"])
            xyz_status_var.set(f"Arrived: X={state['current_x']} / Y={state['current_y']} / Z={state['current_z']}")

        elif ok_text.startswith("GRIPPER_OPEN"):
            log("[GRIPPER] Gripper open confirmed by Arduino")
            try:
                gripper_status_var.set("Gripper open confirmed")
            except Exception:
                pass

        elif ok_text.startswith("GRIPPER_CLOSE"):
            log("[GRIPPER] Gripper close confirmed by Arduino")
            try:
                gripper_status_var.set("Gripper close confirmed")
            except Exception:
                pass

        elif ok_text.startswith("GRIPPER_STOP"):
            log("[GRIPPER] Gripper stop confirmed by Arduino")
            try:
                gripper_status_var.set("Gripper stop confirmed")
            except Exception:
                pass

        elif ok_text.startswith("GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT"):
            log("[GRIPPER TEST] Test position reached: Cycle 1 / Entrée / Z=20")
            try:
                gripper_status_var.set("Test position reached: Cycle 1 / Entrée / Z=20")
            except Exception:
                pass

        elif ok_text.startswith("GRIPPER_AUTO_TEST"):
            log("[GRIPPER TEST] Auto gripper test completed")
            try:
                gripper_status_var.set("Auto test completed")
            except Exception:
                pass

    elif msg_type == "XYZ_OK":
        parts = line.split()
        if len(parts) >= 4:
            state["current_x"] = parts[1]
            state["current_y"] = parts[2]
            state["current_z"] = parts[3]
            state["last_position"] = "Manual XYZ"
            xyz_status_var.set(f"Arrived: X={state['current_x']} / Y={state['current_y']} / Z={state['current_z']}")

    elif msg_type == "EVENT":
        if data.get("TYPE") == "BUTTON":
            name = data.get("NAME", "?")
            status = data.get("STATUS", "RECEIVED")
            message = f"Physical button {name}: {status}"
            log(f"[BUTTON TEST] {message}")
            button_test_status_var.set(message)

    update_dashboard()

# =====================================================
# INDEX HELPERS
# =====================================================
def cycle_to_index(value):
    return int(str(value).replace("Cycle", "").strip()) - 1

def prehenseur_to_index(value):
    if str(value).startswith("Préhenseur 2") or str(value).startswith("Prehenseur 2"):
        return 1
    return 0

def prehenseur_name_from_index(index):
    try:
        index = int(index)
    except Exception:
        return PREHENSEURS[0]
    if 0 <= index < len(PREHENSEURS):
        return PREHENSEURS[index]
    return "Préhenseur ?"

def object_to_index(value):
    if value == "Aucun":
        return -1
    return REAL_OBJECTS.index(value)

def object_name_from_index(index):
    try:
        index = int(index)
    except Exception:
        return "Aucun"
    if index < 0:
        return "Aucun"
    if 0 <= index < len(REAL_OBJECTS):
        return REAL_OBJECTS[index]
    return "Objet ?"

def mode_to_index(value):
    return MODES.index(value)

# =====================================================
# GUI ACTIONS
# =====================================================
def set_cycle(value):
    idx = cycle_to_index(value)
    state["cycle_index"] = idx
    state["cycle_name"] = CYCLES[idx]
    send_command(f"SET_CYCLE {idx}")
    update_position_command_preview()
    update_dashboard()

def set_prehenseur(value):
    idx = prehenseur_to_index(value)
    state["prehenseur_index"] = idx
    state["prehenseur_name"] = PREHENSEURS[idx]
    send_command(f"SET_PREHENSEUR {idx}")
    update_position_command_preview()
    update_dashboard()

def set_object(value):
    idx = object_to_index(value)
    state["object_index"] = idx
    state["object_name"] = value
    send_command(f"SET_OBJECT {idx}")
    update_position_command_preview()
    update_dashboard()

def set_mode(value):
    idx = mode_to_index(value)
    state["mode_index"] = idx
    state["mode_name"] = value
    send_command(f"SET_MODE {idx}")
    update_dashboard()

def set_pieces_from_spinbox():
    try:
        pieces_value = int(config_pieces.get())
    except Exception:
        pieces_value = 1
    pieces_value = max(1, min(10, pieces_value))
    state["pieces"] = pieces_value
    send_command(f"SET_PIECES {pieces_value}")
    update_dashboard()

def apply_config_silently():
    send_command(f"SET_CYCLE {state['cycle_index']}")
    send_command(f"SET_OBJECT {state['object_index']}")
    send_command(f"SET_MODE {state['mode_index']}")
    set_pieces_from_spinbox()

def ihm_init():
    log("[BUTTON] INIT clicked")
    apply_config_silently()
    root.after(250, lambda: send_command("INITIALIZE"))

def ihm_play():
    log("[BUTTON] START clicked")
    send_command(f"SET_PREHENSEUR {state['prehenseur_index']}")
    state["last_cycle_time"] = "In progress..."
    update_dashboard()
    root.after(150, lambda: send_command("RUN_CYCLE"))

def ihm_pause():
    log("[BUTTON] PAUSE clicked")
    send_command("PAUSE_TOGGLE")

def test_serial_link():
    state["serial_link"] = "TEST IN PROGRESS"
    update_dashboard()
    log("[TEST] Sending PING")
    send_command("PING")

def start_button_test_mode():
    log("[BUTTON TEST] Button test mode ON. Press INIT / START / PAUSE.")
    button_test_status_var.set("Button test active. Press a physical button.")
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

def go_repos_from_terminal():
    log("[SERVO TEST] Return to REPOS")
    send_command("GO_REPOS")

def open_gripper_from_terminal(reason=""):
    if reason:
        log(f"[GRIPPER SAFETY] Gripper open - {reason}")
    else:
        log("[GRIPPER SAFETY] Gripper open")
    send_command(GRIPPER_OPEN_COMMAND)

def close_gripper_from_terminal(reason=""):
    if reason:
        log(f"[GRIPPER] Gripper close - {reason}")
    else:
        log("[GRIPPER] Gripper close")
    send_command(GRIPPER_CLOSE_COMMAND)

def stop_gripper_from_terminal(reason=""):
    if reason:
        log(f"[GRIPPER] Gripper stop - {reason}")
    else:
        log("[GRIPPER] Gripper stop")
    send_command(GRIPPER_STOP_COMMAND)

def go_to_cycle1_entree_test_height():
    prehenseur_index = prehenseur_to_index(gripper_prehenseur_var.get())
    object_index = object_to_index(gripper_object_var.get())
    send_command(f"SET_PREHENSEUR {prehenseur_index}")
    send_command(f"SET_OBJECT {object_index}")
    gripper_status_var.set(f"Requested move: {PREHENSEURS[prehenseur_index]} / {gripper_object_var.get()} / Cycle 1 Entrée / Z=20")
    log(f"[GRIPPER TEST] Go to Cycle 1 Entrée at test height 20 with prehenseur {prehenseur_index} and object {object_index}")
    send_command(GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT_COMMAND)

def run_gripper_auto_test(object_index, object_name):
    prehenseur_index = prehenseur_to_index(gripper_prehenseur_var.get())
    send_command(f"SET_PREHENSEUR {prehenseur_index}")
    gripper_status_var.set(f"Auto test requested: {PREHENSEURS[prehenseur_index]} / {object_name} at Cycle 1 Entrée")
    log(f"[GRIPPER TEST] Auto gripper test: prehenseur {prehenseur_index}, object {object_name}: PID grab, lift, place, open")
    send_command(f"{GRIPPER_AUTO_TEST_COMMAND} {prehenseur_index} {object_index}")

def manual_gripper_close():
    gripper_status_var.set("Manual command: gripper close")
    close_gripper_from_terminal("tab 5 - manual test")

def manual_gripper_open():
    gripper_status_var.set("Manual command: gripper open")
    open_gripper_from_terminal("tab 5 - manual test")

def manual_gripper_stop():
    gripper_status_var.set("Manual command: gripper stop")
    stop_gripper_from_terminal("tab 5 - manual test")

def set_servo(servo_name, value):
    value = max(0.0, min(180.0, float(value)))
    send_command(f"SERVO_SET {servo_name} {value:.1f}")

def step_servo(servo_name, delta):
    send_command(f"SERVO_STEP {servo_name} {float(delta):.1f}")

def go_to_xyz_from_servo_tab():
    try:
        x = float(xyz_x_var.get().replace(",", ".").strip())
        y = float(xyz_y_var.get().replace(",", ".").strip())
        z = float(xyz_z_var.get().replace(",", ".").strip())
        xyz_status_var.set(f"Requested move: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
        log(f"[XYZ] Sending command: GOTO_XYZ {x:.1f} {y:.1f} {z:.1f}")
        send_command(f"GOTO_XYZ {x:.1f} {y:.1f} {z:.1f}")
    except Exception:
        xyz_status_var.set("Error: Enter numeric values for X, Y, and Z.")

def update_position_command_preview(*args):
    try:
        cycle_index = cycle_to_index(position_cycle_var.get())
        prehenseur_index = prehenseur_to_index(position_prehenseur_var.get())
        object_index = object_to_index(position_object_var.get())
        position_name = position_name_var.get()
        position_index = POSITION_NAMES.index(position_name)
        position_command_var.set(f"GOTO_POSITION {cycle_index} {prehenseur_index} {object_index} {position_index}")
        position_status_var.set(
            f"Selection: Cycle {cycle_index + 1} / {position_prehenseur_var.get()} / {position_object_var.get()} / {position_name} ({POSITION_CODE_NAMES[position_index]})"
        )
    except Exception:
        pass

def go_to_selected_position():
    try:
        cycle_index = cycle_to_index(position_cycle_var.get())
        prehenseur_index = prehenseur_to_index(position_prehenseur_var.get())
        object_index = object_to_index(position_object_var.get())
        position_name = position_name_var.get()
        position_index = POSITION_NAMES.index(position_name)
        position_command_var.set(f"GOTO_POSITION {cycle_index} {prehenseur_index} {object_index} {position_index}")
        position_status_var.set(f"Moving to Cycle {cycle_index + 1} / {position_prehenseur_var.get()} / {position_object_var.get()} / {position_name}")
        send_command(f"GOTO_POSITION {cycle_index} {prehenseur_index} {object_index} {position_index}")
    except Exception as e:
        position_status_var.set(f"Position command error: {str(e)}")
        log(f"[POSITION TEST] Error: {str(e)}")

def go_to_repos_from_position_tab():
    position_status_var.set("Return to REPOS")
    send_command("GO_REPOS")

def on_tab_changed(event):
    selected_tab = notebook.index(notebook.select())

    if selected_tab == 0:
        stop_button_test_mode()
        led_auto()
        root.after(150, lambda: open_gripper_from_terminal("tab 1 - cycle tests"))

    elif selected_tab == 1:
        stop_button_test_mode()
        led_auto()
        go_repos_from_terminal()

    elif selected_tab == 2:
        led_auto()
        start_button_test_mode()

    elif selected_tab == 3:
        stop_button_test_mode()
        led_auto()
        root.after(150, lambda: open_gripper_from_terminal("tab 4 - position tests"))

    elif selected_tab == 4:
        stop_button_test_mode()
        led_auto()
        gripper_status_var.set("Tab 5 active: choose a gripper test.")

    else:
        stop_button_test_mode()
        led_auto()

# =====================================================
# GUI DISPLAY
# =====================================================
def sync_servo_sliders_from_state():
    try:
        servo_a_var.set(state["servo_a"])
        servo_b_var.set(state["servo_b"])
        servo_z_var.set(state["servo_z"])
    except Exception:
        pass

def update_dashboard():
    connection_var.set(state["connection"])
    robot_state_var.set(state["robot_state"])
    init_var.set(str(state["init"]))
    run_var.set(str(state["run"]))
    pause_var.set(str(state["pause"]))
    cycle_var.set(state["cycle_name"])
    prehenseur_var.set(state["prehenseur_name"])
    object_var.set(state["object_name"])
    mode_var.set(state["mode_name"])
    pieces_var.set(str(state["pieces"]))
    current_piece_var.set(str(state["current_piece"]))
    speed_var.set(f"{state['speed_factor']:.2f}")
    cycle_time_var.set(str(state.get("last_cycle_time", "--")))
    try:
        serial_link_var.set(state.get("serial_link", "NON TESTÉ"))
    except Exception:
        pass

    current_x_var.set(str(state.get("current_x", "?")))
    current_y_var.set(str(state.get("current_y", "?")))
    current_z_var.set(str(state.get("current_z", "?")))
    last_position_var.set(str(state.get("last_position", "None")))
    servo_a_live_var.set(f"{state['servo_a']:.1f}")
    servo_b_live_var.set(f"{state['servo_b']:.1f}")
    servo_z_live_var.set(f"{state['servo_z']:.1f}")

    sync_servo_sliders_from_state()
    draw_oled()

def draw_oled():
    try:
        oled_canvas.delete("all")
        oled_canvas.create_rectangle(0, 0, 300, 170, fill="black", outline="#444444")
        oled_text(0, 0, "PYTHON TERMINAL STATUS")
        oled_text(0, 16, "----------------------")
        oled_text(0, 34, f"Cycle: {state['cycle_name']}")
        oled_text(0, 50, f"Gripper: {state['prehenseur_name'][:16]}")
        oled_text(0, 66, f"Object: {state['object_name']}")
        oled_text(0, 82, f"Pieces: {state['pieces']}")
        oled_text(0, 98, f"State: {state['robot_state']}")
        oled_text(0, 114, f"X:{state['current_x']} Y:{state['current_y']} Z:{state['current_z']}")
        oled_text(0, 130, f"Init:{state['init']} Initializing:{state['initializing']}")
        oled_text(0, 146, f"Run:{state['run']} Pause:{state['pause']} Piece:{state['current_piece']}/{state['pieces']}")
    except Exception:
        pass

def oled_text(x, y, text):
    oled_canvas.create_text(x, y, text=text, fill="white", anchor="nw", font=("Consolas", 9))

def log(message):
    try:
        log_box.insert(tk.END, message + "\n")
        log_box.see(tk.END)
    except Exception:
        pass

# =====================================================
# WINDOW
# =====================================================
root = tk.Tk()
root.title("CYCLES_Terminal_V2 (Cross-Platform)")
root.geometry("1200x800")
root.minsize(1200, 800)

# =====================================================
# VARIABLES
# =====================================================
connection_var = tk.StringVar(value="DISCONNECTED")
robot_state_var = tk.StringVar(value="NOT_INIT")
init_var = tk.StringVar(value="0")
run_var = tk.StringVar(value="0")
pause_var = tk.StringVar(value="0")
cycle_var = tk.StringVar(value="Cycle 1")
prehenseur_var = tk.StringVar(value="Préhenseur 1 - offset")
object_var = tk.StringVar(value="Gomme")
mode_var = tk.StringVar(value="Normal")
pieces_var = tk.StringVar(value="1")
current_piece_var = tk.StringVar(value="0")
speed_var = tk.StringVar(value="1.00")
cycle_time_var = tk.StringVar(value="--")
serial_link_var = tk.StringVar(value="NOT TESTED")
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
tk.Label(root, text="COBOT BOY - TERMINAL IHM (Cross-Platform)", font=("Arial", 18, "bold")).pack(pady=6)

# =====================================================
# DASHBOARD
# =====================================================
dash = tk.LabelFrame(root, text="Real-Time Status", padx=10, pady=10)
dash.pack(fill="x", padx=12, pady=4)

labels = [
    ("Connection:", connection_var), ("State:", robot_state_var), ("Init:", init_var),
    ("Run:", run_var), ("Pause:", pause_var), ("Cycle:", cycle_var),
    ("Gripper:", prehenseur_var), ("Object:", object_var), ("Mode:", mode_var), ("Pieces:", pieces_var),
    ("Current:", current_piece_var), ("Speed:", speed_var),
    ("Cycle Time:", cycle_time_var), ("Serial Link:", serial_link_var),
]
for i, (txt, var) in enumerate(labels):
    r = i // 5
    c = (i % 5) * 2
    tk.Label(dash, text=txt, font=("Arial", 9, "bold")).grid(row=r, column=c, sticky="e", padx=(0, 3), pady=3)
    tk.Label(dash, textvariable=var, width=18, relief="sunken", anchor="w").grid(row=r, column=c + 1, padx=(0, 12), pady=3)

# =====================================================
# MAIN AREA WITH TABS
# =====================================================
main = tk.Frame(root)
main.pack(fill="both", expand=True, padx=12, pady=6)
notebook = ttk.Notebook(main)
notebook.pack(side="left", fill="both", expand=True, padx=(0, 10))
notebook.bind("<<NotebookTabChanged>>", on_tab_changed)

# =====================================================
# TAB 1 - CYCLE TESTS
# =====================================================
config_tab = tk.Frame(notebook)
notebook.add(config_tab, text="Tab 1 - Cycle Tests")
config_box = tk.LabelFrame(config_tab, text="Cycle Test Configuration", padx=20, pady=20)
config_box.pack(fill="both", expand=True, padx=10, pady=10)

config_cycle = tk.StringVar(value="Cycle 1")
config_prehenseur = tk.StringVar(value="Préhenseur 1 - offset")
config_object = tk.StringVar(value="Gomme")
config_mode = tk.StringVar(value="Normal")
config_pieces = tk.IntVar(value=1)

selection_box = tk.LabelFrame(config_box, text="Quick Selections", padx=12, pady=12)
selection_box.grid(row=0, column=0, rowspan=2, sticky="nw", padx=(0, 25), pady=5)

class ButtonSelector:
    def __init__(self, parent, values, variable, command=None, columns=4, width=13):
        self.values = list(values)
        self.variable = variable
        self.command = command
        self.buttons = {}
        self.frame = tk.Frame(parent)

        for i, value in enumerate(self.values):
            b = tk.Button(
                self.frame,
                text=str(value),
                width=width,
                bg=NORMAL_BG,
                command=lambda v=value: self.select(v),
            )
            b.grid(row=i // columns, column=i % columns, padx=4, pady=4, sticky="ew")
            self.buttons[value] = b

        self.variable.trace_add("write", lambda *_: self.refresh())
        self.refresh()

    def grid(self, *args, **kwargs):
        self.frame.grid(*args, **kwargs)

    def pack(self, *args, **kwargs):
        self.frame.pack(*args, **kwargs)

    def select(self, value):
        self.variable.set(value)
        self.refresh()
        if self.command:
            self.command(value)

    def refresh(self):
        selected = self.variable.get()
        for value, button in self.buttons.items():
            button.configure(bg=SELECTED_BG if str(value) == str(selected) else NORMAL_BG)

tk.Label(selection_box, text="Cycle", font=("Arial", 11, "bold")).grid(row=0, column=0, sticky="w", pady=(0, 2))
ButtonSelector(selection_box, CYCLES, config_cycle, command=set_cycle, columns=2, width=12).grid(row=1, column=0, sticky="w", pady=(0, 8))

tk.Label(selection_box, text="Gripper", font=("Arial", 11, "bold")).grid(row=2, column=0, sticky="w", pady=(0, 2))
ButtonSelector(selection_box, PREHENSEURS, config_prehenseur, command=set_prehenseur, columns=1, width=24).grid(row=3, column=0, sticky="w", pady=(0, 8))

tk.Label(selection_box, text="Object", font=("Arial", 11, "bold")).grid(row=4, column=0, sticky="w", pady=(0, 2))
ButtonSelector(selection_box, OBJECT_CHOICES, config_object, command=set_object, columns=2, width=12).grid(row=5, column=0, sticky="w", pady=(0, 8))

tk.Label(selection_box, text="Mode", font=("Arial", 11, "bold")).grid(row=6, column=0, sticky="w", pady=(0, 2))
ButtonSelector(selection_box, MODES, config_mode, command=set_mode, columns=3, width=10).grid(row=7, column=0, sticky="w", pady=(0, 8))

tk.Label(selection_box, text="Pieces", font=("Arial", 11, "bold")).grid(row=8, column=0, sticky="w", pady=(8, 2))
pieces_row = tk.Frame(selection_box)
pieces_row.grid(row=9, column=0, sticky="w")
tk.Spinbox(pieces_row, from_=1, to=10, textvariable=config_pieces, width=8, command=set_pieces_from_spinbox).pack(side="left")
tk.Button(pieces_row, text="VALIDATE PIECES", width=16, command=set_pieces_from_spinbox).pack(side="left", padx=8)

tk.Button(selection_box, text="RECONNECT", width=25, command=reconnect_arduino).grid(row=10, column=0, sticky="w", pady=(18, 4))
tk.Button(selection_box, text="TEST SERIAL LINK", width=25, command=test_serial_link).grid(row=11, column=0, sticky="w", pady=4)
tk.Label(selection_box, textvariable=serial_link_var, width=25, relief="sunken", anchor="w").grid(row=12, column=0, sticky="w", pady=(2, 4))

oled_box = tk.LabelFrame(config_box, text="Simulated OLED Preview", padx=15, pady=15)
oled_box.grid(row=0, column=1, padx=(10, 25), pady=5, sticky="n")
oled_canvas = tk.Canvas(oled_box, width=300, height=170, bg="black", highlightthickness=2, highlightbackground="#444444")
oled_canvas.pack()

ihm_box = tk.LabelFrame(config_box, text="Cycle Control Buttons", padx=16, pady=16)
ihm_box.grid(row=0, column=2, padx=(0, 5), pady=5, sticky="n")

tk.Button(ihm_box, text="INIT", width=16, height=2, bg="#72d572", font=("Arial", 12, "bold"), command=ihm_init).pack(pady=8)
tk.Button(ihm_box, text="START", width=16, height=2, bg="#75bfff", font=("Arial", 12, "bold"), command=ihm_play).pack(pady=8)
tk.Button(ihm_box, text="PAUSE", width=16, height=2, bg="#ffd966", font=("Arial", 12, "bold"), command=ihm_pause).pack(pady=8)
tk.Label(ihm_box, text="Use INIT first.\nThen START to launch.\nPAUSE toggles pause/resume.", fg="red", font=("Arial", 10, "bold"), justify="left").pack(pady=10)

# =====================================================
# TAB 2 - SERVO + XYZ TESTS
# =====================================================
servo_tab = tk.Frame(notebook)
notebook.add(servo_tab, text="Tab 2 - Servo Tests")
servo_box = tk.LabelFrame(servo_tab, text="Individual Servo Tests", padx=20, pady=20)
servo_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(servo_box, text="On opening this tab, Arduino will first go to REPOS.\nUse these commands only when the robot is stopped.", fg="red", font=("Arial", 10, "bold"), justify="left").grid(row=0, column=0, columnspan=5, sticky="w", pady=(0, 15))

def add_servo_control(row, servo_name, label):
    var = tk.DoubleVar(value=90.0)
    tk.Label(servo_box, text=label, font=("Arial", 11, "bold")).grid(row=row, column=0, sticky="e", padx=10, pady=10)
    scale = tk.Scale(servo_box, from_=0, to=180, resolution=1, orient="horizontal", length=260, variable=var)
    scale.grid(row=row, column=1, padx=10, pady=10)
    tk.Button(servo_box, text="APPLY", width=10, command=lambda: set_servo(servo_name, var.get())).grid(row=row, column=2, padx=5)
    tk.Button(servo_box, text="-5", width=5, command=lambda: step_servo(servo_name, -5)).grid(row=row, column=3, padx=2)
    tk.Button(servo_box, text="+5", width=5, command=lambda: step_servo(servo_name, 5)).grid(row=row, column=4, padx=2)
    return var

servo_a_var = add_servo_control(1, "A", "Servo A")
servo_b_var = add_servo_control(2, "B", "Servo B")
servo_z_var = add_servo_control(3, "Z", "Servo Z")

tk.Button(servo_box, text="GO TO REPOS", width=25, height=2, command=go_repos_from_terminal).grid(row=4, column=0, columnspan=5, pady=15)

xyz_box = tk.LabelFrame(servo_box, text="Manual XYZ Coordinate Movement", padx=14, pady=14)
xyz_box.grid(row=1, column=5, rowspan=4, sticky="nw", padx=(45, 0), pady=5)

tk.Label(xyz_box, text="Enter a target position in mm.", fg="red", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))
xyz_x_var = tk.StringVar(value="0")
xyz_y_var = tk.StringVar(value="0")
xyz_z_var = tk.StringVar(value="120")
xyz_status_var = tk.StringVar(value="No XYZ command sent.")

for r, (label, var) in enumerate([("X", xyz_x_var), ("Y", xyz_y_var), ("Z", xyz_z_var)], start=1):
    tk.Label(xyz_box, text=label, font=("Arial", 11, "bold")).grid(row=r, column=0, sticky="e", padx=8, pady=6)
    tk.Entry(xyz_box, textvariable=var, width=16).grid(row=r, column=1, sticky="w", padx=8, pady=6)

tk.Button(xyz_box, text="GO TO XYZ", width=22, height=2, bg="#90ee90", font=("Arial", 11, "bold"), command=go_to_xyz_from_servo_tab).grid(row=4, column=0, columnspan=2, pady=(12, 8))
tk.Label(xyz_box, textvariable=xyz_status_var, width=38, relief="sunken", anchor="w").grid(row=5, column=0, columnspan=2, pady=8)

# =====================================================
# TAB 3 - BUTTON AND LED TESTS
# =====================================================
button_led_tab = tk.Frame(notebook)
notebook.add(button_led_tab, text="Tab 3 - Button/LED Tests")
button_led_box = tk.LabelFrame(button_led_tab, text="Button and LED Tests", padx=20, pady=20)
button_led_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(button_led_box, text="On opening this tab, the robot stays in its current position.\nButton test mode is activated: press INIT / START / PAUSE.", fg="red", font=("Arial", 10, "bold"), justify="left").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 15))
button_test_status_var = tk.StringVar(value="Button test inactive.")

tk.Label(button_led_box, text="Physical Button Tests", font=("Arial", 12, "bold")).grid(row=1, column=0, sticky="w", pady=10)
tk.Label(button_led_box, textvariable=button_test_status_var, width=45, relief="sunken", anchor="w", font=("Arial", 10)).grid(row=1, column=1, columnspan=3, sticky="w", padx=10)
tk.Button(button_led_box, text="BUTTON TEST ON", width=18, command=start_button_test_mode).grid(row=2, column=1, padx=5, pady=5, sticky="w")
tk.Button(button_led_box, text="BUTTON TEST OFF", width=18, command=stop_button_test_mode).grid(row=2, column=2, padx=5, pady=5, sticky="w")

tk.Label(button_led_box, text="LED Tests", font=("Arial", 12, "bold")).grid(row=3, column=0, sticky="w", pady=(25, 10))

def add_led_control(row, led_name, label):
    tk.Label(button_led_box, text=label, font=("Arial", 11, "bold")).grid(row=row, column=0, sticky="e", padx=10, pady=8)
    tk.Button(button_led_box, text="ON", width=10, command=lambda: led_set(led_name, True)).grid(row=row, column=1, padx=5, pady=8)
    tk.Button(button_led_box, text="OFF", width=10, command=lambda: led_set(led_name, False)).grid(row=row, column=2, padx=5, pady=8)
    tk.Button(button_led_box, text="PULSE", width=10, command=lambda: led_pulse(led_name)).grid(row=row, column=3, padx=5, pady=8)

add_led_control(4, "INIT", "INIT LED")
add_led_control(5, "RUN", "RUN LED")
add_led_control(6, "PAUSE", "PAUSE LED")
add_led_control(7, "POWER", "POWER LED")
tk.Button(button_led_box, text="RESTORE AUTO LED STATE", width=28, height=2, command=led_auto).grid(row=8, column=0, columnspan=4, pady=(20, 5))

# =====================================================
# TAB 4 - POSITION CHECK
# =====================================================
position_tab = tk.Frame(notebook)
notebook.add(position_tab, text="Tab 4 - Position Tests")
position_box = tk.LabelFrame(position_tab, text="Position Tests", padx=16, pady=16)
position_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(position_box, text="Test a saved position without running the full cycle.\nChoose the cycle, object, and one of the four positions.", fg="red", font=("Arial", 10, "bold"), justify="left").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 14))

position_cycle_var = tk.StringVar(value="Cycle 1")
position_prehenseur_var = tk.StringVar(value="Préhenseur 1 - offset")
position_object_var = tk.StringVar(value="Gomme")
position_name_var = tk.StringVar(value="Entrée")
position_command_var = tk.StringVar(value="GOTO_POSITION 0 0 0 0")
position_status_var = tk.StringVar(value="Select a cycle, object, and position.")

for var in [position_cycle_var, position_prehenseur_var, position_object_var, position_name_var]:
    var.trace_add("write", update_position_command_preview)

left_panel = tk.Frame(position_box)
left_panel.grid(row=1, column=0, sticky="nw", padx=(0, 28))
right_panel = tk.LabelFrame(position_box, text="Position Returned by Arduino", padx=12, pady=10)
right_panel.grid(row=1, column=1, sticky="nw", padx=(10, 0))

quick_select_box = tk.LabelFrame(left_panel, text="Quick Selections", padx=10, pady=10)
quick_select_box.grid(row=0, column=0, columnspan=2, sticky="w", padx=10, pady=(0, 14))

tk.Label(quick_select_box, text="Cycle", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w")
ButtonSelector(quick_select_box, CYCLES, position_cycle_var, command=lambda v: update_position_command_preview(), columns=2, width=12).grid(row=1, column=0, sticky="w", pady=(0, 8))

tk.Label(quick_select_box, text="Gripper", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="w")
ButtonSelector(quick_select_box, PREHENSEURS, position_prehenseur_var, command=lambda v: update_position_command_preview(), columns=1, width=24).grid(row=3, column=0, sticky="w", pady=(0, 8))

tk.Label(quick_select_box, text="Object", font=("Arial", 10, "bold")).grid(row=4, column=0, sticky="w")
ButtonSelector(quick_select_box, OBJECT_CHOICES, position_object_var, command=lambda v: update_position_command_preview(), columns=4, width=12).grid(row=5, column=0, sticky="w", pady=(0, 8))

tk.Label(quick_select_box, text="Position", font=("Arial", 10, "bold")).grid(row=6, column=0, sticky="w")
ButtonSelector(quick_select_box, POSITION_NAMES, position_name_var, command=lambda v: update_position_command_preview(), columns=4, width=12).grid(row=7, column=0, sticky="w", pady=(0, 8))

tk.Label(left_panel, text="Command", font=("Arial", 11, "bold")).grid(row=1, column=0, sticky="e", padx=10, pady=8)
tk.Entry(left_panel, textvariable=position_command_var, width=34, state="readonly").grid(row=1, column=1, sticky="w", padx=10, pady=8)

tk.Label(left_panel, text="Status", font=("Arial", 11, "bold")).grid(row=2, column=0, sticky="e", padx=10, pady=8)
tk.Label(left_panel, textvariable=position_status_var, width=58, relief="sunken", anchor="w").grid(row=2, column=1, sticky="w", padx=10, pady=8)

button_row = tk.Frame(left_panel)
button_row.grid(row=3, column=0, columnspan=2, sticky="w", padx=10, pady=(18, 8))
tk.Button(button_row, text="GO TO POSITION", width=22, height=2, bg="#90ee90", font=("Arial", 11, "bold"), command=go_to_selected_position).pack(side="left", padx=(0, 12))
tk.Button(button_row, text="GO TO REPOS", width=18, height=2, command=go_to_repos_from_position_tab).pack(side="left", padx=12)
tk.Button(button_row, text="GET STATUS", width=16, height=2, command=lambda: send_command("GET_STATUS")).pack(side="left", padx=12)

for r, (label, var) in enumerate([
    ("Last Position", last_position_var), ("X", current_x_var), ("Y", current_y_var), ("Z", current_z_var),
    ("Servo A", servo_a_live_var), ("Servo B", servo_b_live_var), ("Servo Z", servo_z_live_var),
]):
    tk.Label(right_panel, text=label, font=("Arial", 10, "bold")).grid(row=r, column=0, sticky="e", padx=5, pady=5)
    tk.Label(right_panel, textvariable=var, width=20, relief="sunken", anchor="w").grid(row=r, column=1, padx=5, pady=5)

update_position_command_preview()

# =====================================================
# TAB 5 - GRIPPER TESTS
# =====================================================
gripper_tab = tk.Frame(notebook)
notebook.add(gripper_tab, text="Tab 5 - Gripper Tests")
gripper_box = tk.LabelFrame(gripper_tab, text="Gripper / End Effector Tests", padx=18, pady=18)
gripper_box.pack(fill="both", expand=True, padx=10, pady=10)

tk.Label(
    gripper_box,
    text="Test the gripper without running a full cycle. The auto test uses the Cycle 1 entrance and the correct heights for each object. On Arduino side: go to object, close with PID, lift, place, open, then stop.",
    fg="red",
    font=("Arial", 10, "bold"),
    justify="left",
    wraplength=1050,
).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 15))

gripper_status_var = tk.StringVar(value="No gripper test sent.")
gripper_prehenseur_var = tk.StringVar(value="Préhenseur 1 - offset")
gripper_object_var = tk.StringVar(value="Gomme")

prehenseur_gripper_box = tk.LabelFrame(gripper_box, text="Gripper Used for Tests", padx=14, pady=14)
prehenseur_gripper_box.grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 14))
ButtonSelector(prehenseur_gripper_box, PREHENSEURS, gripper_prehenseur_var, command=set_prehenseur, columns=2, width=24).grid(row=0, column=0, sticky="w")

position_test_box = tk.LabelFrame(gripper_box, text="Test Position", padx=14, pady=14)
position_test_box.grid(row=2, column=0, sticky="nw", padx=(0, 20), pady=5)
tk.Label(
    position_test_box,
    text="Go to Cycle 1 entrance at test height Z=20.",
    justify="left",
    font=("Arial", 10),
    wraplength=260,
).grid(row=0, column=0, sticky="w", pady=(0, 10))
tk.Button(
    position_test_box,
    text="GO CYCLE 1 ENTRANCE\nTEST HEIGHT 20",
    width=28,
    height=3,
    bg="#90ee90",
    font=("Arial", 10, "bold"),
    command=go_to_cycle1_entree_test_height,
).grid(row=1, column=0, sticky="w", pady=8)

auto_test_box = tk.LabelFrame(gripper_box, text="Auto Test by Object", padx=14, pady=14)
auto_test_box.grid(row=2, column=1, sticky="nw", padx=(0, 20), pady=5)
tk.Label(
    auto_test_box,
    text="Choose the part. The robot should grab, lift, place, and open.",
    justify="left",
    font=("Arial", 10),
    wraplength=430,
).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
for i, obj_name in enumerate(REAL_OBJECTS):
    tk.Button(
        auto_test_box,
        text=f"TEST {obj_name.upper()}",
        width=18,
        height=2,
        bg="#75bfff",
        font=("Arial", 10, "bold"),
        command=lambda idx=i, name=obj_name: (gripper_object_var.set(name), run_gripper_auto_test(idx, name)),
    ).grid(row=1, column=i, padx=6, pady=8)

manual_box = tk.LabelFrame(gripper_box, text="Manual Gripper Command", padx=14, pady=14)
manual_box.grid(row=2, column=2, sticky="nw", pady=5)
tk.Label(
    manual_box,
    text="Use to manually place a part: close, open, then stop.",
    justify="left",
    font=("Arial", 10),
    wraplength=360,
).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
tk.Button(manual_box, text="CLOSE", width=12, height=2, bg="#ffd966", font=("Arial", 10, "bold"), command=manual_gripper_close).grid(row=1, column=0, padx=5, pady=8)
tk.Button(manual_box, text="OPEN", width=12, height=2, bg="#90ee90", font=("Arial", 10, "bold"), command=manual_gripper_open).grid(row=1, column=1, padx=5, pady=8)
tk.Button(manual_box, text="STOP", width=12, height=2, bg="#ff9999", font=("Arial", 10, "bold"), command=manual_gripper_stop).grid(row=1, column=2, padx=5, pady=8)

status_box = tk.LabelFrame(gripper_box, text="Gripper Test Status", padx=12, pady=10)
status_box.grid(row=3, column=0, columnspan=3, sticky="we", pady=(22, 5))
tk.Label(status_box, textvariable=gripper_status_var, width=110, relief="sunken", anchor="w", font=("Arial", 10)).pack(fill="x")

command_help_box = tk.LabelFrame(gripper_box, text="Commands Sent to Arduino", padx=12, pady=10)
command_help_box.grid(row=4, column=0, columnspan=3, sticky="we", pady=(12, 5))
tk.Label(
    command_help_box,
    text="Position test: GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT\nAuto tests: GRIPPER_AUTO_TEST gripper object   (gripper 0/1, object 0=Rubber, 1=Cup, 2=Cylinder)\nManual: GRIPPER_CLOSE, GRIPPER_OPEN, GRIPPER_STOP",
    justify="left",
    font=("Consolas", 10),
).pack(anchor="w")

# =====================================================
# TAB 6 - INFO / NOTICE
# =====================================================
info_tab = tk.Frame(notebook)
notebook.add(info_tab, text="Tab 6 - Notice")

info_box = tk.LabelFrame(info_tab, text="User Manual - Maintenance Console", padx=20, pady=20)
info_box.pack(fill="both", expand=True, padx=10, pady=10)

info_text = """
NORMAL OPERATION:
- The computer is disconnected.
- The robot only works with physical buttons: INIT, START, and PAUSE.
- The maintenance console is not used during normal operation.

MAINTENANCE MODE:
- The computer is intentionally connected to use this console.
- The console allows testing, diagnosing, and adjusting the robot without automatically starting a full cycle.
- It is used to verify the proper functioning of actuators, buttons, LEDs, positions, and the gripper.

TAB DESCRIPTIONS:
- Tab 1: Cycle Tests
  Cycle configuration, object choice, speed mode selection, number of parts,
  INIT / START / PAUSE commands, and cycle time display.

- Tab 2: Servo Tests
  Individual control of servos A, B, and Z, return to REPOS, and manual movement by XYZ coordinates.

- Tab 3: Button/LED Tests
  Verification of physical buttons INIT / START / PAUSE and manual LED testing.

- Tab 4: Position Tests
  Verification of saved arm positions:
  Entrance, Machine A, Machine B, and Exit.

- Tab 5: Gripper Tests
  Manual gripper testing: CLOSE, OPEN, STOP.
  Auto test: grab an object, lift, place, then open the gripper.

ADJUSTABLE PARAMETERS:
- Cycle execution speed.
- Number of parts to process.
- Manual arm positioning by servos or XYZ coordinates.
- Gripper behavior through manual and automatic tests.

QUICK USE:
1. Ensure the robot is supervised.
2. Choose the tab corresponding to the desired test.
3. Set the necessary parameters.
4. Launch the test.
5. Observe the robot's behavior.
6. Return to REPOS if necessary.

SAFETY:
- Use the console only in supervised mode.
- Keep hands out of the workspace during movements.
- Do not intervene mechanically during a movement.
- Return to REPOS before any manual handling.
"""

info_scroll_frame = tk.Frame(info_box)
info_scroll_frame.pack(fill="both", expand=True)

info_scrollbar = tk.Scrollbar(info_scroll_frame)
info_scrollbar.pack(side="right", fill="y")

info_display = tk.Text(
    info_scroll_frame,
    wrap="word",
    yscrollcommand=info_scrollbar.set,
    font=("Arial", 10),
    padx=12,
    pady=10,
    height=22
)
info_display.pack(side="left", fill="both", expand=True)
info_scrollbar.config(command=info_display.yview)

info_display.insert("1.0", info_text.strip())
info_display.config(state="disabled")

# =====================================================
# LOG
# =====================================================
log_frame = tk.LabelFrame(root, text="Serial Log", padx=5, pady=5)
log_frame.pack(fill="both", expand=False, padx=12, pady=6)
log_box = tk.Text(log_frame, height=7)
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
