# ROS setup before gevent
import messenger
from websocket_log_handler import WebsocketLogHandler

movement_publisher = messenger.Publisher('/movement', messenger.Messages.motion)
kicker_publisher = messenger.Publisher('/kicker_speed', messenger.Messages.integer)
command_publisher = messenger.Publisher('/command', messenger.Messages.string)
strategy_state = messenger.Listener('/strategy', messenger.Messages.string)
canbus_state = messenger.Listener('/canbus_message', messenger.Messages.string)
websocket_log_handler = WebsocketLogHandler()
logging_state = messenger.Listener('/rosout_agg', messenger.Messages.logging, callback=websocket_log_handler.emit)
node = messenger.Node('websocket', disable_signals=True)

# thread fixes
import gevent
from gevent import monkey

monkey.patch_all(thread=False)

# imports
import json
from time import time

from flask import Flask, render_template
from flask_sockets import Sockets

from config_manager import ConfigManager

logger = node.logger

app = Flask(__name__)

try:
    with open("/etc/machine-id", "r") as fh:
        app.config['SECRET_KEY'] = fh.read()
except:
    app.config['SECRET_KEY'] = 'secret!'

sockets = Sockets(app)

# Queue messages from bootstrap
websockets = websocket_log_handler.websockets


@app.route('/')
def group():
    return render_template('group.html')


@app.route('/logging')
def logging_view():
    return render_template('logging.html')


@sockets.route('/')
def command(websocket):
    # send logging history to ws
    for buf in websocket_log_handler.queue:
        websocket.send(buf)

    game_config = ConfigManager.get_value('game')
    if not game_config:
        ConfigManager.set_value('game|global|field_id', 'A')
        ConfigManager.set_value('game|global|robot_id', 'A')
        ConfigManager.set_value('game|global|target goal color', 'blue')
        ConfigManager.set_value('game|global|gameplay status', 'disabled')

    game_options = [
        ("field_id", [ConfigManager.get_value("game|global|field_id"), "A", "B", "Z"]),
        ("robot_id", [ConfigManager.get_value("game|global|robot_id"), "A", "B"]),
        ("target goal color", [ConfigManager.get_value("game|global|target goal color"), "purple", "blue"]),
        ("gameplay status", [ConfigManager.get_value("game|global|gameplay status"), "disabled", "enabled"]),
    ]

    settings_packet = json.dumps(dict(
        action="settings-packet",
        sliders=dict(color=ConfigManager.get_value("color")),
        options=game_options
    ))
    websocket.send(settings_packet)

    last_press = time()
    last_press_history = []

    counter = 0
    rpm = 6500
    while not websocket.closed:
        counter += 1
        websockets.add(websocket)

        gevent.sleep(0.005)

        msg = websocket.receive()

        if not msg:
            websockets.remove(websocket)
            logger.error("WebSocket connection presumably closed, %d left connected" % len(websockets))
            break

        response = json.loads(msg)

        action = response.pop("action", None)

        game_package = strategy_state.package or {}
        canbus_package = canbus_state.package or {}

        gameplay = game_package.get('gameplay', None)
        target_goal_angle = game_package.get('target_goal_angle', 0)
        average_rpm = canbus_package.get('average_rpm', 0)

        if action == "gamepad":
            controls = response.get("data")
            # print(controls)

            if controls:
                commands = {}

                if gameplay:
                    toggle_gameplay = controls.get("controller0.button8") or controls.get("controller0.button11")
                    if toggle_gameplay:
                        commands['toggle_gameplay'] = None
                else:
                    # # Manual control of the robot
                    x = controls.pop("controller0.axis0", 0) * 0.33
                    y = controls.pop("controller0.axis1", 0) * 0.33
                    w = controls.pop("controller0.axis3", 0) * 0.2

                    if controls.get("controller0.button3"):
                        y = 0.15

                    if x or y or w:
                        movement_publisher.publish(x=x, y=-y, az=-w)

                    delta = controls.pop("controller0.button12", 0)
                    delta = delta or -controls.pop("controller0.button13", 0)

                    if delta:
                        rpm = max(0, min(rpm + delta * 50, 15000))
                        logger.info("PWM+: %.0f", rpm)

                    if controls.get("controller0.button0", None):
                        logger.info(f"drive_towards_target_goal: {target_goal_angle} rpm:{rpm} speed:{average_rpm}")
                        kicker_publisher.publish(rpm)
                        # no driving backwards when angle error
                        command_publisher.command(drive_towards_target_goal=dict(backtrack=False, speed_factor=0.5))

                    if controls.get("controller0.button5", None):
                        logger.info(f"kick: {target_goal_angle} rpm:{rpm} speed:{average_rpm}")
                        kicker_publisher.publish(rpm)

                    if controls.get("controller0.button6", None):
                        logger.info("Drive to center")
                        command_publisher.command(drive_to_field_center=None)

                    if controls.get("controller0.button2", None):
                        logger.info(f"drive_towards_target_goal: {target_goal_angle} speed:{average_rpm}")
                        command_publisher.command(drive_towards_target_goal=dict(backtrack=False, speed_factor=0.5))
                        kicker_publisher.publish(rpm)

                last_press_history = [*last_press_history, time() - last_press][-30:]
                last_press = time()
                if counter % 30 == 1:
                    average = sum(last_press_history) / len(last_press_history)
                    logger.info("Last press %.3f ago on average", average)

        elif action == "set_settings":
            for k, v in response.items():
                ConfigManager.set_value(k, v)
        elif action == "set_options":
            for k, v in response.items():
                ConfigManager.set_value(f"game|global|{k}", v)
        else:
            logger.error("Unhandled action: %s", str(action))

    websockets.remove(websocket)
    logger.info("WebSocket connection closed, %d left connected", len(websockets))
    return b""


def main():
    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler

    messenger.ConnectPythonLoggingToROS.reconnect('config_manager')

    logger.info("Starting robovision")

    ip, port = ('0.0.0.0', 5000)
    # TODO: maybe useful
    # if os.getuid() == 0:
    #     port = 80

    server = pywsgi.WSGIServer((ip, port), app, handler_class=WebSocketHandler)
    logger.info("Started server at http://{}:{}".format(ip, port))

    server.serve_forever()


if __name__ == '__main__':
    main()
