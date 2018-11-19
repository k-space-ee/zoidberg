# ROS setup before gevent
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/movement', Twist, queue_size=10)
rospy.init_node('websocket', anonymous=True)

# thread fixes
import gevent
from gevent import monkey
from gevent.queue import Empty

monkey.patch_all(thread=False)

# imports
import json
import logging
from collections import deque
from time import time

from datetime import datetime, timedelta

from flask import Flask, render_template
from flask_sockets import Sockets

from config_manager import ConfigManager

logger = logging.getLogger("flask")

app = Flask(__name__)

try:
    with open("/etc/machine-id", "r") as fh:
        app.config['SECRET_KEY'] = fh.read()
except:
    app.config['SECRET_KEY'] = 'secret!'

sockets = Sockets(app)

# Queue messages from bootstrap
log_queue = deque(maxlen=1000)

websockets = set()


@app.route('/')
def group():
    return render_template(
        'group.html',
    )


@app.route('/logging')
def logging_view():
    return render_template('logging.html')


@sockets.route('/')
def command(websocket):
    # Todo: what is this
    # for buf in log_queue:
    #     websocket.send(buf)

    game_config = ConfigManager.instance("game")

    def get_game_options():
        if not game_config:
            logger.error("No game config found")
            return []
        return [
            ("field_id", [game_config.get_value("global", "field_id"), "A", "B", "Z"]),
            ("robot_id", [game_config.get_value("global", "robot_id"), "A", "B"]),
            ("target goal color", [game_config.get_value("global", "target goal color"), "yellow", "blue"]),
            ("gameplay status", [game_config.get_value("global", "gameplay status"), "disabled", "enabled"]),
        ]

    game_options = get_game_options()

    settings_packet = json.dumps(dict(
        action="settings-packet",
        sliders=dict(color=ConfigManager.get_value("color")),
        options=game_options
    ))
    websocket.send(settings_packet)

    last_press = time()
    last_press_history = []

    counter = 0
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

        if action == "gamepad":
            controls = response.get("data")

            if controls:
                x = controls.pop("controller0.axis0") * 0.33
                y = controls.pop("controller0.axis1") * 0.33
                w = controls.pop("controller0.axis3") * 0.2

                motion = Twist()
                motion.linear.x = x
                motion.linear.y = y
                motion.angular.z = w

                pub.publish(motion)

                last_press_history = [*last_press_history, time() - last_press][-30:]
                last_press = time()
                if counter % 120 == 1:
                    logger.info("Last press %.3f ago on average", sum(last_press_history) / len(last_press_history))

        elif action == "set_settings":
            for k, v in response.items():
                ConfigManager.set_value(k, v)
        elif action == "set_options":
            for k, v in response.items():
                game_config.get_option("global", k).set_value(v)
                game_config.save()
        else:
            logger.error("Unhandled action: %s", str(action))

    websockets.remove(websocket)
    logger.info("WebSocket connection closed, %d left connected", len(websockets))
    return b""


class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, datetime):
            return obj.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        if isinstance(obj, timedelta):
            f = obj.total_seconds()
            mins = f // 60
            return "%02d:%06.03f" % (mins, f % 60)
        return json.JSONEncoder.default(self, obj)


class WebsocketLogHandler(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.started = datetime.utcnow()

    def emit(self, record):
        timestamp = datetime.utcfromtimestamp(record.created)
        buf = json.dumps(dict(
            action="log-entry",
            created=timestamp,
            uptime=timestamp - self.started,
            message=record.msg % record.args,
            severity=record.levelname.lower()), cls=MyEncoder)
        log_queue.append(buf)
        for websocket in websockets:
            websocket.send(buf)


def main():
    import os
    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler

    logger.info("Starting robovision")

    logging.basicConfig(
        filename="/tmp/robovision.log",
        level=logging.INFO)

    ws_handler = WebsocketLogHandler()
    handler = logging.StreamHandler()
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    handler.setFormatter(formatter)
    for facility in "grabber", "recognition", "cli", "flask", "arduino", "gameplay", "threading", "recorder":
        logging.getLogger(facility).addHandler(handler)
        logging.getLogger(facility).addHandler(ws_handler)
        logging.getLogger(facility).setLevel(logging.DEBUG)

    ip, port = ('0.0.0.0', 5000)
    if os.getuid() == 0:
        port = 80

    server = pywsgi.WSGIServer((ip, port), app, handler_class=WebSocketHandler)
    logger.info("Started server at http://{}:{}".format(ip, port))

    server.serve_forever()


if __name__ == '__main__':
    main()
