
import gevent
from gevent import monkey
from gevent.queue import Queue, Empty
from gevent.event import Event
monkey.patch_all(thread=False)
from time import sleep, time
import json
import cv2
from threading import Thread, Event
import signal
import numpy as np
from flask import Flask, render_template, Response, request
from flask_sockets import Sockets
import logging
import os
import humanize
from collections import deque
from recorder import Recorder
from gameplay.stateful import Gameplay
from datetime import datetime, timedelta
from visualization import Visualizer
from image_recognition import ImageRecognizer, ImageRecognition
from grabber import PanoramaGrabber
from remoterf import RemoteRF
from config_manager import ConfigManager



# Get Gevent websocket patched for Python3 here:
# https://bitbucket.org/noppo/gevent-websocket/
# hg update python3-support
# sudo python3 setup.py install

logger = logging.getLogger("flask")

# Queue messages from bootstrap
log_queue = deque(maxlen=1000)
websockets = set()

app = Flask(__name__)
try:
    with open("/etc/machine-id", "r") as fh:
        app.config['SECRET_KEY'] = fh.read()
except:
    app.config['SECRET_KEY'] = 'secret!'
sockets = Sockets(app)




# Build pipeline
grabber = PanoramaGrabber() # config read from ~/.robovision/grabber.conf
image_recognizer = ImageRecognizer(grabber)
gameplay = Gameplay(image_recognizer)
#rf = RemoteRF("/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1:1.0-port0", gameplay)
# TODO: should also get gameplay?
visualizer = Visualizer(image_recognizer, framedrop=1)
recorder = Recorder(grabber)

def generator(type_str):
    visualizer.enable()
    visualizer.type_str = type_str
    queue = visualizer.get_queue()
    while True:
        try:
            buf, resized, frame, r = queue.get_nowait()
        except Empty:
            sleep(0.001) # Fix this stupid thingie
            continue
        else:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
            yield buf
            yield b'\r\n\r\n'

@app.route('/combined/<path:type_str>')
def video_combined(type_str):
    TYPES = ['VIDEO', 'DEBUG', 'COMBO']
    return Response(generator(type_str.upper()), mimetype='multipart/x-mixed-replace; boundary=frame')

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
    x = 0
    y = 0
    w = 0

    for buf in log_queue:
        websocket.send(buf)

    game_config = ConfigManager.instance("game")

    def get_game_options():
        return [
            ("field_id", [game_config.get_value("global", "field_id"),"A","B","Z"]),
            ("robot_id", [game_config.get_value("global", "robot_id"),"A","B"]),
            ("target goal color", [game_config.get_value("global", "target goal color"),"yellow","blue"]),
            ("gameplay status", [game_config.get_value("global", "gameplay status"),"disabled", "enabled"]),
        ]

    game_options = get_game_options()

    settings_packet = json.dumps(dict(
        action = "settings-packet",
        sliders = ConfigManager.as_list("imgrec"),
        options = game_options
    ))
    websocket.send(settings_packet)

    pwm = 50
    last_press = time()
    while not websocket.closed:
        websockets.add(websocket)

        gevent.sleep(0.01)

        msg = websocket.receive()

        if not msg:
            websockets.remove(websocket)
            logger.info("WebSocket connection presumably closed, %d left connected" % len(websockets))
            break

        response = json.loads(msg)
        action = response.pop("action", None)
        if not action:
            logger.info("Unknown action")
            continue

        if action == "gamepad":
            controls = response.pop("data")

            if (controls.get("controller0.button8", None) or controls.get("controller0.button11", None)) and time() - last_press > 0.5:
                last_press = time()
                if not gameplay.alive:
                    gameplay.enable()
                else:
                    gameplay.disable()

            # Toggle autonomy with button Y on Logitech gamepad
            # if controls.get("controller0.button4", None):
            #     not gameplay.alive and gameplay.enable()
            # else:
            #     gameplay.alive and gameplay.disable()

            # Manual control of the robot
            if not gameplay.alive:
                gameplay.recognition = image_recognizer.last_frame

                x = controls.pop("controller0.axis0", x) * 0.33
                y = controls.pop("controller0.axis1", y) * 0.33
                w = controls.pop("controller0.axis3", w) * 0.2

                if controls.get("controller0.button3", None):
                    y = 0.15

                gameplay.arduino.set_xyw(x,-y,-w)

                # Throw the ball with button A on Logitech gamepad
                delta = controls.pop("controller0.button12", 0)
                delta = delta or -controls.pop("controller0.button13", 0)

                if delta and time() - last_press > 0.1:
                    last_press = time()
                    pwm = max(0, min(pwm + delta * 200, 10000))
                    print("PWM+: ", pwm)
                if controls.get("controller0.button0", None):
                    print("PWM: ", pwm)
                    gameplay.arduino.set_thrower(pwm)
                    gameplay.drive_towards_target_goal(safety=False) # safety=False means no backtrack

                elif controls.get("controller0.button5", None):
                    logger.info(str(gameplay.state))
                    gameplay.arduino.set_thrower(pwm)
                else:
                    gameplay.arduino.set_thrower(0)

                if controls.get("controller0.button1", None):
                    logger.info(str(gameplay.state))
                    gameplay.kick()

                if controls.get("controller0.button6", None) and gameplay.recognition:
                    gameplay.drive_to_field_center()
                    print("button6: drive to center")

                if controls.get("controller0.button2", None):
                    gameplay.align_to_goal()
                    logger.info(str(gameplay.state))

                gameplay.arduino.apply()
                # gameplay.arduino.ser.flushInput()

        # TODO: slders
        elif action == "record_toggle":
            print("TOGGLING RECORDER")
            recorder.toggle()
        elif action == "record_enable":
            recorder.enable()
        elif action == "record_disable":
            recorder.disable()
        elif action == "set_settings":
            for k, v in response.items():
                ConfigManager.set_config_value(k, v)
            print(response.items())
        elif action == "set_options":
            for k, v in response.items():
                game_config.get_option("global", k).set_value(v)
                game_config.save()
            print(response.items())
        else:
            logger.error("Unhandled action: %s", action)
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
                action = "log-entry",
                created = timestamp,
                uptime = timestamp - self.started,
                message = record.msg % record.args,
                severity = record.levelname.lower()), cls=MyEncoder)
        log_queue.append(buf)
        for websocket in websockets:
            websocket.send(buf)

def main():
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

    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler

    ip, port = ('0.0.0.0', 5000)
    if os.getuid() == 0:
        port = 80


    server = pywsgi.WSGIServer((ip, port), app, handler_class=WebSocketHandler)
    logger.info("Started server at http://{}:{}".format(ip, port))

    # Quick'n'diry hacks
    image_recognizer.grabber = grabber
    image_recognizer.websockets = websockets

    # Start all threads
    image_recognizer.start()
    gameplay.start()
    grabber.start()
    recorder.start()
    visualizer.start()
    #rf.start()

    # Register threads for monitoring
    from managed_threading import ThreadManager
    manager = ThreadManager()
    manager.register(gameplay)
    manager.register(recorder)
    manager.register(grabber)
    manager.register(visualizer)
    manager.register(image_recognizer)
    manager.start()

    # Enable some threads
    image_recognizer.enable()
    visualizer.enable()
    #if gameplay.is_enabled:
        #gameplay.enable()
    server.serve_forever()

if __name__ == '__main__':
    main()

#start_server = websockets.serve(time, '0.0.0.0', 5001)
#asyncio.get_event_loop().run_until_complete(start_server)
#asyncio.get_event_loop().run_forever()
