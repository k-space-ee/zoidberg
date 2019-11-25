import cv2 as cv

import messenger
from config_manager import ConfigManager
from shared import attach
from utils import RecognitionState

listener_wrapper = messenger.CallbackListenerWrapper()

node = messenger.Node('image_server')
settings_change = messenger.Listener('/settings_changed', messenger.Messages.string, callback=listener_wrapper.update)

visualizer = None


def recognition_callback(*args):
    package = recognition_listener.package
    if package and visualizer:
        visualizer.recognition = RecognitionState.from_dict(package)


def strategy_callback(*args):
    package = strategy_listener.package
    if package and visualizer:
        visualizer.gamestate = package


recognition_listener = messenger.Listener(
    '/recognition', messenger.Messages.string, callback=recognition_callback)

strategy_listener = messenger.Listener(
    '/strategy', messenger.Messages.string, callback=strategy_callback)

from gevent import monkey

monkey.patch_all(thread=False)

from time import sleep
from flask import Flask, Response
from collections import deque
from camera.visualization import Visualizer

logger = node.logger

# Queue messages from bootstrap
log_queue = deque(maxlen=1000)
websockets = set()

app = Flask(__name__)

config = ConfigManager.get_value('camera')
visualizer = Visualizer(config)


def generator(type_str):
    visualizer.type_str = type_str
    while True:
        sleep(0.050)  # Fix this stupid thingie
        if visualizer.jpeg:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
            yield visualizer.jpeg
            yield b'\r\n\r\n'


def realsense_generator(type_str):
    image = attach("shm://depth-color")
    while True:
        sleep(0.050)  # Fix this stupid thingie
        try:
            ret, jpeg = cv.imencode('.jpg', image, (cv.IMWRITE_JPEG_QUALITY, 80))
            buf = jpeg.tostring()
        except Exception as e:
            logger.error_throttle(1, f"{e}")
            continue

        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
        yield buf
        yield b'\r\n\r\n'


@app.route('/combined/<path:type_str>')
def video_combined(type_str):
    return Response(generator(type_str.upper()), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/realsense')
def video_realsense():
    return Response(realsense_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


def main(silent=False):
    from gevent import pywsgi

    messenger.ConnectPythonLoggingToROS.reconnect('visualization', 'threading')

    ip, port = ('0.0.0.0', 5005)

    server = pywsgi.WSGIServer((ip, port), app)

    visualizer.thread.start()

    logger.info("Started IMAGE server at http://{}:{}".format(ip, port))
    server.start()
    node.spin()


if __name__ == '__main__':
    main()
