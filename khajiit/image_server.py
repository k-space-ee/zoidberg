import messenger
from config_manager import ConfigManager

listener_wrapper = messenger.CallbackListenerWrapper()

node = messenger.Node('image_server')
settings_change = messenger.Listener('/settings_changed', messenger.Messages.string, callback=listener_wrapper.update)
recognition_publisher = messenger.Publisher('/recognition', messenger.Messages.string)

from gevent import monkey
from gevent.queue import Empty

monkey.patch_all(thread=False)

from time import sleep
from flask import Flask, Response
from collections import deque
from camera.recorder import Recorder
from camera.visualization import Visualizer
from camera.image_recognition import ImageRecognizer
from camera.grabber import PanoramaGrabber

logger = node.logger

# Queue messages from bootstrap
log_queue = deque(maxlen=1000)
websockets = set()

app = Flask(__name__)

# Build pipeline
config = ConfigManager.get_value('camera')
grabber = PanoramaGrabber(config)
image_recognizer = ImageRecognizer(
    grabber, config_manager=ConfigManager, publisher=recognition_publisher, produce_rate=10)
visualizer = Visualizer(image_recognizer, framedrop=1)
# recorder = Recorder(grabber)

# settings listeners
listener_wrapper.listeners.append(image_recognizer.refresh_config)


def generator(type_str):
    visualizer.enable()
    visualizer.type_str = type_str
    queue = visualizer.get_queue()
    while True:
        try:
            buf, resized, frame, r = queue.get_nowait()
        except Empty:
            sleep(0.001)  # Fix this stupid thingie
            continue
        else:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
            yield buf
            yield b'\r\n\r\n'


@app.route('/combined/<path:type_str>')
def video_combined(type_str):
    return Response(generator(type_str.upper()), mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    from gevent import pywsgi

    messenger.ConnectPythonLoggingToROS.reconnect('image_recognition', 'visualization', 'threading')

    ip, port = ('0.0.0.0', 5005)

    server = pywsgi.WSGIServer((ip, port), app)

    # Quick'n'diry hacks
    image_recognizer.grabber = grabber
    image_recognizer.websockets = websockets

    # Start all threads
    image_recognizer.start()
    grabber.start()
    # recorder.start()
    visualizer.start()

    # Register threads for monitoring
    from camera.managed_threading import ThreadManager
    manager = ThreadManager()
    manager.register(grabber)
    manager.register(visualizer)
    manager.register(image_recognizer)
    manager.start()

    # Enable some threads
    image_recognizer.enable()
    visualizer.enable()

    logger.info("Started IMAGE server at http://{}:{}".format(ip, port))
    server.serve_forever()


if __name__ == '__main__':
    main()
