from time import sleep

import messenger
from config_manager import ConfigManager

from camera.image_recognition import ImageRecognizer
from camera.grabber import PanoramaGrabber

def kill():
    if grabber and grabber.slaves:
        cameras = list(grabber.slaves)
        for camera in cameras:
            camera.stop()

        sleep(0.030)
        for camera in cameras:
            camera.die("rospy shutdown")

    if manager and manager.threads:
        for t in manager.threads:
            try:
                t.stop()
            except:
                print("BLOOP!?")
    print("KILL DONE")
    exit(0)


listener_wrapper = messenger.CallbackListenerWrapper()

node = messenger.Node('octocamera', on_shutdown=kill)
settings_change = messenger.Listener('/settings_changed', messenger.Messages.string, callback=listener_wrapper.update)
recognition_publisher = messenger.Publisher('/recognition', messenger.Messages.string)

logger = node.logger

# Build pipeline
config = ConfigManager.get_value('camera')
grabber = PanoramaGrabber(config)
image_recognizer = ImageRecognizer(
    grabber, config_manager=ConfigManager, publisher=recognition_publisher)

# settings listeners
listener_wrapper.listeners.append(image_recognizer.refresh_config)

manager = None  # type: ThreadManager


def main(silent=False):
    if not silent:
        messenger.ConnectPythonLoggingToROS.reconnect('image_recognition', 'visualization', 'threading', 'grabber')
    else:
        image_recognizer.silent = True
        messenger.ConnectPythonLoggingToROS.reconnect('grabber', 'image_recognition')

    # Quick'n'diry hacks
    image_recognizer.grabber = grabber

    # Start all threads
    image_recognizer.start()
    grabber.start()

    # Register threads for monitoring
    from camera.managed_threading import ThreadManager
    manager = ThreadManager()
    manager.register(grabber)
    manager.register(image_recognizer)
    manager.start()

    # Enable some threads
    image_recognizer.enable()

    node.spin()


if __name__ == '__main__':
    main()
