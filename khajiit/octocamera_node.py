import messenger
from config_manager import ConfigManager

from camera.image_recognition import ImageRecognizer
from camera.grabber import PanoramaGrabber

listener_wrapper = messenger.CallbackListenerWrapper()

node = messenger.Node('octocamera')
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
