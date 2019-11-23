#!/usr/bin/python3
from multiprocessing import Process
from time import sleep
from typing import List
from argparse import ArgumentParser

import messenger
from controller_node import ControllerNode
from gameplay_node import GameplayNode
from injector_node import InjectorNode
from kicker_node import KickerNode
from realsense_node import RealSenseNode
from remoterf import RemoteRF
from tfmini import TFMiniNode

parser = ArgumentParser()
parser.add_argument("-m", "--mock", dest="mock", action="store_true", default=False,
                    help="no actual actuators are invoked", )
parser.add_argument("-n", "--nuke", dest="nuke", action="store_true", default=False,
                    help="nuke ros on exit", )
parser.add_argument("-r", "--remote", dest="remote", action="store_true", default=False,
                    help="listen to remote", )
args = parser.parse_args()


class Launcher:
    def __init__(self) -> None:
        self.nodes: List[Process] = []

    def launch(self, node, *args, **kwargs):
        p = Process(target=node, args=args, name=str(node), kwargs=kwargs)
        self.nodes.append(p)

        p.start()
        print(f'Started node {node}')

    def spin(self):
        keyboard = False
        try:
            for process in reversed(self.nodes):
                process.join()
                process.terminate()
        except KeyboardInterrupt:
            keyboard = True
            print("\nKeyboard interrupt")
            if args.nuke:
                import os
                os.system('pkill -f ros')

        for process in self.nodes:
            process.terminate()

        print("Game over!")
        if keyboard:
            exit()


def server():
    import io_server
    io_server.main()


def image_server(silent=False):
    import image_server
    image_server.main(silent)


class RestartWrapper:
    def __init__(self, node, **kwargs) -> None:
        self.node = node
        self.kwargs = kwargs

    def __call__(self, *args, **kwargs):
        # the node kills itself when the cameras fail so that we could restart it here :)

        launcher = Launcher()
        for i in range(100):
            launcher.launch(self.node, **self.kwargs)
            launcher.spin()
            print(self.node, "RESTART == ", i)
            sleep(1)


def image_server_launcher(silent=False):
    # the image server kills itself when the cameras fail so that we could restart it here :)
    launcher = Launcher()
    for i in range(100):
        launcher.launch(image_server, silent=silent)

        launcher.spin()
        print("IMAGE SERVER RESTART == ", i)
        sleep(1)


if __name__ == '__main__':
    launcer = Launcher()

    launcer.launch(messenger.core)
    launcer.launch(image_server_launcher, silent=True)
    # launcer.launch(RestartWrapper(RealSenseNode, mock=args.mock))
    launcer.launch(server)
    launcer.launch(ControllerNode, mock=args.mock, silent=True)
    launcer.launch(KickerNode, mock=args.mock, silent=True)
    launcer.launch(GameplayNode, mock=args.mock)
    launcer.launch(InjectorNode, mock=args.mock)
    launcer.launch(RestartWrapper(TFMiniNode, mock=args.mock))

    if args.remote:
        launcer.launch(RemoteRF, mock=args.mock)

    launcer.spin()
