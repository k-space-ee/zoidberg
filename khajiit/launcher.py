#!/usr/bin/python3
from multiprocessing import Process
from typing import List
from argparse import ArgumentParser

import messenger
from controller_node import ControllerNode
from gameplay_node import GameplayNode
from injector_node import InjectorNode
from kicker_node import KickerNode
from remoterf import RemoteRF


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
        try:
            for process in reversed(self.nodes):
                process.join()
                process.terminate()
        except KeyboardInterrupt:
            print("\nKeyboard interrupt")
            if args.nuke:
                import os
                os.system('pkill -f ros')

        for process in self.nodes:
            process.terminate()

        print("Game over!")


def server():
    import io_server
    io_server.main()


def image_server(silent=False):
    import image_server
    image_server.main(silent)


launcer = Launcher()

launcer.launch(messenger.core)
launcer.launch(image_server, silent=True)
launcer.launch(server)
launcer.launch(ControllerNode, mock=args.mock, silent=True)
launcer.launch(KickerNode, mock=args.mock, silent=True)
launcer.launch(GameplayNode, mock=args.mock)
launcer.launch(InjectorNode, mock=args.mock)
if args.remote:
    launcer.launch(RemoteRF, mock=args.mock)

launcer.spin()
