from multiprocessing import Process
from typing import List

import messenger
from controller_node import ControllerNode
from gameplay_node import GameplayNode
from kicker_node import KickerNode
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-m", "--mock", dest="mock", action="store_true", default=False,
                    help="no actual actuators are invoked", )
parser.add_argument("-n", "--nuke", dest="nuke", action="store_true", default=False,
                    help="nuke ros on exit", )
args = parser.parse_args()


class Launcher:
    def __init__(self) -> None:
        self.nodes: List[Process] = []

    def launch(self, node, *args, **kwargs):
        p = Process(target=node, args=args, kwargs=kwargs)
        self.nodes.append(p)

        p.start()
        print(f'Started node {node}')

    def spin(self):
        try:
            for process in self.nodes:
                process.join()
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


def image_server():
    import image_server
    image_server.main()


launcer = Launcher()

launcer.launch(messenger.core)
launcer.launch(server)
launcer.launch(image_server)
launcer.launch(ControllerNode, mock=args.mock)
launcer.launch(KickerNode, mock=args.mock)
launcer.launch(GameplayNode, mock=args.mock)

launcer.spin()