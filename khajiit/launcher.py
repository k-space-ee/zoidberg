from multiprocessing import Process

from controller_node import ControllerNode
from kicker_node import KickerNode


class Launcher:
    def __init__(self) -> None:
        self.nodes = []

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

        print("Game over!")


launcer = Launcher()

launcer.launch(ControllerNode, mock=True)
launcer.launch(KickerNode, mock=True)
launcer.spin()
