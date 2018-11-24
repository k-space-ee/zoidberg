from threading import Thread, Event
import logging

logger = logging.getLogger("threading")
from collections import deque
from queue import Queue, Empty
import ctypes

libc = ctypes.cdll.LoadLibrary('libc.so.6')
from time import sleep, time


class ThreadManager(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.threads = set()

    def register(self, thread):
        self.threads.add(thread)

    def run(self):
        while False:
            sleep(1)
            now = time()
            for thread in self.threads:
                if not thread.tid:
                    continue
                if thread.running:
                    if thread.alive:
                        if now - thread.last_product < 1.0 and len(thread.rate) > 0:
                            fps = len(thread.rate) / sum(thread.rate)
                            latency = sum(thread.latency) / len(thread.latency)
                            utilization = latency * 100 * fps
                            if utilization > 99:
                                logger.warn("%s thread (PID %d) gets %.2f fps, %.1fms latency, choking",
                                            thread.__class__.__name__, thread.tid, fps, latency * 1000)
                            else:
                                logger.info("%s thread (PID %d) gets %.2f fps, %.1fms latency, %d%% utilization",
                                            thread.__class__.__name__, thread.tid, fps, latency * 1000, utilization)
                        else:
                            logger.info("%s thread (PID %d) stuck", thread.__class__.__name__, thread.tid)
                    else:
                        logger.info("%s thread (PID %d) not enabled", thread.__class__.__name__, thread.tid)
                else:
                    logger.info("%s thread not started yet", thread.__class__.__name__)


class ManagedThread(Thread):

    def __init__(self, upstream_producer=None, framedrop=0, lossy=True):
        Thread.__init__(self)
        self.running = False
        self.daemon = True
        self.wake = Event()  # wake up signal for the /dev observer
        self.tid = 0
        self.alive = False
        self.latency = deque(maxlen=10)
        self.rate = deque(maxlen=10)
        self.queues = set()  # Consumer queues
        self.framedrop = framedrop
        self.subscribers = set()
        self.last_product = 0

        if upstream_producer:
            if hasattr(upstream_producer, "get_queue"):
                self.queue = upstream_producer.get_queue(lossy)  # Queue of frames
                self.args = ()
            else:
                self.queue = None
                self.args = upstream_producer,
        else:
            self.queue = None
            self.args = ()

    def produce(self, *args):
        for output_queue, lossy in self.queues:
            if lossy:
                try:
                    output_queue.get_nowait()
                except Empty:
                    pass
            output_queue.put(args)
        self.last_product = time()

    def get_queue(self, lossy=True):
        """
        Create queue for new consumer
        """
        q = Queue(maxsize=1)
        self.queues.add((q, lossy))
        return q

    def enable(self):
        self.alive = True
        self.wake.set()
        logger.info("Enabling thread %s (PID %d)", self.__class__.__name__, self.tid)

    def disable(self):
        self.wake.clear()
        self.alive = False
        logger.info("Disabling thread %s (PID %d)", self.__class__.__name__, self.tid)

    def toggle(self):
        if self.alive:
            self.disable()
        else:
            self.enable()

    def stop(self):
        self.running = False
        self.alive = False
        self.wake.set()

    def on_enabled(self, *args):
        pass

    def on_disabled(self, *args):
        pass

    @property
    def average_fps(self):
        if self.rate:
            return len(self.rate) / sum(self.rate)

    @property
    def average_latency(self):
        if self.latency:
            return sum(self.latency) / len(self.latency)

    def run(self):
        self.tid = libc.syscall(186)
        logger.info("%s thread spawned with PID %d", self.__class__.__name__, self.tid)
        self.running = True

        while self.running:  # Exit with .stop()
            self.wake.wait()
            first = True
            while self.alive:  # Toggle with .toggle(), .enable(), .disable()
                then = time()
                if self.queue:
                    for skips in range(0, self.framedrop):
                        self.queue.get()
                    args = self.queue.get()
                else:
                    args = self.args
                if first:
                    self.on_enabled(*args)
                    first = False
                then2 = time()
                try:
                    self.step(*args)
                except TypeError:
                    print(self.__class__.__name__, ".step() failed to make use of", args)
                    raise
                now = time()
                self.latency.append(now - then2)
                self.rate.append(now - then)
            self.on_disabled()
