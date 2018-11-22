import json
from datetime import datetime, timedelta
from collections import deque


class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, datetime):
            return obj.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        if isinstance(obj, timedelta):
            f = obj.total_seconds()
            mins = f // 60
            return "%02d:%06.03f" % (mins, f % 60)
        return json.JSONEncoder.default(self, obj)


class WebsocketLogHandler:
    def __init__(self):
        self.started = datetime.utcnow()
        self.queue = deque(maxlen=1000)
        self.websockets = set()

    def emit(self, record):
        # messenger.Messages.logging
        timestamp = datetime.utcfromtimestamp(record.header.stamp.secs)
        buf = json.dumps(dict(
            action="log-entry",
            created=timestamp,
            uptime=timestamp - self.started,
            file=record.file.replace('.py', ''),
            message=record.msg,
            severity=record.level), cls=MyEncoder)
        self.queue.append(buf)
        for websocket in self.websockets:
            websocket.send(buf)
