import json
import logging
import os
import yaml
import rospy
from std_msgs.msg import String

logger = logging.getLogger("config_manager")


class ConfigManager:
    INSTANCE_MAP = {}

    PUBLISHER = rospy.Publisher('settings_changed', String, queue_size=10)
    NODE = rospy.init_node('config_manager', anonymous=True)

    def __init__(self, basename):
        logging.info("New config %s", basename)
        self.path = "configs/%s.yaml" % basename
        ConfigManager.INSTANCE_MAP[basename] = self

        self.config = {}
        self.reload()

    def __str__(self) -> str:
        return json.dumps(self.config, indent=1)

    def reload(self):
        if os.path.isfile(self.path):
            with open(self.path) as file:
                self.config = yaml.safe_load(file) or {}
        else:
            self.config = {}

    @staticmethod
    def instance(basename):
        return ConfigManager.INSTANCE_MAP.get(basename) or ConfigManager(basename)

    @classmethod
    def set_value(cls, key, value):
        basename, section, option = key.split("|")
        instance = ConfigManager.INSTANCE_MAP.get(basename)
        if instance:
            section_dict = instance.config.get(section, {})
            section_dict[option] = value
            instance.config[section] = section_dict
            instance.save()

        cls.PUBLISHER.publish(String(basename))

    @classmethod
    def get_value(cls, key, reload=True, default=None):
        basename, section, option = key.split("|")
        instance = ConfigManager.instance(basename)

        if reload:
            instance.reload()

        return instance.config.get(section, {}).get(option, default=None)

    def save(self):
        with open(self.path + ".part", "w") as file:
            yaml.dump(self.config, file, default_flow_style=False)

        os.rename(self.path + ".part", self.path)


if __name__ == "__main__":
    config = ConfigManager("settings")

    print(config)
    print(config.get_value("settings|turbo|full_blast"))
    print(config.get_value("settings|turbo|wont_Exist"))

    config.set_value("settings|turbo|full_blast", True)
    config.set_value("settings|wasd|volume", 0.78)
    from random import randint

    config.set_value("settings|pants|tomate", randint(1, 100))
    config.set_value("settings|pants|intel", 12)

    config = ConfigManager.instance("settings")
    print(config)

    config.save()
