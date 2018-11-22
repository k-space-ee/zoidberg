import json
import logging
import os

import yaml

import messenger

logger = logging.getLogger("config_manager")


class ConfigManager:
    INSTANCE_MAP = {}

    publisher = messenger.Publisher('settings_changed', messenger.Messages.string)

    def __init__(self, basename):
        logger.info("New config %s", basename)
        self.path = "config/%s.yaml" % basename
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
        instance = ConfigManager.instance(basename)
        if instance:
            section_dict = instance.config.get(section, {})
            section_dict[option] = value
            instance.config[section] = section_dict
            instance.save()
            logger.info("%s -> %s", key, value)

        cls.publisher.publish(basename)

    @classmethod
    def get_value(cls, key, default=None, reload=True):
        basename, section, option, *_ = [*key.split("|"), None, None]
        instance = ConfigManager.instance(basename)

        if reload:
            instance.reload()

        value = instance.config

        if section:
            value = value.get(section, {})

        if section and option:
            value = value.get(option, default)

        return value

    def save(self):
        with open(self.path + ".part", "w") as file:
            yaml.dump(self.config, file, default_flow_style=False)

        os.rename(self.path + ".part", self.path)


if __name__ == "__main__":
    from random import randint

    messenger.Node('test')

    print(ConfigManager.get_value("test|turbo|full_blast"), "<- might exist")
    print(ConfigManager.get_value("test|turbo|wont_Exist"), "<- can't exist")

    ConfigManager.set_value("test|turbo|full_blast", True)
    ConfigManager.set_value("test|wasd|volume", 0.78)

    ConfigManager.set_value("test|pants|tomate", randint(1, 100))
    ConfigManager.set_value("test|pants|intel", 12)
    ConfigManager.set_value("test|pants|something", True)
    ConfigManager.set_value("test|pants|something", [1, 2, 3, 4])

    print(ConfigManager.get_value("test|pants", reload=True))
    print(ConfigManager.get_value("test", reload=True))

    imgrec = [
        ('color|field|luma', [64, 255]),
        ('color|field|chroma blue', [0, 149]),
        ('color|field|chroma red', [178, 229]),
        ('color|ball|luma', [73, 221]),
        ('color|ball|chroma blue', [63, 76]),
        ('color|ball|chroma red', [62, 151]),
        ('color|goal A|luma', [63, 232]),
        ('color|goal A|chroma blue', [89, 255]),
        ('color|goal A|chroma red', [116, 156]),
        ('color|goal B|luma', [18, 255]),
        ('color|goal B|chroma blue', [54, 68]),
        ('color|goal B|chroma red', [142, 176]),
    ]

    for k, v in imgrec:
        ConfigManager.set_value(k, v)
