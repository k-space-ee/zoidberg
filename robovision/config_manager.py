import os
import atexit
from configparser import ConfigParser

class ConfigOption(object):
    """
    Proxy object for a configuration option,
    handles deferred saves etc
    """
    def __init__(self, manager, section, option, default=None, validator=None, type=str):
        self.manager = manager
        self.section = section
        self.option = option
        self.type = type
        self.validator = validator
        assert not default or not validator or validator(default), "Default value %s invalid" % default
        if default and not self.manager.cp.has_option(self.section, self.option):
            self.set_value(default)

        if validator and not validator(self.get_value()):
            raise ValueError("supplied value %s for option %s in section %s of %s invalid" %
                (self.get_value(), self.option, self.section, self.manager.path))
        print("ConfigOption(section=%s, option=%s)" % (repr(section), repr(option)))

    def get_value(self):
        if self.manager.cp.has_option(self.section, self.option):
            return self.type(self.manager.cp.get(self.section, self.option))
        raise ValueError("Option %s in section %s of %s not specified and no default supplied" %
            (self.option, self.section, self.manager.path))

    def set_value(self, value):
        if self.validator and not self.validator(value):
            raise ValueError("new value %s for option %s in section %s of %s invalid" %
                (value, self.option, self.section, self.manager.path))
        new_value = str(value)

        if not self.manager.cp.has_section(self.section):
            self.manager.cp.add_section(self.section)
        if self.manager.cp.has_option(self.section, self.option):
            current_value = self.manager.cp.get(self.section, self.option)
            if current_value != new_value:
                return self
        self.manager.cp.set(self.section, self.option, new_value)
        if self.manager.defer:
            self.manager.dirty = True
        else:
            self.manager.save()
        return self

class ConfigManager(object):
    def __init__(self, basename, defer=True):
        self.path = os.path.expanduser("~/.robovision/%s.conf" % basename)
        self.cp = ConfigParser()
        try:
            self.cp.readfp(open(self.path))
        except FileNotFoundError:
            pass
        self.defer = defer
        self.refs = {}
        self.dirty = False

        if self.defer:
            atexit.register(self.term_handler)

    def term_handler(self):
        if self.dirty:
            self.save()

    def get_value(self, *args):
        return self.get_option(*args).get_value()

    def get_option(self, section, option, *args, **kwargs):
        key = section, option
        if key not in self.refs:
            self.refs[key] = ConfigOption(self, section, option, *args, **kwargs)
        return self.refs[key]

    def save(self):
        self.cp.write(open(self.path + ".part", "w"))
        os.rename(self.path + ".part", self.path)
        print("Saved:", self.path)
        self.dirty = False

if __name__ == "__main__":
    config = ConfigManager("test")
    config.get_option("global", "kicker", type=int, default=12, validator=lambda j: j <= 14).get_value()
    config.get_option("global", "grabber", type=int, default=13).get_value()

