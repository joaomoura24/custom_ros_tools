import os
import re
import yaml
import rospkg

"""Load config files"""

rp = rospkg.RosPack()

def ros_package_path(package: str) -> str:
    """Wrapper for rp.get_path - to ease effort when porting to ROS2."""
    return rp.get_path(package)

def replace_package(path: str) -> str:
    """Returns the absolute path to a file. The path can be given relative to ROS package in the format '{ros_package_name}/path/to/file'."""
    matches = re.findall(r'{.+?}', path)
    if len(matches) > 0:
        match = matches[0]
        package = match[1:-1]
        root = rp.get_path(package)
        path = path.replace(match, root)
    return path

def load_config(path: str) -> dict:
    """Load config from file."""
    path_ = replace_package(path)
    with open(path_, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    return config

def load_configs(s: str) -> dict:
    """Load config from string."""
    return yaml.load(s, Loader=yaml.FullLoader)

def config_to_str(config: dict) -> str:
    """Convert a configuration to string."""
    return yaml.dump(config)
