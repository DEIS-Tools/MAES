from pathlib import Path
import os

def get_root_dir():
    return Path(__file__).parents[1]

def get_src_dir():
    return Path(__file__).parent

def get_dir_contents(directory):
    os.chdir(os.path.join(get_root_dir(), "datasets", directory))
    return os.listdir()
