"""
save_gif.py

Author: Aikaterini Katsampoula
"""

import sys


def is_save_gif_enabled():
    """Return GIF output when requested on the command line"""
    return "--save-gif" in sys.argv
