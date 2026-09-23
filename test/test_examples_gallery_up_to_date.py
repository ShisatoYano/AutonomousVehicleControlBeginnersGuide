"""
Test that doc/EXAMPLES.md is up to date with generate_example_gallery.py's output

Author: Shisato Yano
"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).absolute().parent) + "/..")
import generate_example_gallery as gallery


def test_examples_gallery_up_to_date():
    committed = gallery.OUTPUT_PATH.read_text()
    regenerated = gallery.generate()

    assert committed == regenerated, (
        "doc/EXAMPLES.md is stale. Run `python generate_example_gallery.py` "
        "and commit the result."
    )
