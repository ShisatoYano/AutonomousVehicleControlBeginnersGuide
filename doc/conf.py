"""
conf.py

Sphinx configuration of the web book built from doc/

Author: Shisato Yano
"""

import posixpath
import re
import runpy
import sys
from pathlib import Path

from sphinx.application import Sphinx

project = "Autonomous Vehicle Control Beginners Guide"
author = "Shisato Yano"
copyright = "2022, Shisato Yano"

extensions = [
    "myst_parser",
    "sphinxcontrib.mermaid",
    "sphinx_copybutton",
]

DOC_DIR = Path(__file__).absolute().parent
sys.path.append(str(DOC_DIR))

myst_enable_extensions = ["dollarmath", "amsmath"]
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 3

exclude_patterns = ["_build", "DESIGN_DOCUMENT.md"]
# Section pages of chapters 3 and 4 start at H2 until they are moved into the book (#170)
suppress_warnings = ["myst.header"]

html_theme = "sphinx_rtd_theme"
html_title = "Autonomous Vehicle Control Beginners Guide"
html_theme_options = {
    "navigation_depth": 3,
}
# Shows the "Edit on GitHub" link at the top of each page
html_context = {
    "display_github": True,
    "github_user": "ShisatoYano",
    "github_repo": "AutonomousVehicleControlBeginnersGuide",
    "github_version": "main",
    "conf_py_path": "/doc/",
}

GITHUB_URL = "https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide"
IMAGE_SUFFIXES = (".gif", ".png", ".jpg", ".jpeg", ".svg")
LINK_PATTERN = re.compile(r"(\]\(|src=\")([^)\"\s]+)")
# Section pages nested under their chapter in the sidebar. A toctree directive in the
# Markdown files would show up as a code block on GitHub, so it is added at build time
SECTION_PAGES = {
    "3_sensor_models/3_sensor_models": ["3_1_obstacle", "3_2_lidar", "3_3_sensors"],
    "4_path_tracking/4_path_tracking": [
        "4_1_stanley_controller", "4_2_mpc_controller", "4_3_mppi_controller"
    ],
}


def rewrite_link(docname: str, target: str, in_html: bool) -> str:
    """
    Return the link target to use in the book for a link written for GitHub
    docname: name of the page that has the link, relative to doc/ without suffix
    target: link target as written in the Markdown file
    in_html: True when the link is in a raw HTML tag, whose files Sphinx doesn't copy
    """

    if "://" in target or target.startswith(("#", "mailto:")):
        return target

    path, _, anchor = target.partition("#")
    if path.startswith("/"):
        repo_path = posixpath.normpath(path.lstrip("/"))
    else:
        repo_path = posixpath.normpath(posixpath.join("doc", posixpath.dirname(docname), path))

    if repo_path.startswith("doc/") and not in_html:
        relative = posixpath.relpath(repo_path[len("doc/"):], posixpath.dirname(docname) or ".")
        return relative + ("#" + anchor if anchor else "")

    if repo_path.lower().endswith(IMAGE_SUFFIXES):
        kind = "raw"
    elif posixpath.splitext(repo_path)[1] == "":
        kind = "tree"  # a directory, such as a simulation's folder
    else:
        kind = "blob"
    return f"{GITHUB_URL}/{kind}/main/{repo_path}" + ("#" + anchor if anchor else "")


def rewrite_links(app: Sphinx, docname: str, source: list[str]) -> None:
    # The pages use repository-root links (/doc/..., /src/...) so they also work on GitHub
    source[0] = LINK_PATTERN.sub(
        lambda m: m.group(1) + rewrite_link(docname, m.group(2), m.group(1) == 'src="'),
        source[0],
    )


def add_section_toctree(app: Sphinx, docname: str, source: list[str]) -> None:
    pages = SECTION_PAGES.get(docname)
    if pages is None:
        return
    title, _, body = source[0].partition("\n")
    toctree = "```{toctree}\n:hidden:\n\n" + "\n".join(pages) + "\n```\n"
    source[0] = title + "\n\n" + toctree + body


def draw_figures(app: Sphinx) -> None:
    """
    Run every figure script (doc/**/figures/*.py) and save its figure as an SVG
    next to it, so the pages include figures that are never committed as images
    """

    # matplotlib's plot directive would do this, but it doesn't work in MyST Markdown
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    import figure_style

    for script in sorted(DOC_DIR.glob("**/figures/*.py")):
        with plt.rc_context(figure_style.RC_PARAMS):
            runpy.run_path(str(script))
            plt.savefig(script.with_suffix(".svg"))
            plt.close("all")


def setup(app: Sphinx) -> None:
    app.connect("builder-inited", draw_figures)
    app.connect("source-read", rewrite_links)
    app.connect("source-read", add_section_toctree)
