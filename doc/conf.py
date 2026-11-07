"""
conf.py

Sphinx configuration of the web book built from doc/

Author: Shisato Yano
"""

import posixpath
import re

from sphinx.application import Sphinx

project = "Autonomous Vehicle Control Beginners Guide"
author = "Shisato Yano"
copyright = "2022, Shisato Yano"

extensions = [
    "myst_parser",
    "sphinxcontrib.mermaid",
    "sphinx_copybutton",
]

myst_enable_extensions = ["dollarmath", "amsmath"]
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 3

exclude_patterns = ["_build", "DESIGN_DOCUMENT.md"]
# Section pages of chapters 3 and 4 start at H2 until they are moved into the book (#170)
suppress_warnings = ["myst.header"]

html_theme = "sphinx_book_theme"
html_title = "Autonomous Vehicle Control Beginners Guide"
html_theme_options = {
    "repository_url": "https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide",
    "repository_branch": "main",
    "path_to_docs": "doc",
    "use_repository_button": True,
    "use_edit_page_button": True,
    "use_issues_button": True,
}

GITHUB_URL = "https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide"
IMAGE_SUFFIXES = (".gif", ".png", ".jpg", ".jpeg", ".svg")
LINK_PATTERN = re.compile(r"(\]\(|src=\")([^)\"\s]+)")


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

    kind = "raw" if repo_path.lower().endswith(IMAGE_SUFFIXES) else "blob"
    return f"{GITHUB_URL}/{kind}/main/{repo_path}" + ("#" + anchor if anchor else "")


def rewrite_links(app: Sphinx, docname: str, source: list[str]) -> None:
    # The pages use repository-root links (/doc/..., /src/...) so they also work on GitHub
    source[0] = LINK_PATTERN.sub(
        lambda m: m.group(1) + rewrite_link(docname, m.group(2), m.group(1) == 'src="'),
        source[0],
    )


def setup(app: Sphinx) -> None:
    app.connect("source-read", rewrite_links)
