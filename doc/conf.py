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
    "sphinx_sitemap",
    "sphinxext.opengraph",
]

DOC_DIR = Path(__file__).absolute().parent
sys.path.append(str(DOC_DIR))

myst_enable_extensions = ["dollarmath", "amsmath"]
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 3

exclude_patterns = ["_build"]

html_theme = "sphinx_rtd_theme"
html_title = "Autonomous Vehicle Control Beginners Guide"
html_theme_options = {
    "navigation_depth": 3,
}
templates_path = ["_templates"]

# Search engines and link previews: canonical URLs and sitemap.xml come from html_baseurl,
# and the description of each page from its first paragraph (notes are skipped)
BOOK_URL = "https://shisatoyano.github.io/AutonomousVehicleControlBeginnersGuide/"
html_baseurl = BOOK_URL
sitemap_url_scheme = "{link}"
sitemap_excludes = ["search.html", "genindex.html"]
ogp_site_url = BOOK_URL
ogp_site_name = html_title
ogp_description_length = 160
# The demo GIF at the top of the README, until the social preview image of #88 exists
ogp_image = ("https://raw.githubusercontent.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/"
             "main/src/simulations/mapping/ndt_map_construction/ndt_map_construction.gif")
ogp_image_alt = "A vehicle building a map from LiDAR points in the simulation"
ogp_social_cards = {"enable": False}
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
CODE_FENCE_PATTERN = re.compile(r"^\s*(`{3,}|~{3,})\s*(.*)$")
# A link from a chapter page to a page in its own folder, after rewrite_links()
SECTION_LINK_PATTERN = re.compile(r"\]\(([^)/#\s]+)\.md[)#]")


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
    # The pages use repository-root links (/doc/..., /src/...) so they also work on GitHub.
    # Code blocks are shown as written; directive blocks such as {figure} and {note} are not code
    lines = []
    fences = []  # open blocks as (fence, is_code), innermost last
    for line in source[0].split("\n"):
        match = CODE_FENCE_PATTERN.match(line)
        closes = (match is not None and match.group(2) == "" and bool(fences)
                  and match.group(1)[0] == fences[-1][0][0]
                  and len(match.group(1)) >= len(fences[-1][0]))
        in_code = bool(fences) and fences[-1][1]
        if closes:
            fences.pop()
        elif in_code:
            pass
        elif match:
            fences.append((match.group(1), not match.group(2).startswith("{")))
        else:
            line = LINK_PATTERN.sub(
                lambda m: m.group(1) + rewrite_link(docname, m.group(2), m.group(1) == 'src="'),
                line,
            )
        lines.append(line)
    source[0] = "\n".join(lines)


def add_section_toctree(app: Sphinx, docname: str, source: list[str]) -> None:
    # Nests the section pages that a chapter page links to under it in the sidebar, in the
    # order of the links. A toctree in the Markdown would show as a code block on GitHub
    if not docname.endswith("/index"):
        return
    pages = list(dict.fromkeys(
        page for page in SECTION_LINK_PATTERN.findall(source[0]) if page != "index"
    ))
    if not pages:
        return
    title, _, body = source[0].partition("\n")
    toctree = "```{toctree}\n:hidden:\n\n" + "\n".join(pages) + "\n```\n"
    source[0] = title + "\n\n" + toctree + body


def drop_front_page_title_suffix(app: Sphinx, pagename: str, templatename: str,
                                 context: dict, doctree) -> None:
    # The front page's heading is the book's title, so "<title> — <html_title>" would repeat it
    if pagename == app.config.root_doc:
        context["docstitle"] = None


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
    app.connect("html-page-context", drop_front_page_title_suffix)
