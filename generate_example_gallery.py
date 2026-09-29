"""
generate_example_gallery.py

Scan src/simulations, collect each algorithm's demo images (gif/png),
and generate doc/EXAMPLES.md.

Algorithms used to be added to README.md's table of contents and body by hand,
so newly added simulations were sometimes left out
(e.g. gradient_path_planning, lidar_obstacle_sensing, point_cloud_search).
This script builds the list only from the existing directories and image files,
so no simulation gets left out.

The heading, description and author of each entry are read from the module
docstring of the simulation scripts, so nobody needs to edit this script or
doc/EXAMPLES.md by hand. The docstring of every script under src/simulations
must consist of exactly these lines, with nothing else (the test fails otherwise).
If a directory has multiple scripts, their Title must be the same:

    \"\"\"
    pure_pursuit_path_tracking.py

    Title: Pure Pursuit Path Tracking
    Description: One-line summary shown under the heading
    Author: Your Name
    \"\"\"

After changes are merged into main, .github/workflows/Update_Examples_Gallery.yml
runs this script and commits doc/EXAMPLES.md.

Author: Shisato Yano
"""
import ast
import os
import re
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
SIMULATIONS_DIR = PROJECT_ROOT / "src" / "simulations"
OUTPUT_PATH = PROJECT_ROOT / "doc" / "EXAMPLES.md"

IMAGE_EXTENSIONS = (".gif", ".png", ".jpg")

# Display order of categories. Categories not listed here are still included,
# appended in alphabetical order, so adding a new category needs no edit here
CATEGORY_ORDER = [
    "localization",
    "mapping",
    "path_planning",
    "path_tracking",
    "perception",
    "course",
]

# Lines 3-5 of the simulation docstring template; line 1 is the file name and line 2 is empty
DOCSTRING_FIELDS = ("Title", "Description", "Author")
DOCSTRING_FIELD_LINE = re.compile(r"^(\w+): (\S.*)$")


def _default_title(dir_name):
    return dir_name.replace("_", " ").title()


def _relative_to_repo_root(path):
    return path.relative_to(PROJECT_ROOT).as_posix()


def _relative_to_output_dir(path):
    # Markdown resolves image paths from its own directory (doc/), not the repository root
    return Path(os.path.relpath(path, OUTPUT_PATH.parent)).as_posix()


def _read_docstring(py_path):
    """
    Returns: {"Title": ..., "Description": ..., "Author": ...} from the module docstring
    ast is used instead of importing the file, so the simulation is not executed
    Raises ValueError with the file to fix when the docstring doesn't follow the template
    """
    docstring = ast.get_docstring(ast.parse(py_path.read_text(encoding="utf-8"))) or ""
    lines = docstring.splitlines()
    matches = [DOCSTRING_FIELD_LINE.match(line) for line in lines[2:]]
    if (len(lines) != 5 or lines[0] != py_path.name or lines[1] != ""
            or not all(matches) or tuple(m.group(1) for m in matches) != DOCSTRING_FIELDS):
        raise ValueError(
            f"{_relative_to_repo_root(py_path)}: the docstring must be exactly the file name, an empty line, "
            f"and one line each of {', '.join(k + ':' for k in DOCSTRING_FIELDS)} in this order. "
            "See the template in generate_example_gallery.py"
        )
    return {m.group(1): m.group(2).strip() for m in matches}


def _read_entry(sim_dir, py_paths):
    """
    Returns: (title, [description, ...], [author, ...]) of a simulation directory
    """
    docstrings = [_read_docstring(p) for p in py_paths]
    titles = {d["Title"] for d in docstrings}
    if len(titles) != 1:
        raise ValueError(f"{_relative_to_repo_root(sim_dir)}: all scripts must have the same 'Title:', found {sorted(titles)}")

    # Scripts in the same directory may have different authors, so credit all of them
    authors = []
    for d in docstrings:
        for author in d["Author"].split(","):
            if author.strip() not in authors:
                authors.append(author.strip())

    return titles.pop(), [d["Description"] for d in docstrings], authors


def _category_dirs():
    dirs = [d for d in SIMULATIONS_DIR.iterdir() if d.is_dir()]
    known = [d for name in CATEGORY_ORDER for d in dirs if d.name == name]
    return known + sorted(d for d in dirs if d.name not in CATEGORY_ORDER)


def collect_gallery_entries():
    """
    Returns: {category_title: [(title, [description, ...], [author, ...], [relative image path, ...]), ...]}
    in display order. Only directories that contain both a .py file and an image file (gif/png/jpg) are included,
    but every script's docstring is validated so a missing field is caught before a demo image is added
    """
    gallery = {}

    for category_dir in _category_dirs():
        entries = []
        for sim_dir in sorted(category_dir.iterdir()):
            if not sim_dir.is_dir():
                continue
            py_paths = sorted(sim_dir.glob("*.py"))
            if not py_paths:
                continue
            title, descriptions, authors = _read_entry(sim_dir, py_paths)
            images = sorted(
                p for p in sim_dir.iterdir()
                if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
            )
            if not images:
                continue
            entries.append((title, descriptions, authors, [_relative_to_output_dir(p) for p in images]))

        if entries:
            gallery[_default_title(category_dir.name)] = entries

    return gallery


def render_markdown(gallery):
    """
    Build a Markdown string from the return value of collect_gallery_entries()
    """
    lines = [
        "# Examples of Simulation",
        "",
        "This page is generated by `generate_example_gallery.py` from the demo "
        "images under `src/simulations` and updated automatically after merge. "
        "Do not edit it by hand — edit the docstring of the simulation script "
        "instead.",
        "",
    ]

    for category_title, entries in gallery.items():
        lines.append(f"## {category_title}")
        lines.append("")
        for title, descriptions, authors, image_paths in entries:
            lines.append(f"### {title}")
            for description in descriptions:
                lines.append(description + "  ")
            lines.append(f"Author: {', '.join(authors)}  ")
            for image_path in image_paths:
                lines.append(f"![]({image_path})  ")
            lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def generate():
    """
    Generate and return the content of doc/EXAMPLES.md from the current state of src/simulations
    """
    return render_markdown(collect_gallery_entries())


def main():
    OUTPUT_PATH.write_text(generate(), encoding="utf-8")
    print(f"Generated {OUTPUT_PATH.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    main()
