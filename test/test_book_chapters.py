"""
Test that every simulation has a chapter in the book, and that every section page has the
headings of the writing guide

Author: Shisato Yano
"""

from pathlib import Path
import re

ROOT_DIR = Path(__file__).absolute().parent.parent
SIMULATIONS_DIR = ROOT_DIR / "src" / "simulations"
DOC_DIR = ROOT_DIR / "doc"

# Matches the links to simulation folders on the chapter pages, e.g. (/src/simulations/localization/kalman_filter_1d)
SIMULATION_LINK = re.compile(r"\(/src/simulations/([^/)]+/[^/)]+)\)")


def simulation_dirs() -> list[str]:
    return sorted(
        f"{d.parent.name}/{d.name}"
        for d in SIMULATIONS_DIR.glob("*/*")
        if d.is_dir() and any(d.glob("*.py"))
    )


def linked_simulations() -> list[str]:
    links = []
    for page in sorted(DOC_DIR.glob("*/index.md")):
        links += SIMULATION_LINK.findall(page.read_text(encoding="utf-8"))
    return links


def test_every_simulation_is_in_exactly_one_chapter():
    sims = simulation_dirs()
    links = linked_simulations()

    missing = [s for s in sims if s not in links]
    assert not missing, (
        f"Add these simulations to a section of a chapter page (doc/<chapter>/index.md): {missing}"
    )

    duplicated = sorted({s for s in links if links.count(s) > 1})
    assert not duplicated, f"These simulations are in more than one chapter: {duplicated}"

    unknown = sorted(set(links) - set(sims))
    assert not unknown, f"Chapter pages link to simulations that don't exist: {unknown}"


# The headings of the theory layer, in this order (doc/WRITING_GUIDE.md, "Structure of a chapter")
SECTION_HEADINGS = [
    "Problem and intuition",
    "Theory",
    "Algorithm",
    "Limits",
    "Check your understanding",
    "References",
]
CODE_LAYER_HEADING = "Where the equations live in the code"
CODE_LAYER_NOTE = "The code walkthrough, experiments and exercises are added after the code roadmap's Phase 1"


def section_pages() -> list[Path]:
    # Chapter 1 is a practical guide, not algorithm sections
    return sorted(
        page for page in DOC_DIR.glob("*/*.md")
        if page.name != "index.md" and page.parent.name != "getting_started"
    )


def test_every_section_has_the_headings_of_the_writing_guide():
    for page in section_pages():
        text = page.read_text(encoding="utf-8")
        headings = re.findall(r"^## (.+?)\s*$", text, flags=re.M)
        required = [h for h in headings if h in SECTION_HEADINGS]
        name = page.relative_to(ROOT_DIR)
        assert required == SECTION_HEADINGS, (
            f"{name} must have the headings {SECTION_HEADINGS} in this order, but has {required}"
        )
        if CODE_LAYER_HEADING not in headings:
            assert CODE_LAYER_NOTE in text, (
                f"{name} has no code layer yet, so it needs the note of the writing guide about it"
            )
