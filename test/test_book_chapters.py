"""
Test that every simulation has a chapter in the book

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
