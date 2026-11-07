"""
Test of generate_social_preview.py
"""

from pathlib import Path
import re
import subprocess
import sys

from PIL import Image, ImageChops
import pytest

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SCRIPT = PROJECT_ROOT / "generate_social_preview.py"


def test_repository_preview(tmp_path: Path) -> None:
    readme = (PROJECT_ROOT / "README.md").read_text()
    block = readme.split("<!-- featured-demos:start -->")[1].split("<!-- featured-demos:end -->")[0]
    paths = re.findall(r"!\[[^\]]+\]\(([^)]+)\)", block)
    assert len(paths) == 4
    assert all((PROJECT_ROOT / path).is_file() for path in paths)
    output = tmp_path / "preview.png"
    command = [sys.executable, str(SCRIPT), "--output", str(output)]
    subprocess.run(command, cwd=tmp_path, check=True)
    original = output.read_bytes()
    assert len(original) < 1_000_000
    with Image.open(output) as preview:
        assert preview.format == "PNG"
        assert preview.size == (1280, 640)
        assert preview.mode == "RGB"
        preview.load()
    subprocess.run(command, cwd=tmp_path, check=True)
    assert output.read_bytes() == original


def test_featured_selection_and_aspect_ratio(tmp_path: Path) -> None:
    # Single-frame landscape and portrait GIFs must be fitted, never stretched.
    for name, size in [("wide", (200, 100)), ("tall", (100, 200))]:
        with Image.new("RGB", size, "red") as frame:
            frame.save(tmp_path / f"{name}.gif")
    frames = [Image.new("RGB", (100, 100), color) for color in ("red", "lime", "blue")]
    frames[0].save(tmp_path / "animated.gif", save_all=True, append_images=frames[1:], duration=100)
    for frame in frames:
        frame.close()
    readme = tmp_path / "README.md"
    text = "![ignored badge](missing.svg)\n<!-- featured-demos:start -->\n" + "\n".join(
        f"![Demo {i}]({name}.gif)" for i, name in enumerate(["wide", "tall", "animated", "wide"])
    ) + "\n<!-- featured-demos:end -->"
    readme.write_text(text)
    output = tmp_path / "preview.png"
    command = [sys.executable, str(SCRIPT), "--readme", str(readme), "--output", str(output)]
    subprocess.run(command, check=True)
    with Image.open(output) as preview:
        original = preview.copy()
    # Red/green bounding boxes verify both frame selection and unchanged aspect ratio.
    for box, ratio, color in [((24, 106, 628, 344), 2, (255, 0, 0)),
                               ((652, 106, 1256, 344), 0.5, (255, 0, 0)),
                               ((24, 382, 628, 620), 1, (0, 255, 0))]:
        with original.crop(box) as tile:
            mask = Image.new("1", tile.size)
            mask.putdata([tile.getpixel((x, y)) == color
                          for y in range(tile.height) for x in range(tile.width)])
            bounds = mask.getbbox()
            assert bounds is not None
            assert (bounds[2] - bounds[0]) / (bounds[3] - bounds[1]) == pytest.approx(ratio, abs=0.01)
            mask.close()
    readme.write_text(text.replace("(wide.gif)", "(tall.gif)", 1))
    subprocess.run(command, check=True)
    with Image.open(output) as changed:
        bounds = ImageChops.difference(original, changed).getbbox()
        assert bounds is not None
        assert bounds[0] >= 24 and bounds[2] <= 628
        assert bounds[1] >= 106 and bounds[3] <= 344
    original.close()


@pytest.mark.parametrize("problem, message", [
    ("delimiters", "delimiters"), ("count", "exactly four"),
    ("missing", "missing.gif"), ("corrupt", "corrupt.gif"),
])
def test_invalid_input(tmp_path: Path, problem: str, message: str) -> None:
    with Image.new("RGB", (10, 10)) as frame:
        frame.save(tmp_path / "valid.gif")
    (tmp_path / "corrupt.gif").write_text("not a GIF")
    images = ["![Demo](valid.gif)"] * 4
    if problem == "count":
        images.pop()
    if problem in ("missing", "corrupt"):
        images[-1] = f"![Demo]({problem}.gif)"
    text = "\n".join(images)
    if problem != "delimiters":
        text = f"<!-- featured-demos:start -->\n{text}\n<!-- featured-demos:end -->"
    readme = tmp_path / "README.md"
    readme.write_text(text)
    output = tmp_path / "preview.png"
    result = subprocess.run([sys.executable, str(SCRIPT), "--readme", str(readme),
                             "--output", str(output)], capture_output=True, text=True)
    assert result.returncode != 0
    assert message in result.stderr
    assert not output.exists()
