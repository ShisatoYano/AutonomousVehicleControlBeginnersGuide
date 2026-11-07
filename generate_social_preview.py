"""
generate_social_preview.py

Generate the social preview from the four featured README demos.
"""

import argparse
from io import BytesIO
from pathlib import Path
import re

from PIL import Image, ImageDraw, ImageFont, ImageOps

PROJECT_ROOT = Path(__file__).resolve().parent
START = "<!-- featured-demos:start -->"
END = "<!-- featured-demos:end -->"


def featured_demos(readme: Path) -> list[tuple[str, Path]]:
    """Read labeled local GIF paths, in table order, relative to the README."""
    text = readme.read_text(encoding="utf-8")
    if text.count(START) != 1 or text.count(END) != 1 or text.index(START) > text.index(END):
        raise ValueError(f"{readme}: expected one ordered pair of {START} / {END} delimiters")
    block = text.split(START)[1].split(END)[0]
    images = re.findall(r"!\[([^\]]*)\]\(([^)]+)\)", block)
    if len(images) != 4:
        raise ValueError(f"{readme}: featured demos must contain exactly four Markdown GIF images")
    demos = []
    for label, source in images:
        path = Path(source)
        if not label.strip() or path.suffix.lower() != ".gif" or path.is_absolute() or "://" in source:
            raise ValueError(f"{readme}: use a descriptive label and a relative local GIF path: {source}")
        demos.append((label.strip(), readme.parent / path))
    return demos


def render_preview(demos: list[tuple[str, Path]]) -> bytes:
    """Fit deterministic midpoint frames into a fixed two-by-two RGB layout."""
    with Image.new("RGB", (1280, 640), "#101c30") as canvas:
        draw = ImageDraw.Draw(canvas)
        title_font = ImageFont.load_default(size=32)
        label_font = ImageFont.load_default(size=22)
        draw.text((24, 20), "Autonomous Vehicle Control", font=title_font, fill="white")
        for index, (label, path) in enumerate(demos):
            x = 24 + (index % 2) * 628
            y = 106 + (index // 2) * 276
            if draw.textlength(label, font=label_font) > 604:
                raise ValueError(f"{path}: shorten the featured image label to fit the preview")
            draw.text((x, y - 30), label, font=label_font, fill="white")
            draw.rectangle((x, y, x + 603, y + 237), fill="white")
            try:
                with Image.open(path) as source:
                    if source.format != "GIF":
                        raise ValueError("expected GIF format")
                    source.seek(source.n_frames // 2)
                    with source.convert("RGB") as frame:
                        with ImageOps.contain(frame, (604, 238), Image.Resampling.LANCZOS) as fitted:
                            canvas.paste(fitted, (x + (604 - fitted.width) // 2,
                                                  y + (238 - fitted.height) // 2))
            except (OSError, ValueError, EOFError) as error:
                raise ValueError(f"Cannot read featured GIF {path}: {error}") from error
        with BytesIO() as buffer:
            canvas.save(buffer, format="PNG", optimize=True)
            data = buffer.getvalue()
    if len(data) >= 1_000_000:
        raise ValueError("Preview must be below 1,000,000 bytes; choose simpler featured GIFs")
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--readme", type=Path, default=PROJECT_ROOT / "README.md")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "doc" / "social-preview.png")
    args = parser.parse_args()
    try:
        data = render_preview(featured_demos(args.readme))
        # Validate every input and the encoded size before creating any output.
        args.output.write_bytes(data)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Error: {error}\n")
    print(f"Generated {args.output} ({len(data):,} bytes)")


if __name__ == "__main__":
    main()
