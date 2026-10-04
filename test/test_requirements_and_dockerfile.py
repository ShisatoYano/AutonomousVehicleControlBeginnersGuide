"""Regression checks for dependency installation files."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_requirements_omit_unused_packages():
    requirements = (ROOT / "requirements.txt").read_text()
    names = []
    for line in requirements.splitlines():
        spec = line.split("#", 1)[0].strip()
        if not spec:
            continue
        name = spec.split(">=")[0].split("==")[0].strip().lower()
        names.append(name)
    assert "pandas" not in names
    assert "seaborn" not in names
    assert "pytest-cov" not in names
    assert "casadi" in names
    assert "do-mpc" in names


def test_dockerfile_does_not_redirect_version_pins():
    dockerfile = (ROOT / "Dockerfile").read_text()
    # An unquoted >= in a RUN instruction is a shell redirect, so the pin is ignored.
    for line in dockerfile.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        assert ">=" not in stripped
    assert "pip install --user -r requirements.txt" in dockerfile
