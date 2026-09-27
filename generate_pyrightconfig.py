"""
generate_pyrightconfig.py

Automatically find the directories under src/components and src/simulations
that directly contain .py files, and write them to pyrightconfig.json as extraPaths.
It also updates python.analysis.extraPaths in .devcontainer/devcontainer.json
(for VSCode + Dev Containers users) with the same content.

Each script adds module search paths at runtime with sys.path.append(),
but pyright/Pylance (static analysis) cannot read them, so the same information is given separately.
When you add a new module (directory), just run this script again
to bring both configs up to date.

Author: Shisato Yano
"""
import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
TARGET_DIRS = ["src/components", "src/simulations"]
PYRIGHTCONFIG_PATH = PROJECT_ROOT / "pyrightconfig.json"
DEVCONTAINER_PATH = PROJECT_ROOT / ".devcontainer" / "devcontainer.json"
# devcontainer.json uses paths inside the container (workspaceFolder) as the base
CONTAINER_WORKSPACE = "/home/dev-user/workspace"


def find_extra_paths():
    extra_paths = []
    for target in TARGET_DIRS:
        base = PROJECT_ROOT / target
        for path in sorted(base.rglob("*")):
            if not path.is_dir():
                continue
            if any(p.suffix == ".py" for p in path.glob("*.py")):
                extra_paths.append(str(path.relative_to(PROJECT_ROOT)))
    return sorted(extra_paths)


def update_pyrightconfig(extra_paths):
    with open(PYRIGHTCONFIG_PATH, "w") as f:
        json.dump({"extraPaths": extra_paths}, f, indent=2)
        f.write("\n")
    print(f"Wrote {len(extra_paths)} paths to {PYRIGHTCONFIG_PATH}")


def update_devcontainer(extra_paths):
    with open(DEVCONTAINER_PATH) as f:
        config = json.load(f)

    container_paths = [f"{CONTAINER_WORKSPACE}/{p}" for p in extra_paths]
    config["customizations"]["vscode"]["settings"]["python.analysis.extraPaths"] = container_paths

    with open(DEVCONTAINER_PATH, "w") as f:
        json.dump(config, f, indent=4)
        f.write("\n")
    print(f"Wrote {len(container_paths)} paths to {DEVCONTAINER_PATH}")


def main():
    extra_paths = find_extra_paths()
    update_pyrightconfig(extra_paths)
    update_devcontainer(extra_paths)


if __name__ == "__main__":
    main()
