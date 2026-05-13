Import("env")

import importlib.util
from pathlib import Path


project_dir = Path(env.subst("$PROJECT_DIR"))
repo_root = project_dir.parent
helper_path = repo_root / "platformio_tools" / "generate_platformio_variants.py"

spec = importlib.util.spec_from_file_location(
    "generate_platformio_variants", helper_path
)
helper_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper_module)

framework_dir = Path(env.PioPlatform().get_package_dir("framework-arduino-mbed"))
updated = helper_module.generate_all(
    repo_root=repo_root,
    framework_dir=framework_dir,
)
if updated:
    print("[gatekeeper-variants] Generated " + ", ".join(updated))
