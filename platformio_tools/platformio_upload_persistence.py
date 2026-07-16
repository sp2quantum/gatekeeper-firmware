Import("env")

import importlib.util
from pathlib import Path


def _load_upload_persistence_module():
    helper_path = (
        Path(env.subst("$PROJECT_DIR")).parent
        / "platformio_tools"
        / "upload_persistence.py"
    )
    spec = importlib.util.spec_from_file_location("upload_persistence", helper_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


if not env.get("GATEKEEPER_USB_BUNDLE_UPLOAD"):
    _upload_persistence = _load_upload_persistence_module()

    def _before_upload(source, target, env):
        del source, target
        _upload_persistence.run_pre_upload(env)

    def _after_upload(source, target, env):
        del source, target
        _upload_persistence.run_post_upload(env)

    env.AddPreAction("upload", _before_upload)
    env.AddPostAction("upload", _after_upload)
