Import("env")

import importlib.util
from pathlib import Path


def _load_gatekeeper_upload_module():
    helper_path = (
        Path(env.subst("$PROJECT_DIR")).parent
        / "firmware_uploader"
        / "gatekeeper_upload.py"
    )
    spec = importlib.util.spec_from_file_location("gatekeeper_upload", helper_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


if not env.get("GATEKEEPER_USB_BUNDLE_UPLOAD"):
    _gatekeeper_upload = _load_gatekeeper_upload_module()

    def _before_upload(source, target, env):
        del source, target
        _gatekeeper_upload.run_pre_upload(env)

    def _after_upload(source, target, env):
        del source, target
        _gatekeeper_upload.run_post_upload(env)

    env.AddPreAction("upload", _before_upload)
    env.AddPostAction("upload", _after_upload)
