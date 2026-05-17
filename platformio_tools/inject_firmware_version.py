import subprocess
Import("env")


def _get_git_commit_hash():
    try:
        commit_hash = subprocess.check_output([
            "git", "rev-parse", "--short", "HEAD"
        ]).decode("utf-8").strip()
        if not commit_hash:
            return "UNKNOWN"
        return commit_hash
    except Exception:
        return "UNKNOWN"


def _get_head_tag():
    try:
        tags = subprocess.check_output(
            ["git", "tag", "--points-at", "HEAD"]
        ).decode("utf-8").splitlines()
    except Exception:
        return None

    return tags[0] if tags else None


_firmware_version = _get_git_commit_hash()
env.Append(CPPDEFINES=[("FIRMWARE_VERSION_HASH", _firmware_version)])

_tag = _get_head_tag()
if _tag:
    env.Append(CPPDEFINES=[("FIRMWARE_VERSION_TAG", _tag)])
