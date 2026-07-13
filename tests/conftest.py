import os

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--port",
        default=os.environ.get("GATEKEEPER_PORT"),
        help="Gatekeeper serial port; auto-detected when omitted.",
    )

