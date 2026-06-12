"""Manually restart the robot's rosbridge stack from the laptop.

The cure for the Melodic rosbridge silent-drop registration bug (and the
wedged /voltage publisher after a power cycle): restart
rosbridge-dashboard.service over SSH. The runner preflight and the
dashboard's RESTART ROSBRIDGE button trigger the same heal automatically;
this CLI is the by-hand path.

  python tools/heal_rosbridge.py                 # home profile
  python tools/heal_rosbridge.py --profile hotspot

Drops the camera and every client connection for ~15-30 s -- only run it
with the robot stationary.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ai import config  # noqa: E402
from ai.common.heal import heal, host_from_url  # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", choices=sorted(config.PROFILES),
                   default=config.DEFAULT_PROFILE)
    args = p.parse_args(argv)
    host = host_from_url(config.PROFILES[args.profile]["rosbridge_url"])
    print(f"restarting rosbridge-dashboard.service on {host} ...")
    ok, detail = heal(host)
    print(detail)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
