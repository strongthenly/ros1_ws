#!/usr/bin/env python3
"""Start MAVROS only from a reviewed PX4 connection file."""
import argparse
import os
import subprocess

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    if not os.path.isfile(args.config):
        parser.error("control config does not exist: " + args.config)
    with open(args.config, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    required = ("px4_version", "fcu_url", "gcs_url", "mavlink_system_id", "mavlink_component_id")
    missing = [name for name in required if config.get(name) in (None, "")]
    if config.get("status") != "verified" or not config.get("failsafe_behavior_verified") or missing:
        parser.error("PX4 control topology is not verified; missing: " + ", ".join(missing))
    command = [
        "roslaunch", "mavros", "px4.launch",
        "fcu_url:=" + str(config["fcu_url"]),
        "gcs_url:=" + str(config["gcs_url"]),
        "tgt_system:=" + str(config["mavlink_system_id"]),
        "tgt_component:=" + str(config["mavlink_component_id"]),
    ]
    subprocess.check_call(command)


if __name__ == "__main__":
    main()
