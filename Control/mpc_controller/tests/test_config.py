import pathlib

import pytest
import yaml

CONFIG_PATH = pathlib.Path(__file__).resolve().parent.parent / "config" / "mpc_controller.yaml"


@pytest.fixture(scope="module")
def config():
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_top_level_keys(config):
    for key in [
        "control_rate",
        "publish_debug",
        "reference_frame",
        "base_frame",
        "path_resolution_m",
        "profile_upsample_factor",
        "path_topic",
        "speed_profile_topic",
        "ego_topic",
        "imu_topic",
        "cmd_topic",
        "lateral_mpc",
        "longitudinal_mpc",
    ]:
        assert key in config, f"Missing key: {key}"


def test_lateral_dynamic_horizon_bounds(config):
    dyn = config["lateral_mpc"]["dynamic_horizon"]
    assert dyn["min_steps"] < dyn["max_steps"], "min_steps should be < max_steps"
    assert dyn["quantize_steps"] >= 1
    assert 0.0 <= dyn["kappa_low"] <= dyn["kappa_high"]


def test_longitudinal_limits(config):
    lon = config["longitudinal_mpc"]
    assert lon["u_min"] < lon["u_max"], "u_min must be less than u_max"
    assert lon["v_min"] <= lon["v_max"], "v_min must be <= v_max"


def test_topics_unique(config):
    topics = {
        config["path_topic"],
        config["speed_profile_topic"],
        config["ego_topic"],
        config["imu_topic"],
        config["cmd_topic"],
    }
    assert len(topics) == 5, "Topic names should be unique to avoid collisions"
