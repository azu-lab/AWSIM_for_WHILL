"""Consistency checks of the optimizer configuration files (no AWSIM / ROS 2 needed)."""

from dataclasses import fields
from pathlib import Path

import pytest
import yaml

from whill_optimizer.grpc_client import VehicleParameters
from whill_optimizer.paths import DEFAULT_SEARCH_SPACE, REPO_ROOT

VEHICLE_DIR = REPO_ROOT / "config" / "vehicle"
SETTABLE = {f.name for f in fields(VehicleParameters)}


def load_params(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get("vehicle_params", data)


def test_search_space_matches_runtime_parameters():
    with open(DEFAULT_SEARCH_SPACE) as f:
        space = yaml.safe_load(f)["parameters"]
    assert set(space) == SETTABLE
    for name, spec in space.items():
        assert spec["low"] < spec["high"], name
        assert spec["low"] <= spec["default"] <= spec["high"], name


@pytest.mark.parametrize("path", sorted(VEHICLE_DIR.rglob("*.yaml")), ids=lambda p: p.name)
def test_vehicle_yaml_only_uses_settable_parameters(path):
    params = load_params(path)
    assert set(params) <= SETTABLE, set(params) - SETTABLE


@pytest.mark.parametrize("name", ["whill_model_cr_optimized_4scenario_v2.yaml", "whill_model_cr_optimized_3scenario_v2.yaml"])
def test_optimized_parameters_are_inside_the_search_space(name):
    with open(DEFAULT_SEARCH_SPACE) as f:
        space = yaml.safe_load(f)["parameters"]
    params = load_params(VEHICLE_DIR / name)
    for key, value in params.items():
        assert space[key]["low"] <= value <= space[key]["high"], key


def test_initial_parameters_are_the_search_space_defaults():
    with open(DEFAULT_SEARCH_SPACE) as f:
        space = yaml.safe_load(f)["parameters"]
    params = load_params(VEHICLE_DIR / "whill_model_cr_initial.yaml")
    assert params == {name: spec["default"] for name, spec in space.items()}


def test_load_parameter_ranges():
    optimizer = pytest.importorskip("whill_optimizer.optimizer")
    ranges = optimizer.load_parameter_ranges(DEFAULT_SEARCH_SPACE)
    assert [r.name for r in ranges] == list(yaml.safe_load(open(DEFAULT_SEARCH_SPACE))["parameters"])
