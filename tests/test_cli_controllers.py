"""
Checks that the --controller CLI option stays in sync with the controller
packages in acsl_pychrono/control. These tests do not need PyChrono.
"""
import ast
import sys
from pathlib import Path

import pytest

from acsl_pychrono import get_cli_args
from acsl_pychrono.config.config import MissionConfig
from acsl_pychrono.control import available_controllers

CONTROL_DIR = Path(__file__).resolve().parent.parent / "acsl_pychrono" / "control"


def parse_cli(monkeypatch, *argv):
  monkeypatch.setattr(sys, "argv", ["main.py", *argv])
  return get_cli_args()


@pytest.mark.parametrize("controller", available_controllers())
def test_cli_accepts_every_discovered_controller(monkeypatch, controller):
  args = parse_cli(monkeypatch, "--controller", controller)
  assert args.controller == controller


@pytest.mark.parametrize("controller", [
  "MRACwithRKHS",
  "TwoLayerMRACwithRKHS",
  "HybridMRACwithRKHS",
  "HybridTwoLayerMRACwithRKHS",
])
def test_cli_accepts_rkhs_controllers(monkeypatch, controller):
  args = parse_cli(monkeypatch, "--controller", controller)
  assert args.controller == controller


def test_cli_accepts_default_controller(monkeypatch):
  default = MissionConfig().controller_type
  args = parse_cli(monkeypatch, "--controller", default)
  assert args.controller == default


def test_cli_rejects_unknown_controller(monkeypatch):
  with pytest.raises(SystemExit):
    parse_cli(monkeypatch, "--controller", "NotAController")


def test_helper_packages_are_not_controllers():
  assert "rkhs" not in available_controllers()


@pytest.mark.parametrize("controller", available_controllers())
def test_controller_package_exports_required_classes(controller):
  """get_controller_classes() expects <Name>, <Name>Gains and <Name>Logger."""
  init_file = CONTROL_DIR / controller / "__init__.py"
  tree = ast.parse(init_file.read_text(encoding="utf-8"))
  exported = {
    alias.asname or alias.name
    for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
    for alias in node.names
  }
  for name in (controller, f"{controller}Gains", f"{controller}Logger"):
    assert name in exported, f"{init_file} does not export {name}"
