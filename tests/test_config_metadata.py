"""Packaging/dependency guardrails independent from a ROS installation."""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET
from humanoid_description.robot_params import load_robot_params
from humanoid_motion_controller.command_model import CommandComposer

ROOT = Path(__file__).resolve().parents[1]


def test_command_dimensions_match_canonical_model_yaml():
    p = load_robot_params()
    composer = CommandComposer()
    assert composer.kinematics.l1 == p.thigh_length_m
    assert composer.kinematics.l2 == p.shank_length_m
    assert abs((composer.base_height_m - composer.sole_offset_m) + p.standing_foot_z_m) < 1e-10


def test_ros_packages_have_metadata_and_build_files():
    packages = sorted((ROOT / 'src').glob('*/package.xml'))
    assert len(packages) == 12
    for xml_file in packages:
        x = ET.parse(xml_file).getroot()
        package_dir = xml_file.parent
        assert (package_dir / 'setup.py').exists()
        assert x.findtext('maintainer') and x.findtext('license') == 'MIT'
        assert 'you@example.com' not in xml_file.read_text()
        assert 'ARTEMIS' not in xml_file.read_text()


def test_launch_python_files_parse_without_ros():
    for p in (ROOT / 'src/humanoid_bringup/launch').glob('*.py'):
        ast.parse(p.read_text())


def test_ros_smoke_and_publication_scripts_exist():
    assert (ROOT / 'scripts/ros_smoke.sh').is_file()
    assert (ROOT / 'scripts/publish_github.sh').is_file()
