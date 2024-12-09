import pytest
from pathlib import Path
from dexrobot_kinematics.config import KinematicsConfig

def test_config_loading():
    """Test config loading from YAML"""
    config_path = Path("config/test_config.yaml")
    config = KinematicsConfig.from_yaml(config_path)

    assert config.left_hand_urdf.exists()
    assert config.right_hand_urdf.exists()
    assert config.left_hand_urdf != config.right_hand_urdf

def test_config_validation():
    """Test config validation"""
    with pytest.raises(ValueError):
        KinematicsConfig(
            left_hand_urdf=Path("nonexistent.urdf"),
            right_hand_urdf=Path("nonexistent.urdf")
        )
