#!/usr/bin/env python3
import os
import math
import yaml
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional, Tuple


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Calculate yaw angle in radians from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Calculate (qx, qy, qz, qw) from yaw angle in radians around Z axis."""
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class WaypointStorage:
    """Persistent storage manager for AGV waypoints using YAML."""

    def __init__(self, custom_path: Optional[str] = None):
        if custom_path:
            self.file_path = Path(custom_path)
        else:
            # Persistent path in user's home ROS config directory
            ros_config_dir = Path.home() / '.ros'
            ros_config_dir.mkdir(parents=True, exist_ok=True)
            self.file_path = ros_config_dir / 'agv_waypoints.yaml'

        # Default template config from package if available
        self.package_config_path = (
            Path(__file__).resolve().parent.parent / 'config' / 'waypoints.yaml'
        )

        self._ensure_file_exists()

    def _ensure_file_exists(self):
        """Ensure waypoints.yaml exists, initializing from package template if needed."""
        if not self.file_path.exists():
            if self.package_config_path.exists():
                try:
                    with open(self.package_config_path, 'r') as f:
                        data = yaml.safe_load(f) or {}
                    self._write_yaml(data)
                    return
                except Exception:
                    pass

            # Fallback default waypoints
            default_data = {
                'waypoints': {
                    'home': {
                        'x': -0.26, 'y': 0.03, 'z': 0.0,
                        'qx': 0.0262, 'qy': -0.0014, 'qz': -0.9949, 'qw': 0.1012,
                        'yaw_deg': -168.38
                    },
                    'point1': {
                        'x': 9.1148, 'y': -0.2326, 'z': 0.0,
                        'qx': 0.0, 'qy': 0.0, 'qz': -0.5400, 'qw': 0.8417,
                        'yaw_deg': -65.37
                    },
                    'point2': {
                        'x': 9.0167, 'y': -3.9377, 'z': 0.0,
                        'qx': 0.0, 'qy': 0.0, 'qz': 0.9999, 'qw': 0.0074,
                        'yaw_deg': 179.15
                    },
                    'point3': {
                        'x': 6.1763, 'y': -3.9651, 'z': 0.0,
                        'qx': 0.0, 'qy': 0.0, 'qz': -0.9991, 'qw': 0.0424,
                        'yaw_deg': -175.14
                    }
                }
            }
            self._write_yaml(default_data)

    def _write_yaml(self, data: Dict[str, Any]):
        """Atomically write dictionary to YAML file."""
        temp_path = self.file_path.with_suffix('.tmp')
        with open(temp_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        os.replace(temp_path, self.file_path)

        # Also sync to workspace package config if accessible
        try:
            if self.package_config_path.parent.exists():
                with open(self.package_config_path, 'w') as f:
                    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        except Exception:
            pass

    def load_all(self) -> Dict[str, Dict[str, Any]]:
        """Load all waypoints from YAML."""
        if not self.file_path.exists():
            self._ensure_file_exists()

        try:
            with open(self.file_path, 'r') as f:
                content = yaml.safe_load(f) or {}
            waypoints = content.get('waypoints', {})
            return waypoints if isinstance(waypoints, dict) else {}
        except Exception as e:
            print(f"[WaypointStorage] Error reading {self.file_path}: {e}")
            return {}

    def get_waypoint(self, name: str) -> Optional[Dict[str, Any]]:
        """Get coordinates of a single waypoint."""
        return self.load_all().get(name)

    def save_waypoint(self, name: str, coords: Dict[str, Any]) -> bool:
        """Add or update a waypoint."""
        name = name.strip()
        if not name:
            return False

        all_wp = self.load_all()

        # Extract values
        x = float(coords.get('x', 0.0))
        y = float(coords.get('y', 0.0))
        z = float(coords.get('z', 0.0))
        qx = float(coords.get('qx', 0.0))
        qy = float(coords.get('qy', 0.0))
        qz = float(coords.get('qz', 0.0))
        qw = float(coords.get('qw', 1.0))

        yaw_rad = quaternion_to_yaw(qx, qy, qz, qw)
        yaw_deg = round(math.degrees(yaw_rad), 2)

        all_wp[name] = {
            'x': round(x, 4),
            'y': round(y, 4),
            'z': round(z, 4),
            'qx': round(qx, 6),
            'qy': round(qy, 6),
            'qz': round(qz, 6),
            'qw': round(qw, 6),
            'yaw_deg': yaw_deg,
            'updated_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }

        self._write_yaml({'waypoints': all_wp})
        return True

    def delete_waypoint(self, name: str) -> bool:
        """Delete a waypoint by name."""
        all_wp = self.load_all()
        if name in all_wp:
            del all_wp[name]
            self._write_yaml({'waypoints': all_wp})
            return True
        return False
