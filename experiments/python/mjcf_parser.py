#!/usr/bin/env python3
"""
MJCF (MuJoCo XML Format) Parser - Python equivalent of mjcf_parser.hh

Parses MuJoCo MJCF files and extracts geometry information with world-space poses.
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional
from enum import Enum


# =========================
# Math types
# =========================

@dataclass
class Vec3:
    """3D vector."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: 'Vec3') -> 'Vec3':
        """Vector addition."""
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __repr__(self) -> str:
        return f"Vec3({self.x:.6f}, {self.y:.6f}, {self.z:.6f})"


@dataclass
class Quat:
    """Quaternion (w, x, y, z)."""
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __repr__(self) -> str:
        return f"Quat(w={self.w:.6f}, x={self.x:.6f}, y={self.y:.6f}, z={self.z:.6f})"


@dataclass
class Pose:
    """Position and orientation."""
    pos: Vec3 = field(default_factory=Vec3)
    rot: Quat = field(default_factory=Quat)

    def __repr__(self) -> str:
        return f"Pose(pos={self.pos}, rot={self.rot})"


# =========================
# Geometry
# =========================

class GeomType(Enum):
    """Geometry types."""
    BOX = "box"
    SPHERE = "sphere"
    CYLINDER = "cylinder"
    UNKNOWN = "unknown"


@dataclass
class Geom:
    """Geometry object with world-space pose."""
    type: GeomType
    size: Vec3  # box: half-extents, sphere: radius in x, cylinder: (r, h/2)
    world_pose: Pose
    body_name: str = ""
    geom_name: str = ""

    def __repr__(self) -> str:
        return (f"Geom(type={self.type.value}, size={self.size}, "
                f"world_pose={self.world_pose}, body_name={self.body_name}, "
                f"geom_name={self.geom_name})")


# =========================
# Math helpers
# =========================

def quat_mul(a: Quat, b: Quat) -> Quat:
    """Quaternion multiplication (Hamilton product)."""
    return Quat(
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    )


def rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate a vector by a quaternion."""
    p = Quat(w=0, x=v.x, y=v.y, z=v.z)
    qi = Quat(w=q.w, x=-q.x, y=-q.y, z=-q.z)  # conjugate
    r = quat_mul(quat_mul(q, p), qi)
    return Vec3(r.x, r.y, r.z)


# =========================
# Parsing helpers
# =========================

def parse_vec3(s: Optional[str]) -> Vec3:
    """Parse a Vec3 from a space-separated string."""
    if not s:
        return Vec3()
    try:
        parts = s.split()
        return Vec3(float(parts[0]), float(parts[1]), float(parts[2]))
    except (ValueError, IndexError):
        return Vec3()


def parse_quat(s: Optional[str]) -> Quat:
    """Parse a Quat from a space-separated string (w x y z)."""
    if not s:
        return Quat()
    try:
        parts = s.split()
        return Quat(float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
    except (ValueError, IndexError):
        return Quat()


def euler_to_quat(e: Vec3) -> Quat:
    """Convert Euler angles (XYZ, radians) to quaternion."""
    cx = math.cos(e.x * 0.5)
    sx = math.sin(e.x * 0.5)
    cy = math.cos(e.y * 0.5)
    sy = math.sin(e.y * 0.5)
    cz = math.cos(e.z * 0.5)
    sz = math.sin(e.z * 0.5)

    return Quat(
        w=cx*cy*cz + sx*sy*sz,
        x=sx*cy*cz - cx*sy*sz,
        y=cx*sy*cz + sx*cy*sz,
        z=cx*cy*sz - sx*sy*cz
    )


def parse_pose(element: ET.Element) -> Pose:
    """Parse pose (position and orientation) from XML element."""
    pose = Pose()
    
    if element.get("pos"):
        pose.pos = parse_vec3(element.get("pos"))
    
    if element.get("quat"):
        pose.rot = parse_quat(element.get("quat"))
    elif element.get("euler"):
        pose.rot = euler_to_quat(parse_vec3(element.get("euler")))
    # else: identity (default Quat())
    
    return pose


def parse_geom_type(s: Optional[str]) -> GeomType:
    """Parse geometry type from string."""
    if not s:
        return GeomType.UNKNOWN
    try:
        return GeomType(s)
    except ValueError:
        return GeomType.UNKNOWN


# =========================
# Core traversal
# =========================

def parse_body(
    body_element: ET.Element,
    parent_world: Pose,
    out_geoms: List[Geom]
) -> None:
    """
    Recursively parse a body element and extract geometries.
    
    Args:
        body_element: XML element representing a body
        parent_world: Parent body's world pose
        out_geoms: Output list to accumulate geometries
    """
    # Get local pose
    body_local = parse_pose(body_element)
    
    # Compute world pose
    body_world = Pose()
    body_world.rot = quat_mul(parent_world.rot, body_local.rot)
    body_world.pos = parent_world.pos + rotate(parent_world.rot, body_local.pos)
    
    body_name = body_element.get("name", "")
    
    # Parse geometries
    for geom_element in body_element.findall("geom"):
        # Skip visual geometries
        if geom_element.get("class") == "visual":
            continue
        
        geom = Geom(
            type=parse_geom_type(geom_element.get("type")),
            size=parse_vec3(geom_element.get("size")),
            world_pose=Pose()
        )
        
        # Skip unknown geometry types
        if geom.type == GeomType.UNKNOWN:
            continue
        
        # Get local geom pose
        geom_local = parse_pose(geom_element)
        
        # Compute world pose
        geom.world_pose.rot = quat_mul(body_world.rot, geom_local.rot)
        geom.world_pose.pos = body_world.pos + rotate(body_world.rot, geom_local.pos)
        
        geom.geom_name = geom_element.get("name", "")
        geom.body_name = body_name
        
        out_geoms.append(geom)
    
    # Recursively parse child bodies
    for child_body in body_element.findall("body"):
        parse_body(child_body, body_world, out_geoms)


# =========================
# Public API
# =========================

def parse_mjcf(filename: str) -> List[Geom]:
    """
    Parse an MJCF file and extract all geometries with world poses.
    
    Args:
        filename: Path to MJCF XML file
    
    Returns:
        List of Geom objects with world-space poses
    """
    geoms: List[Geom] = []
    
    try:
        tree = ET.parse(filename)
    except ET.ParseError as e:
        print(f"Failed to parse MJCF file: {e}")
        return geoms
    except FileNotFoundError:
        print(f"File not found: {filename}")
        return geoms
    
    root = tree.getroot()
    
    # Handle both <mujoco> and <mujocoinclude> root elements
    if root.tag not in ("mujoco", "mujocoinclude"):
        print("No <mujoco> or <mujocoinclude> root found")
        return geoms
    
    # Find worldbody
    worldbody = root.find("worldbody")
    if worldbody is None:
        print("No <worldbody> found")
        return geoms
    
    # Start from identity pose
    world_pose = Pose()
    
    # Parse all top-level bodies
    for body in worldbody.findall("body"):
        parse_body(body, world_pose, geoms)
    
    return geoms


# =========================
# Utility functions
# =========================

def print_geoms(geoms: List[Geom]) -> None:
    """Print geometry information in a readable format."""
    print(f"Found {len(geoms)} geometries:\n")
    for i, geom in enumerate(geoms):
        print(f"Geom {i}:")
        print(f"  Type: {geom.type.value}")
        print(f"  Body: {geom.body_name}")
        print(f"  Name: {geom.geom_name}")
        print(f"  Size: {geom.size}")
        print(f"  World Position: ({geom.world_pose.pos.x:.4f}, {geom.world_pose.pos.y:.4f}, {geom.world_pose.pos.z:.4f})")
        print(f"  World Rotation (Quat): {geom.world_pose.rot}")
        print()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python mjcf_parser.py <mjcf_file>")
        sys.exit(1)
    
    filename = sys.argv[1]
    geoms = parse_mjcf(filename)
    print_geoms(geoms)
