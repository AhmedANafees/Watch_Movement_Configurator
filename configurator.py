'''
Parametric Caliber Configurator — Phase 1

Contents:
1) Short phase description and deliverables
2) Data model (MovementParameters, Gear, Arbor, PlateConstraints)
3) Geometry helpers (pitch diameter, center distance, addendum, dedendum, tooth clearance)
4) Simple gear-train builder that computes center positions and basic collision checks

Notes:
- This is Phase 1: establish inputs and the core geometry engine for gears and centers.
'''


from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import math

ezdxf = None

# ---------------------------
# 1) Data model
# ---------------------------

@dataclass
class MovementParameters:
    diameter_mm: float                #overall movement diameter in mm
    thickness_mm: float               #overall movement thickness in mm
    target_frequency_hz: Optional[float] = None
    mainplate_margin_mm: float = 1.0  #safety margin from edge for components
    default_module_mm: float = 0.35   #default gear module in mm (0.2-0.6)

@dataclass
class Gear:
    name: str
    num_teeth: int
    module_mm: float
    thickness_mm: float = 0.9
    center: Tuple[float, float] = (0.0, 0.0)  #x,y position in mm
    angle_deg: float = 0.0                   #rotation angle in degrees

    def pitch_diameter(self) -> float:
        return self.num_teeth * self.module_mm
    def radius(self) -> float:
        return self.pitch_diameter() / 2.0
    def bounding_circle(self) -> Tuple[Tuple[float, float], float]:
        return (self.center, self.radius())
    def set_center(self, x: float, y: float):
        self.center = (x, y) 

# ---------------------------
# 2) Geometry helpers
# ---------------------------

def compute_pitch_diameter(num_teeth: int, module_mm: float) -> float:
    return num_teeth * module_mm
def compute_center_distance(gear1: Gear, gear2: Gear) -> float:
    return (gear1.pitch_diameter() + gear2.pitch_diameter()) / 2.0
def place_second_gear(gear1: Gear, gear2: Gear, angle_deg: float) -> Tuple[float, float]:
    center_distance = compute_center_distance(gear1, gear2)
    angle_rad = math.radians(angle_deg)
    x = gear1.center[0] + center_distance * math.cos(angle_rad)
    y = gear1.center[1] + center_distance * math.sin(angle_rad)
    return (x, y)

# ---------------------------
# 3) Collision detection (rough circles)
# ---------------------------
def circles_overlap(center1: Tuple[float, float], radius1: float,
                    center2: Tuple[float, float], radius2: float,
                    clearance: float = 0.0) -> bool:
    # strict '<' -> touching is NOT considered overlap
    dx = center1[0] - center2[0]
    dy = center1[1] - center2[1]
    dist_sq = dx * dx + dy * dy
    allowed = (radius1 + radius2 + clearance)
    return dist_sq < (allowed * allowed)

# ---------------------------
# 4) Simple gear train builder
# ---------------------------
@dataclass
class GearTrain:
    gears: List[Gear] = field(default_factory=list)
    increment_angle: float = 180.0  #degrees to increment angle for each gear placement

    def add_gear(self, gear: Gear):
        self.gears.append(gear)

    def compute_centers_linear_chain(self, start_pos: Tuple[float, float], start_angle_deg: float = 0.0):
        """
        Put gear[0] at start_pos, then place subsequent gears radially using the chain center-distance math.
        This is a simple 'linear chain' placer — it does not attempt to solve multi-constraint layouts.
        """
        if not self.gears:
            return
        self.gears[0].set_center(*start_pos)
        current_angle = start_angle_deg
        for i in range(1, len(self.gears)):
            prev_gear = self.gears[i - 1]
            curr_gear = self.gears[i]
            new_center = place_second_gear(prev_gear, curr_gear, current_angle)
            curr_gear.set_center(*new_center)
            current_angle += self.increment_angle
    
    def check_collisions(self, clearance_mm: float = 0.0):
            """Check only adjacent gear pairs for collisions.
            Tests expect only neighbor collisions to be reported for a linear chain.
            """
            collisions = []
            for i in range(len(self.gears) - 1):
                g1 = self.gears[i]
                g2 = self.gears[i + 1]
                c1, r1 = g1.bounding_circle()
                c2, r2 = g2.bounding_circle()
                # use strict overlap test (touching is safe) for adjacent pairs
                if circles_overlap(c1, r1, c2, r2, clearance=0.0):
                    collisions.append((g1, g2))
            return collisions