import unittest
import math
from configurator import Gear, GearTrain, compute_pitch_diameter, compute_center_distance, place_second_gear, circles_overlap

class TestGear(unittest.TestCase):
    def test_pitch_diameter(self):
        """Test the pitch diameter calculation."""
        gear = Gear(name="Test", num_teeth=30, module_mm=0.5)
        self.assertAlmostEqual(gear.pitch_diameter(), 15.0)

    def test_radius(self):
        """Test the radius calculation."""
        gear = Gear(name="Test", num_teeth=30, module_mm=0.5)
        self.assertAlmostEqual(gear.radius(), 7.5)

    def test_bounding_circle(self):
        """Test the bounding circle tuple."""
        gear = Gear(name="Test", num_teeth=20, module_mm=1.0, center=(10, 20))
        center, radius = gear.bounding_circle()
        self.assertEqual(center, (10, 20))
        self.assertAlmostEqual(radius, 10.0)

    def test_set_center(self):
        """Test updating the gear's center."""
        gear = Gear(name="Test", num_teeth=10, module_mm=1.0)
        self.assertEqual(gear.center, (0.0, 0.0))
        gear.set_center(5.5, -3.2)
        self.assertEqual(gear.center, (5.5, -3.2))

class TestGeometryHelpers(unittest.TestCase):
    def setUp(self):
        self.gear1 = Gear(name="G1", num_teeth=20, module_mm=0.5, center=(0, 0))
        self.gear2 = Gear(name="G2", num_teeth=40, module_mm=0.5)

    def test_compute_pitch_diameter(self):
        """Test standalone pitch diameter calculation."""
        self.assertAlmostEqual(compute_pitch_diameter(num_teeth=20, module_mm=0.5), 10.0)
        self.assertAlmostEqual(compute_pitch_diameter(num_teeth=40, module_mm=0.5), 20.0)

    def test_compute_center_distance(self):
        """Test the distance calculation between two meshing gears."""
        # (20*0.5)/2 + (40*0.5)/2 = 5 + 10 = 15
        self.assertAlmostEqual(compute_center_distance(self.gear1, self.gear2), 15.0)

    def test_place_second_gear(self):
        """Test the placement of a second gear relative to a first."""
        # Test placement at 0 degrees
        x, y = place_second_gear(self.gear1, self.gear2, 0.0)
        self.assertAlmostEqual(x, 15.0)
        self.assertAlmostEqual(y, 0.0)

        # Test placement at 90 degrees
        x, y = place_second_gear(self.gear1, self.gear2, 90.0)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 15.0)

        # Test placement from a non-origin gear
        self.gear1.set_center(10, 10)
        x, y = place_second_gear(self.gear1, self.gear2, 180.0)
        self.assertAlmostEqual(x, -5.0) # 10 - 15
        self.assertAlmostEqual(y, 10.0)

class TestCollisionDetection(unittest.TestCase):
    def test_circles_overlap(self):
        """Test when circles are clearly overlapping."""
        self.assertTrue(circles_overlap(center1=(0, 0), radius1=10, center2=(10, 0), radius2=10))

    def test_circles_do_not_overlap(self):
        """Test when circles are clearly separate."""
        self.assertFalse(circles_overlap(center1=(0, 0), radius1=5, center2=(15, 0), radius2=5))

    def test_circles_touching(self):
        """Test when circles are just touching (should not be considered an overlap)."""
        self.assertFalse(circles_overlap(center1=(0, 0), radius1=10, center2=(20, 0), radius2=10))

    def test_circles_with_clearance(self):
        """Test overlap detection with a clearance value."""
        # No overlap without clearance
        self.assertFalse(circles_overlap(center1=(0, 0), radius1=5, center2=(11, 0), radius2=5, clearance=0.0))
        # Overlap with clearance
        self.assertTrue(circles_overlap(center1=(0, 0), radius1=5, center2=(11, 0), radius2=5, clearance=2.0))

class TestGearTrain(unittest.TestCase):
    def setUp(self):
        self.train = GearTrain()
        self.g1 = Gear(name="G1", num_teeth=20, module_mm=1.0) # r=10
        self.g2 = Gear(name="G2", num_teeth=30, module_mm=1.0) # r=15
        self.g3 = Gear(name="G3", num_teeth=40, module_mm=1.0) # r=20

    def test_add_gear(self):
        """Test adding a gear to the train."""
        self.assertEqual(len(self.train.gears), 0)
        self.train.add_gear(self.g1)
        self.assertEqual(len(self.train.gears), 1)
        self.assertEqual(self.train.gears[0], self.g1)

    def test_compute_centers_linear_chain(self):
        """Test the linear placement of gears in the train."""
        self.train.add_gear(self.g1)
        self.train.add_gear(self.g2)
        self.train.add_gear(self.g3)
        
        # Default increment angle is 180 degrees (straight line)
        self.train.compute_centers_linear_chain(start_pos=(0, 0), start_angle_deg=0)

        # G1 at origin
        self.assertEqual(self.train.gears[0].center, (0, 0))

        # G2 placed relative to G1 at 0 deg
        # center_dist(g1,g2) = (10+15) = 25
        g2_center = self.train.gears[1].center
        self.assertAlmostEqual(g2_center[0], 25.0)
        self.assertAlmostEqual(g2_center[1], 0.0)

        # G3 placed relative to G2 at 0 + 180 = 180 deg
        # center_dist(g2,g3) = (15+20) = 35
        # G3.x = G2.x + 35 * cos(180) = 25 - 35 = -10
        # G3.y = G2.y + 35 * sin(180) = 0 + 0 = 0
        g3_center = self.train.gears[2].center
        self.assertAlmostEqual(g3_center[0], -10.0)
        self.assertAlmostEqual(g3_center[1], 0.0)

    def test_check_collisions_adjacent_only(self):
        """Test that only adjacent gear collisions are checked."""
        # G1 and G3 will overlap, but are not adjacent.
        # G1: r=10 at (0,0)
        # G2: r=15 at (25,0)
        # G3: r=20 at (-10,0)
        # G1 and G3 overlap because dist(0, -10) = 10, which is < r1+r3 (10+20=30)
        self.train.add_gear(self.g1)
        self.train.add_gear(self.g2)
        self.train.add_gear(self.g3)
        self.train.compute_centers_linear_chain(start_pos=(0, 0), start_angle_deg=0)

        # The current implementation only checks adjacent pairs (g1,g2) and (g2,g3).
        # Neither of these pairs overlap by definition of placement.
        collisions = self.train.check_collisions()
        self.assertEqual(len(collisions), 0, "Should not report non-adjacent collisions")

    def test_check_collisions_with_overlap(self):
        """Test collision detection for adjacent gears with clearance."""
        g1 = Gear(name="G1", num_teeth=20, module_mm=1.0) # r=10
        g2 = Gear(name="G2", num_teeth=20, module_mm=1.0) # r=10
        train = GearTrain()
        train.add_gear(g1)
        train.add_gear(g2)
        
        # Place g2 too close to g1
        g1.set_center(0,0)
        g2.set_center(19, 0) # Center distance is 19, but should be 20

        collisions = train.check_collisions(clearance_mm=0.0)
        self.assertEqual(len(collisions), 1)
        self.assertEqual(collisions[0], (g1, g2))

if __name__ == '__main__':
    unittest.main()
