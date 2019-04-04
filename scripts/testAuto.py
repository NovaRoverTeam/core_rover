import unittest
from auto import angleBetween


'''
Expected values were calculated using https://www.igismap.com/map-tool/bearing-angle. Note that
this tool switches to negative values for any true bearing over 180deg, so in that case we add
360 to its value.
Some values may be slightly different to our intuition because of the curvature of the earth. 
This is why the expected values below are slightly off from the value given in the test name.
'''

class testStartAtOrigin(unittest.TestCase):
    def setUp(self):
        self.latA = 0
        self.lngA = 0

    def test90Deg(self):
        # This test is conducted along the equator line (lat=0), so earth curvature doesn't matter.
        latB = 0
        lngB = 10
        expected = 90
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertEqual(actual, expected)
    
    def test45Deg(self):
        latB = 10
        lngB = 10
        expected = 44.561
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3) # Should be accurate to 3 decimal places

    def test135Deg(self):
        latB = -10
        lngB = 10
        expected = 135.439
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3) 

    def test225Deg(self):
        latB = -10
        lngB = -10
        expected = 224.561
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3) 

    def test315Deg(self):
        latB = 10
        lngB = -10
        expected = 315.439
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3)


class testStartAtWorkshop(unittest.TestCase):
    def setUp(self):
        self.latA = -37.90893
        self.lngA = 145.13414
    
    def test90Deg(self):
        latB = self.latA + 0
        lngB = self.lngA + 10
        expected = 93.077
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3)
    
    def test45Deg(self):
        latB = self.latA + 10
        lngB = self.lngA + 10
        expected = 42.854
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3)

    def test135Deg(self):
        latB = self.latA -10
        lngB = self.lngA + 10
        expected = 147.097
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3) 

    def test225Deg(self):
        latB = self.latA -10
        lngB = self.lngA - 10
        expected = 212.903
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3) 

    def test315Deg(self):
        latB = self.latA + 10
        lngB = self.lngA - 10
        expected = 317.146
        actual = angleBetween(self.latA, self.lngA, latB, lngB)
        self.assertAlmostEqual(actual, expected, 3)


if __name__ == "__main__":
    unittest.main()
