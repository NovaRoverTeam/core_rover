import unittest
from auto import angleBetween


class testAngleBetween(unittest.TestCase):
    def test90Deg(self):
        latA = 0
        lngA = 0
        latB = 0
        lngB = 10
        expected = 90
        actual = angleBetween(latA, lngA, latB, lngB)
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
