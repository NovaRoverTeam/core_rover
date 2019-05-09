import matplotlib.pyplot as plotter
from auto_functions import sectorSearch
from auto_classes import RoveyPosClass, WaypointClass, Vector2D

if __name__ == "__main__":

    vec = Vector2D(-1, 1)
    vec.rotate(90)

    current_pos = RoveyPosClass(0, 0, 0, 0, 0)
    search_radius = 20 # meters

    search_path = sectorSearch(current_pos, search_radius)
    plottable_points = [[], []]
    for point in search_path:
        plottable_points[0].append(point.latitude)
        plottable_points[1].append(point.longitude)

    plotter.plot(plottable_points[0], plottable_points[1])
    plotter.show()