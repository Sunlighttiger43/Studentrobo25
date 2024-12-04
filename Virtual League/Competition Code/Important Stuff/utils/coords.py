from math import pi
import pickle

if __name__ == "__main__":
    resolution = 100
    map_coords = [[(0, 0) for i in range(resolution)] for i in range(resolution)]

    max_y, max_x = 5.750, 5.750
    step_x, step_y = 5.750 / resolution, 5.750 / resolution
    current_x, current_y = 0, 0
    centre_distance = 0.300
    centre_distance_step = 0.300 / resolution
    centre_point = resolution // 2
    centre_markers = (centre_point - centre_distance_step, centre_point + centre_distance_step)

    coords_set = set()

    for i in map_coords:
        current_x = 0
        for x in range(len(i)):
            # if x in range(centre_markers[0], centre_markers[1]):
            #     i.pop(x)
            # else:
            i[x] = (round(current_x, 4), round(current_y, 4))
            current_x += step_x
            coords_set.add((round(current_x, 4), round(current_y, 4)))
        current_y += step_y

    map_coords = map_coords

    # matrix = [[0 for i in range(resolution**2)] for i in range(resolution**2)]
    # # print(matrix)

    # # Check if 2 coordinates are next to each other
    # def adjacent(coord1, coord2):
    #     return (
    #         coord1[0] < coord2[0] + step_x * 1.1
    #         and coord1[0] > coord2[0] - step_x * 1.1
    #         and coord1[1] < coord2[1] + step_y * 1.1
    #         and coord1[1] > coord2[1] - step_y * 1.1
    #     )

    # # Check if a coordinate is in the centre
    # def in_centre(coord1):
    #     return (
    #         coord1[0] < max_x / 2 + centre_distance
    #         and coord1[0] > max_x / 2 - centre_distance * 1.1
    #         and coord1[1] < max_y + centre_distance
    #         and coord1[1] > max_y - centre_distance
    #     )

    # # For each pair of coordinates
    # for row in range(len(matrix)):
    #     for col in range(len(matrix[row])):
    #         coord1 = map_coords[row // resolution][row % resolution]
    #         coord2 = map_coords[col // resolution][col % resolution]
    #         # Only indicate edge exists if coordinates are next to each other and does connect to centre
    #         matrix[row][col] = (
    #             1
    #             if adjacent(coord1, coord2) and not in_centre(coord1) and not in_centre(coord2)
    #             else 0
    #         )

    with open("maps/map.pkl", "wb") as fh:
        pickle.dump(map_coords, fh)

    with open("maps/matrix.pkl", "wb") as fh:
        pickle.dump(coords_set, fh)
