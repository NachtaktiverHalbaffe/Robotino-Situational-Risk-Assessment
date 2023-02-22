# reference:    https://docs.opencv.org/4.5.2/d4/dc6/tutorial_py_template_matching.html
import math
import os
import cv2 as cv
import numpy as np
from numpy import random
from scipy import ndimage
import copy
import time
from PIL import Image, ImageDraw, ImageOps

PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))


class Obstacle:
    """
    Obstacles consist of corners and borders connecting those corners -> a border is just a list like for example:\
     [[corner1, corner2], [corner1, corner3], [corner2, corner4], ...]  (corner1 and so on are only tuples with x and y \
    coordinates, not some class-objects)
    """

    def __init__(self, corners, label="generic"):
        # Corners is a list of tuples (coordinates)
        self.corners = corners
        # borders are represented as a list of lists, each containing two points (tuples)
        self.borders = self.find_outer_borders()
        # name of the obstacle
        self.label = label
        # A simulation can be run in risk estimator and there can be stochastically determined how often the object was involved in a collision.
        # This value is then stored in here
        self.collisionPotential = 0

    def __str__(self):
        corners_str = f"corners: {str(self.corners)}"
        borders = f"borders: {str(self.borders)}"
        return f"{self.label}: {corners_str}\n{borders}"

    __repr__ = __str__  # X kind of a bad practice

    def distance_to_point(self, pos):  # -> pass position as tuple(x,y)
        """
        Calculates the shortest distance from the calling object ("self") to the passed point
        -> "shortest distance" as in shortest distance to any of the borders

        Args:
            pos ( (x (int), y (int)) ): point or position we want to calculate to distance to

        Returns:
            min_dist (float): The minimal distance to any of the borders of the obstacle
        """
        distances = []
        for border in self.borders:
            pos_x = pos[0]
            pos_y = pos[1]

            # Extract the two points of the border
            x1 = border[0][0]
            x2 = border[1][0]
            y1 = border[0][1]
            y2 = border[1][1]

            # Calculate vector of border
            px = x2 - x1
            py = y2 - y1

            # Calculate anchor point of vector from which the distance will be calculated
            norm = px * px + py * py
            u = ((pos_x - x1) * px + (pos_y - y1) * py) / (float(norm) + 0.00000001)

            #  u should be between 0 and 1
            if u > 1:
                u = 1
            elif u < 0:
                u = 0

            # Calculate end of vector
            x = x1 + u * px
            y = y1 + u * py

            # Calculate vector between the point and the border
            dx = x - pos_x
            dy = y - pos_y

            # Note: If the actual distance does not matter,
            # if you only want to compare what this function
            # returns to other results of this function, you
            # can just return the squared distance instead
            # (i.e. remove the sqrt) to gain a little performance
            dist = (dx * dx + dy * dy) ** 0.5

            # Little correction so prm tends to plan nearer free space/obstacles than workstations
            if "ws" in self.label.lower():
                dist = dist - 2
            else:
                dist = dist + 2
            distances.append(dist)

        if len(distances) != 0:
            min_dist = np.min(np.array(distances))
        else:
            # this is some unfixed error that results from create_random_map (probably the rotation and extraction of corner points).
            # It will occure very rarely and cause the system to only learn from collisions (not from distance to object) until the next call
            # of create_random_map() -> currently every 150 episodes
            min_dist = 5

        return min_dist

    def find_outer_borders(self):
        """
        Checks for the corners tuples of the obstacle that represent the outer borders.
        This function only works for non-concave obstacles

        Returns:
            borders: a list of lists, each containing two corner points (tuples)
        """
        corners = copy.copy(self.corners)
        outer_connections = []
        for corner_a in self.corners:
            corner_a_connections = []
            for corner_b in corners:
                if corner_a is corner_b:
                    continue

                # Calculate all cross products
                crossproducts = []
                for corner_any in self.corners:
                    # Vector 1
                    v1 = (
                        corner_b[0] - corner_a[0],
                        corner_b[1] - corner_a[1],
                    )
                    # Vector 2
                    v2 = (
                        corner_b[0] - corner_any[0],
                        corner_b[1] - corner_any[1],
                    )
                    # Cross product
                    xp = v1[0] * v2[1] - v1[1] * v2[0]

                    if xp > 1e-12:
                        crossproducts.append(1)
                    elif xp < -1e-12:
                        crossproducts.append(-1)
                    else:
                        crossproducts.append(0)

                # vector is border if all cross products are between -1e-12 and 1e-12
                if (1 in crossproducts) and (-1 in crossproducts):
                    pass
                else:
                    corner_a_connections.append([corner_a, corner_b])

            # Add borders to list and remove the checked corner
            corners.remove(corner_a)
            outer_connections.extend(corner_a_connections)

        return outer_connections


def extract_corners(rotated_corner_template):
    """
    To be able to consider the borders of an object (at least how I implemented it-) we need the corners of the object.
    In theory that is easy in template matching, since we exactly know the shape of object -> every object can have a
    "corners.png" file indicating the position of the corners with black pixels (that's how it's actually done here).

    Then: if we rotate the template we want to rotate the corners image also and here is the problem!
    Problem - after rotating the image, single pixels can "disappear" or when increasing the "order" of the rotate(..) \
        function (some parameter) they will grey out.

    So this function aims to find the right pixels that represent the corners of the corresponding obstacle by using the
    rotated corner image

    Args:
        rotated_corner_template: the rotated image with the corners of an obstacle, that has been rotated

    Returns:
        rotated_corner_template: not used
        indexes_final: list of coordinates (tuples) that tell where the corners are after rotating

    !! Note !!
    ---
    This function does not work perfectly all the time, and leads to non perfectly detected borders,
    which shows when using random_maps. -> which might affect training performance a bit but shouldn't actually be a major problem
    """
    # Get indexes and sort them
    indexes = list(np.where(rotated_corner_template < 255))
    sort_indexes = np.argsort(rotated_corner_template[tuple(indexes)])
    indexes_sorted = copy.copy(indexes)
    for i in range(0, len(indexes_sorted)):
        indexes_sorted[i] = indexes_sorted[i][sort_indexes]
    sort_indexes_copy = list(copy.copy(sort_indexes))

    for j in range(0, len(indexes_sorted[0])):
        x_0 = indexes_sorted[0][j]
        y_0 = indexes_sorted[1][j]
        if j < len(indexes_sorted[0]) - 1:
            for k in range(j + 1, len(indexes_sorted[0])):
                x_1 = indexes_sorted[0][k]
                y_1 = indexes_sorted[1][k]
                if (
                    ((x_0 + 1 == x_1) and (y_0 + 1 == y_1))
                    or ((x_0 + 1 == x_1) and (y_0 + 0 == y_1))
                    or ((x_0 + 1 == x_1) and (y_0 - 1 == y_1))
                    or ((x_0 + 0 == x_1) and (y_0 + 1 == y_1))
                    or ((x_0 + 0 == x_1) and (y_0 + 0 == y_1))
                    or ((x_0 + 0 == x_1) and (y_0 - 1 == y_1))
                    or ((x_0 - 1 == x_1) and (y_0 + 1 == y_1))
                    or ((x_0 - 1 == x_1) and (y_0 + 0 == y_1))
                    or ((x_0 - 1 == x_1) and (y_0 - 1 == y_1))
                ):
                    if k in sort_indexes_copy:
                        sort_indexes_copy.remove(k)
    indexes_final = copy.copy(indexes_sorted)
    for i in range(0, len(indexes_final)):
        indexes_final[i] = indexes_final[i][sort_indexes_copy]
    rotated_corner_template[tuple(indexes_final)] = 0

    return rotated_corner_template, indexes_final


def crop_to_ref_map(im):
    """
    Crops the passed scanner map to the minimal needed size.

    !! Important note !!
    ---
    For the real scanner map this funtions hasn't been helpful, since it will cut a
    non-quadratic map which causes problems later on for the NN. Better don't
    use this function and crop the scanner map by hand

    ---
    Args:
        im: scanner map from ros

    Returns:
        cut scanner map
    """
    pixels = np.array(im.getdata()).reshape((im.size[1], im.size[0]))
    img_size = [len(pixels[0]) - 1, len(pixels[1]) - 1]

    y_lowest = len(pixels[1]) - 1
    y_highest = 0
    x_lowest = len(pixels[0]) - 1
    x_highest = 0

    for pixel_y in range(0, len(pixels)):
        for pixel_x in range(0, len(pixels[0])):
            if pixels[pixel_x][pixel_y] != 205 and pixels[pixel_x][pixel_y] != 40:
                if pixel_x < x_lowest:
                    x_lowest = pixel_x
                if pixel_x > x_highest:
                    x_highest = pixel_x
                if pixel_y < y_lowest:
                    y_lowest = pixel_y
                if pixel_y > y_highest:
                    y_highest = pixel_y
            if pixels[pixel_x][pixel_y] == 205:
                # unknown pixels will be colored (205,205,205) by ros gmapping scanner
                pixels[pixel_x][pixel_y] = 40
    if y_lowest > 0:
        y_lowest -= 1
    if x_lowest > 0:
        x_lowest -= 1
    if y_highest < img_size[1]:
        y_highest += 2
    if x_highest < img_size[0]:
        x_highest += 2
    crop_box = (y_lowest, x_lowest, y_highest, x_highest)

    new_map = Image.fromarray(pixels.astype("uint8"), mode="L")

    cropped_im = new_map.crop(crop_box)

    # TODO: cheated for quadratic image size here
    cropped_im = ImageOps.pad(cropped_im, (160, 160), color=0)

    return cropped_im


def generate_map_ref(map_ref):
    """
    Changes (basically inverts) the colors colors of the passed map and generates the "reference map"

    Args:
        map_ref: reference map with "wrong" color layout yet

    Returns:
        grayscale reference map
    """

    pixels = map_ref
    obs = np.array(np.zeros(len(map_ref) * len(map_ref[0]))).reshape(
        len(map_ref), len(map_ref[0])
    )

    for y in range(0, len(pixels)):
        for x in range(0, len(pixels[0])):
            if pixels[y][x] == 0:
                obs[y][x] = 255
            elif pixels[y][x] == 254 or pixels[y][x] == 255:
                obs[y][x] = 0
            elif (
                pixels[y][x] == 40
            ):  # unknown area - for now treated like walls / in PRM it is still set as 40
                obs[y][x] = 255
            elif pixels[y][x] == 205:  # also unknown area (if we didn't crop)
                obs[y][x] = 255

    map_ref = Image.fromarray(obs.astype("uint8"), mode="L")

    return map_ref


# it can still always happen that two tables stand in a way so that it is impossible to know how to place the representation perfectly
def replace_object(
    threshold, map_gray, template, representation, mask, corner_coordinates=None
):
    """
    Applies template matching and wherever the correlation threshold is at maximum and exceeding the threshold, the
    obstacle-representation is put into the map (colored white)

    Args:
        threshold (float): correlation threshold necessary to exceed for putting the representation in the map\
                        -> the higher, the stricter the template matching will be (might not recognize objects then)
        map_gray: scanner map with the footprints of the potential obstacles
        template (image-matrix): current template  to match
        representation (image-matrix) : corresponding representation of the obstacle of the current template
        mask (image-matrix) : corresponding mask of the obstacle of the current template
        corner_coordinates (image-matrix): corresponding corner image-matrix of the obstacle of the current template

    Returns:
        map_grey: map with the obstacles (representations) filled in
        confidence: confidence of how high the correlation of the template matching was
        replaced: true if there were matches
        loc: location at which position the match was
        obstacle: Obstacle-object for the newly detected obstacle
    """

    # map_gray = cv.cvtColor(map_gray, cv.COLOR_BGR2GRAY)
    w, h = template.shape[::-1]
    confidence = []
    max_over_threshold = True
    replaced = False
    loc = (0, 0)
    obstacle = None
    # replace the spots sequentially -> first the maximum correlation ones and then the lesser ones until threshold is no more reached/exceeded
    while max_over_threshold:
        # res = cv.matchTemplate(map_gray, templ=template, method=cv.TM_SQDIFF_NORMED, mask=mask)
        # res = cv.matchTemplate(image=cv.bitwise_not(map_gray), templ=cv.bitwise_not(template), method=cv.TM_CCORR_NORMED, mask=mask)     # "negation" of colors is needed when working with TM_CCORR_NORMED
        res = cv.matchTemplate(
            image=map_gray, templ=template, method=cv.TM_CCOEFF_NORMED, mask=mask
        )  # cv.TM_CCOEFF_NORMED, cv.TM_CCORR_NORMED, cv.TM_SQDIFF_NORMED
        # cv.imshow('res', res)
        # cv.waitKey(0)
        _minVal, _maxVal, minLoc, maxLoc = cv.minMaxLoc(res, None)
        val = _maxVal
        if val == math.inf:
            val = 0
        loc = maxLoc
        confidence.append(np.abs(val - threshold))
        if val >= threshold:
            replaced = True
            for x in range(0, w):
                for y in range(0, h):
                    if template[y, x] != 255 and representation[y, x] != 255:
                        map_gray[loc[1] + y, loc[0] + x] = representation[y, x]
            if corner_coordinates:
                corners = []
                for i in range(0, len(corner_coordinates[0])):
                    # corners.append((loc[1] + corner_coordinates[0][i], loc[0] + corner_coordinates[1][i]))
                    corners.append(
                        (
                            loc[0] + corner_coordinates[1][i],
                            loc[1] + corner_coordinates[0][i],
                        )
                    )
                obstacle = Obstacle(corners)
                # for corner in corners:
                #   map_gray[corner] = 100
        else:
            max_over_threshold = False
    # map_replaced_rgb = cv.cvtColor(map_gray, cv.COLOR_GRAY2RGB)

    return map_gray, confidence, replaced, loc, obstacle


def apply_object_detection(map_path):
    """
    Applies the whole template matching process

    !! Important note !!
    ---
    The directory has to be the following in the project folder:
            /templates/scanner/*TEMPLATES_HERE* \\
            /representations/scanner/*REPRESENTATIONS_HERE* \\
            /masks/*MASKS_HERE* \\
            /representations/scanner/*CORNERS_HERE* 
        
    ALSO: thresholds parameter in this function is important and can be adjusted for all \
    the considered obstacles. If object detection doesn't work properly this can help

    ---
    Args:
        map_path: path to the scanner map to do the template matching on

    Returns:
        map_ref: reference map (with the detected obstacles) after applying object detection
        obstacles: list of the found Obstacles-objects
    """
    map_grey = Image.open(map_path)
    map_grey = map_grey.convert("L")
    map_grey.save(f"{PATH}/maps/map_cropped.png")
    # map_grey = Image.open(map_path)
    map_grey = (
        np.array(map_grey.getdata())
        .reshape((map_grey.size[1], map_grey.size[0]))
        .astype("uint8")
    )

    # handcrafted map:
    # all of these lists need to have the right order of elements (same order)

    # templates = [cv.imread('./templates/test/test_fat/mask_longtable.png', 0),
    #              cv.imread('./templates/test/test_fat/mask_trapez.png', 0),
    #              cv.imread('./templates/test/test_fat/mask_table.png', 0)]
    # representations = [cv.imread('./representations/fat/representation_longtable.png', 0),
    #                    cv.imread('./representations/fat/representation_trapez.png', 0),
    #                    cv.imread('./representations/fat/representation_table.png', 0)]
    # masks = [cv.imread('./masks/test/mask_longtable.png', 0), cv.imread('./masks/test/mask_trapez.png', 0), cv.imread('./masks/test/mask_table.png', 0)]
    # corner_points = [cv.imread('./representations/fat/cornerpoints_longtable.png', 0),
    #                  cv.imread('./representations/fat/cornerpoints_trapez.png', 0),
    #                  cv.imread('./representations/fat/cornerpoints_table.png', 0)]
    # thresholds = [0.7, 0.7, 0.7]

    # scanner map:
    # these files only work with maps built in gmapping with delta=0.05
    templates = [
        cv.imread(f"{PATH}/templates/scanner/template_H_table.png", 0),
        cv.imread(f"{PATH}/templates/scanner/template_trapezoid_table4.png", 0),
    ]
    representations = [
        cv.imread(f"{PATH}/representations/scanner/representation_H_table.png", 0),
        cv.imread(
            f"{PATH}/representations/scanner/representation_trapezoid_table4.png", 0
        ),
    ]
    masks = [
        cv.imread(f"{PATH}/masks/mask_H_table.png", 0),
        cv.imread(f"{PATH}/masks/mask_trapezoid_table4.png", 0),
    ]
    corner_points = [
        cv.imread(f"{PATH}/representations/scanner/corner_points_H.png", 0),
        cv.imread(f"{PATH}/representations/scanner/corner_points_trapezoid.png", 0),
    ]
    thresholds = [0.70, 0.49]  # 0.54

    obstacles = []
    a = 1
    confidence = []
    conf_sum = 0
    # one thing that could be improved here is that it replaces the objects not in a fix order but the ones with the highest confidence first
    # -> maybe e.g. a trapez will somewhat fit on a rect table and mess up the map before rect spots will be checked
    # also this should probably wrapped into a function
    time_0 = time.perf_counter()
    for i in range(0, len(templates)):
        highest_conf = 0
        highest_conf_angle = 0
        angles_over_threshold = {}
        for angle in range(0, int(360 / a)):  # 4    #360
            angle = a * angle
            template = ndimage.rotate(templates[i], angle=angle, cval=255, order=5)
            representation = ndimage.rotate(
                representations[i], angle=angle, cval=255, order=5
            )
            mask = ndimage.rotate(masks[i], angle=angle, cval=0, order=5)
            threshold = thresholds[i]
            # h, w = template.shape

            _, conf, replaced, _, _ = replace_object(
                threshold, copy.deepcopy(map_grey), template, representation, mask
            )
            if replaced:
                angles_over_threshold[angle] = conf

        angles_sorted = sorted(
            angles_over_threshold, key=angles_over_threshold.get, reverse=True
        )
        for angle in angles_sorted:
            template = ndimage.rotate(templates[i], angle=angle, cval=255, order=0)
            representation = ndimage.rotate(
                representations[i], angle=angle, cval=255, order=0
            )
            mask = ndimage.rotate(masks[i], angle=angle, cval=0, order=0)
            corner_points_i = ndimage.rotate(
                corner_points[i], angle=angle, cval=255, order=1
            )  # XXX this order=1 sometimes causes trouble -> corners can "go missing" in rotation process... >1 => less trouble but less nice borders
            # cv.imshow('sooso', corner_points_i)
            # cv.waitKey(0)
            threshold = thresholds[i]
            _, corner_coordinates = extract_corners(corner_points_i)
            map_grey, conf, replaced, loc, obstacle = replace_object(
                threshold,
                map_grey,
                template,
                representation,
                mask,
                corner_coordinates=corner_coordinates,
            )
            confidence.extend(conf)
            if obstacle:
                obstacles.append(obstacle)

            if replaced:
                pass

    # these are only for the handcrafted map to represent the walls at the top and bottom
    # obstacles.append(Obstacle([(0, 16), (159, 16)]))
    # obstacles.append(Obstacle([(0, 143), (159, 143)]))

    avg_confidence = 0
    min_conf = 1
    for conf in confidence:
        conf_sum += conf
        if conf < min_conf:
            min_conf = conf
    if len(confidence) > 0:
        avg_confidence = conf_sum / len(confidence)

    cv.imwrite(f"{PATH}/obj_det_output/res.png", map_grey)
    map_ref = generate_map_ref(map_grey)
    map_ref.save(f"{PATH}/obj_det_output/map_ref_test_obj.png")
    return map_ref, obstacles


def create_random_map():
    """
    This function was implemented to prevent overfitting of the agents to one map by generating random maps and training
    with those.

    !! Important note !!
    ---
    There is a bit of a problem with the corner extraction for random generated obstacles (see function corner_extraction(..)\
    for more information) but might be neglectable
                              
    ALSO: training with random maps (to tackle map overfitting) did not work out successfully yet but there was not time \
    for further investigation.

    ---
    Returns:
        map_ref: Random reference map (PIL Image)
        obstacles: List of random Obstacle-objects
    """
    map_path = "./maps/custom_map_empty.png"

    map_empty = Image.open(map_path)
    map_empty = map_empty.convert("L")
    map_empty = crop_to_ref_map(map_empty)

    map_size = map_empty.size

    map_empty = (
        np.array(map_empty.getdata())
        .reshape((map_size[1], map_size[0]))
        .astype("uint8")
    )
    n_objects = np.random.randint(8, 12)

    positions = []
    while len(positions) == 0:
        for i in range(0, n_objects):
            x = np.random.randint(0, map_size[0])
            y = np.random.randint(0, map_size[1])
            if not (x, y) in positions:
                positions.append((x, y))

    n_objects = len(positions)

    rotations = []
    for i in range(0, n_objects):
        rotation = np.random.randint(0, 360)
        rotations.append(rotation)

    temps = [
        cv.imread(f"{PATH}/templates/test/test_fat/mask_longtable.png", 0),
        cv.imread(f"{PATH}/templates/test/test_fat/mask_trapez.png", 0),
        cv.imread(f"{PATH}/templates/test/test_fat/mask_table.png", 0),
    ]
    reps = [
        cv.imread(f"{PATH}/representations/fat/representation_longtable.png", 0),
        cv.imread(f"{PATH}/representations/fat/representation_trapez.png", 0),
        cv.imread(f"{PATH}/representations/fat/representation_table.png", 0),
    ]
    corners = [
        cv.imread(f"{PATH}/representations/fat/cornerpoints_longtable.png", 0),
        cv.imread(f"{PATH}/representations/fat/cornerpoints_trapez.png", 0),
        cv.imread(f"{PATH}/representations/fat/cornerpoints_table.png", 0),
    ]

    representations = []
    templates = []
    corner_points = []
    for i in range(0, n_objects):
        index = np.random.randint(0, 3)  # three different possible objects
        representations.append(reps[index])
        templates.append(temps[index])
        corner_points.append(corners[index])

    obstacles = []
    for i in range(0, n_objects):
        template = ndimage.rotate(templates[i], angle=rotations[i], cval=255, order=0)
        representation = ndimage.rotate(
            representations[i], angle=rotations[i], cval=255, order=0
        )
        corner_points_i = ndimage.rotate(
            corner_points[i], angle=rotations[i], cval=255, order=2
        )
        _, corner_coordinates = extract_corners(corner_points_i)

        w, h = template.shape[::-1]
        for x in range(0, w):
            for y in range(0, h):
                if (
                    positions[i][1] + y < map_size[1]
                    and positions[i][0] + x < map_size[0]
                ):
                    if template[y, x] != 255 and representation[y, x] != 255:
                        map_empty[
                            positions[i][1] + y, positions[i][0] + x
                        ] = representation[y, x]
        corners = []
        for j in range(0, len(corner_coordinates[0])):
            corners.append(
                (
                    positions[i][0] + corner_coordinates[1][j],
                    positions[i][1] + corner_coordinates[0][j],
                )
            )
        obstacles.append(Obstacle(corners))

    map_ref = generate_map_ref(map_empty)
    map_ref.save("./obj_det_output/random_map.png")

    return map_ref, obstacles


def test_detection():
    """
    Just for debugging...
    """

    # I want to have this function as part of the environment class or so, not here in obj_detection
    def interpolate_segment(segment):
        p1 = segment[0]
        p2 = segment[1]

        interpol_stepsize = 2
        length = np.linalg.norm(np.array(p2) - np.array(p1))

        n_interpol_steps = int(length / interpol_stepsize)

        step_x = (p2[0] - p1[0]) / n_interpol_steps
        step_y = (p2[1] - p1[1]) / n_interpol_steps

        segment_interpolated = []
        for i in range(0, n_interpol_steps + 1):
            segment_interpolated.append(
                (np.round(p1[0] + i * step_x), np.round(p1[1] + i * step_y))
            )

        return segment_interpolated

    # if object detection keeps causing trouble it might be best to increase resolution in gmapping and find a way to scale img down after obj. detection for training (otherwise input obs will be huge)

    map_ref, obstacles = apply_object_detection("./maps/map_y.png")
    # map_ref, obstacles = create_random_map()

    map_ref_draw = ImageDraw.Draw(map_ref)

    for obstacle in obstacles:
        for border in obstacle.borders:
            try:
                map_ref_draw.line([border[0], border[1]], fill=100)
            except:
                pass

    map_ref.save(f"{PATH}/obj_det_output/map_ref_test_obj2.png")


# test_detection()
