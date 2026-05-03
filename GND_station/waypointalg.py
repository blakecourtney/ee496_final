import math
from collections import deque
from config import NUM_DRONES, DRONE_RADIUS, DRONE_DIAMETER

EARTH_RADIUS = 6371000.0  # meters
PI = math.pi


def deg2rad(deg):
    return deg * PI / 180.0

def rad2deg(rad):
    return rad * 180.0 / PI

def midpoint(a, b):
    return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)


def dist(a, b):
    """Haversine distance (meters)"""
    lat1 = deg2rad(a[0])
    lon1 = deg2rad(a[1])
    lat2 = deg2rad(b[0])
    lon2 = deg2rad(b[1])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    h = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )

    return 2 * EARTH_RADIUS * math.atan2(math.sqrt(h), math.sqrt(1 - h))


def interpolate(a, b, fraction):
    lat1 = deg2rad(a[0])
    lon1 = deg2rad(a[1])
    lat2 = deg2rad(b[0])
    lon2 = deg2rad(b[1])

    d = dist(a, b) / EARTH_RADIUS

    if d == 0:
        return a

    A = math.sin((1 - fraction) * d) / math.sin(d)
    B = math.sin(fraction * d) / math.sin(d)

    x = A * math.cos(lat1) * math.cos(lon1) + B * math.cos(lat2) * math.cos(lon2)
    y = A * math.cos(lat1) * math.sin(lon1) + B * math.cos(lat2) * math.sin(lon2)
    z = A * math.sin(lat1) + B * math.sin(lat2)

    lat = math.atan2(z, math.sqrt(x * x + y * y))
    lon = math.atan2(y, x)

    return (rad2deg(lat), rad2deg(lon))

def split_region(tl, tr, br, bl):
    top_mid = midpoint(tl, tr)
    bottom_mid = midpoint(bl, br)
    left_mid = midpoint(tl, bl)
    right_mid = midpoint(tr, br)

    center = midpoint(top_mid, bottom_mid)

    # 4 subregions
    return [
        (tl, top_mid, center, left_mid),        # top-left
        (top_mid, tr, right_mid, center),       # top-right
        (center, right_mid, br, bottom_mid),    # bottom-right
        (left_mid, center, bottom_mid, bl)      # bottom-left
    ]

def region_too_large(tl, tr, br, bl):
    height = (max(tr[0], br[0]) - min(tl[0], bl[0])) * 111320

    width = (
        (max(tr[1], br[1]) - min(tl[1], bl[1]))
        * 111320
        * math.cos(deg2rad(tr[0] - br[0]))
    )

    return width > DRONE_DIAMETER or height > DRONE_DIAMETER

def subdivide_region(tl, tr, br, bl):
    queue = deque()
    result = []

    queue.append((tl, tr, br, bl))

    while queue:
        region = queue.popleft()
        tl, tr, br, bl = region

        if region_too_large(tl, tr, br, bl):
            subregions = split_region(tl, tr, br, bl)
            queue.extend(subregions)
        else:
            result.append(region)

    return result

# -----------------------------
# BACKBONE GENERATION
# -----------------------------

def create_backbone(home, dest):
    distance = dist(home, dest)

    num_segments = int(distance // DRONE_DIAMETER)
    needed_drones = max(0, num_segments - 1)

    if NUM_DRONES < needed_drones:
        return []

    drone_locations = []

    for i in range(1, needed_drones + 1):
        fraction = i / num_segments
        drone_locations.append(interpolate(home, dest, fraction))

    return drone_locations


# -----------------------------
# MAIN WAYPOINT ALGORITHM
# -----------------------------

def relay_search_drones(tl, tr, br, bl, home):
    """
    tl, tr, br, bl = (lat, lon)
    home = (lat, lon)

    returns:
        relay_points, search_point, update_flag
    """

    height = (max(tr[0], br[0]) - min(tl[0], bl[0])) * 111320

    width = (
        (max(tr[1], br[1]) - min(tl[1], bl[1]))
        * 111320
        * math.cos(deg2rad(tr[0] - br[0]))
    )

    if width > DRONE_DIAMETER or height > DRONE_DIAMETER:
        print("Need to split into regions")

        subregions = subdivide_region(tl, tr, br, bl)

        if not subregions:
            return [], None, 1

        def center_of(region):
            tl, tr, br, bl = region
            return (
                (tl[0] + tr[0] + br[0] + bl[0]) / 4,
                (tl[1] + tr[1] + br[1] + bl[1]) / 4
            )

        # pick closest region to home
        closest_region = min(
            subregions,
            key=lambda r: dist(home, center_of(r))
        )

        tl, tr, br, bl = closest_region

        print("Selected subregion center:", center_of(closest_region))

        # compute center of selected region
        center_lat = (tl[0] + tr[0] + br[0] + bl[0]) / 4
        center_lon = (tl[1] + tr[1] + br[1] + bl[1]) / 4
        center = (center_lat, center_lon)

        relay_points = create_backbone(home, center)

        return relay_points, center, 0

    # compute center
    center_lat = (tl[0] + tr[0] + br[0] + bl[0]) / 4
    center_lon = (tl[1] + tr[1] + br[1] + bl[1]) / 4
    center = (center_lat, center_lon)

    relay_points = create_backbone(home, center)

    if not relay_points:
        print("Not enough drones")
        return [], center, 1

    return relay_points, center, 0