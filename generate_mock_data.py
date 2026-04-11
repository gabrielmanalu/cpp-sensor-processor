import os
import numpy as np

# ---------------------------------------------------------------------------
# CSV writer
# ---------------------------------------------------------------------------
def save_csv(filename, columns: list, xyz: np.ndarray, rgb: np.ndarray = None):
    """Write point cloud to CSV using numpy for speed.

    xyz  : float32 array (N, 3)
    rgb  : uint8  array (N, 3) or None for XYZ-only files
    """
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    n = len(xyz)
    with open(filename, 'w', newline='') as f:
        f.write(','.join(columns) + '\n')
        if rgb is None:
            np.savetxt(f, xyz, fmt='%.4f', delimiter=',')
        else:
            # Build one (N, 6) array: floats for xyz, ints for rgb
            block = np.empty((n, 6))
            block[:, :3] = xyz
            block[:, 3:]  = rgb.astype(np.float32)
            np.savetxt(f, block, fmt=['%.4f', '%.4f', '%.4f', '%d', '%d', '%d'],
                       delimiter=',')
    print(f"  Saved {n:,} points -> {filename}")


def noise(n, sigma=0.03):
    return np.random.normal(0, sigma, (n, 3))


# ---------------------------------------------------------------------------
# XYZ — Industrial structure (torus knot + pipes + disc + ground)
# ---------------------------------------------------------------------------
def _torus_knot(p, q, R, r_tube, n):
    """Parametric (p,q)-torus knot thickened into a tube surface."""
    t  = np.random.uniform(0, 2 * np.pi, n)
    phi = np.random.uniform(0, 2 * np.pi, n)

    # Centreline
    cx = (R + r_tube * np.cos(q * t)) * np.cos(p * t)
    cy = (R + r_tube * np.cos(q * t)) * np.sin(p * t)
    cz = r_tube * np.sin(q * t)

    # Approximate Frenet frame: tangent -> normal -> binormal
    dt  = 0.001
    t2  = t + dt
    cx2 = (R + r_tube * np.cos(q * t2)) * np.cos(p * t2)
    cy2 = (R + r_tube * np.cos(q * t2)) * np.sin(p * t2)
    cz2 = r_tube * np.sin(q * t2)

    tx, ty, tz = cx2 - cx, cy2 - cy, cz2 - cz
    length = np.sqrt(tx**2 + ty**2 + tz**2) + 1e-9
    tx /= length; ty /= length; tz /= length

    # Normal: rotate tangent 90° around world-Z for a stable frame
    nx = -ty; ny = tx; nz = np.zeros_like(tz)
    ln = np.sqrt(nx**2 + ny**2) + 1e-9
    nx /= ln; ny /= ln

    tube_r = 0.4
    x = cx + tube_r * (np.cos(phi) * nx - np.sin(phi) * tz * ny)
    y = cy + tube_r * (np.cos(phi) * ny + np.sin(phi) * tz * nx)
    z = cz + tube_r * np.sin(phi)
    return np.stack([x, y, z], axis=1)


def _helix_pipe(radius, pitch, height, tube_r, n):
    """A helical pipe."""
    t   = np.random.uniform(0, 2 * np.pi * (height / pitch), n)
    phi = np.random.uniform(0, 2 * np.pi, n)

    cx = radius * np.cos(t)
    cy = radius * np.sin(t)
    cz = (pitch / (2 * np.pi)) * t

    # Radial normal
    nx = np.cos(t); ny = np.sin(t); nz = np.zeros_like(t)

    x = cx + tube_r * (np.cos(phi) * nx)
    y = cy + tube_r * (np.cos(phi) * ny)
    z = cz + tube_r * np.sin(phi)
    return np.stack([x, y, z], axis=1)


def _perforated_disc(R_inner, R_outer, n_holes, hole_r, n):
    """Flat annular disc with circular holes punched through it."""
    # Random polar coords on the annulus
    r   = np.sqrt(np.random.uniform(R_inner**2, R_outer**2, n * 3))
    ang = np.random.uniform(0, 2 * np.pi, n * 3)
    x   = r * np.cos(ang)
    y   = r * np.sin(ang)
    z   = np.random.uniform(-0.1, 0.1, n * 3)

    # Punch holes evenly spaced
    hole_angles = np.linspace(0, 2 * np.pi, n_holes, endpoint=False)
    hole_R      = (R_inner + R_outer) / 2
    mask = np.ones(len(x), dtype=bool)
    for ha in hole_angles:
        hx, hy = hole_R * np.cos(ha), hole_R * np.sin(ha)
        dist   = np.sqrt((x - hx)**2 + (y - hy)**2)
        mask  &= dist > hole_r

    pts = np.stack([x[mask], y[mask], z[mask]], axis=1)
    return pts[:n]


def _ground_plane(xrange, yrange, n):
    """Bumpy ground plane using multiple Gaussian mounds."""
    x = np.random.uniform(*xrange, n)
    y = np.random.uniform(*yrange, n)
    # Several bumps
    z = np.zeros(n)
    bumps = [( 5,  5, 2.0, 4), (-6,  3, 1.5, 3),
             ( 2, -7, 1.0, 5), (-3, -4, 0.8, 6)]
    for bx, by, bh, bw in bumps:
        z += bh * np.exp(-((x - bx)**2 + (y - by)**2) / (2 * bw**2))
    z += np.random.normal(0, 0.05, n)
    return np.stack([x, y, z], axis=1)


def generate_industrial(filename, num_points=500000):
    print("  Generating industrial structure (XYZ)...")
    rng_seed = 42
    np.random.seed(rng_seed)

    alloc = {
        "knot_235":  int(num_points * 0.22),   # (2,3) torus knot
        "knot_357":  int(num_points * 0.18),   # (3,5) torus knot offset
        "helix_a":   int(num_points * 0.12),
        "helix_b":   int(num_points * 0.10),
        "disc":      int(num_points * 0.15),
        "ground":    int(num_points * 0.18),
        "scatter":   int(num_points * 0.05),
    }

    parts = []

    # Knot 1: (2,3) trefoil, centred at origin
    k1 = _torus_knot(2, 3, R=7, r_tube=2, n=alloc["knot_235"])
    k1 += noise(len(k1), 0.02)
    parts.append(k1)

    # Knot 2: (3,5) cinquefoil, offset and scaled down
    k2 = _torus_knot(3, 5, R=4, r_tube=1.2, n=alloc["knot_357"])
    k2 += np.array([20, 0, 0])
    k2 += noise(len(k2), 0.02)
    parts.append(k2)

    # Helix A: large radius, low pitch
    h1 = _helix_pipe(radius=5, pitch=3, height=15, tube_r=0.35, n=alloc["helix_a"])
    h1 += np.array([-18, 0, -5])
    h1 += noise(len(h1), 0.02)
    parts.append(h1)

    # Helix B: smaller, tighter
    h2 = _helix_pipe(radius=2.5, pitch=1.5, height=10, tube_r=0.25, n=alloc["helix_b"])
    h2 += np.array([0, 18, -3])
    h2 += noise(len(h2), 0.02)
    parts.append(h2)

    # Perforated disc centred at (0, -18, 2)
    d1 = _perforated_disc(R_inner=3, R_outer=9, n_holes=8, hole_r=1.2, n=alloc["disc"])
    d1 += np.array([0, -18, 2])
    d1 += noise(len(d1), 0.015)
    parts.append(d1)

    # Ground plane
    gnd = _ground_plane((-25, 25), (-25, 25), alloc["ground"])
    gnd[:, 2] -= 10
    parts.append(gnd)

    # Scattered sensor noise (simulates random reflections)
    sc = np.random.uniform(-25, 25, (alloc["scatter"], 3))
    sc[:, 2] = np.random.uniform(-12, 12, alloc["scatter"])
    parts.append(sc)

    cloud = np.concatenate(parts, axis=0)
    np.random.shuffle(cloud)
    save_csv(filename, ['x', 'y', 'z'], cloud.astype(np.float32))


# ---------------------------------------------------------------------------
# XYZRGB — Urban outdoor scan (buildings, road, trees, vehicles, sky dome)
# ---------------------------------------------------------------------------
def _box_surface(cx, cy, cz, lx, ly, lz, n):
    """Sample points on the surface of an axis-aligned box."""
    # Face areas determine sampling weight
    Axy = lx * ly; Axz = lx * lz; Ayz = ly * lz
    total = 2 * (Axy + Axz + Ayz)
    w = np.array([Axy, Axy, Axz, Axz, Ayz, Ayz]) / total
    counts = np.round(w * n).astype(int)
    counts[-1] = n - counts[:-1].sum()

    def face(u0, u1, v0, v1, fixed_axis, fixed_val, nu, nv):
        u = np.random.uniform(u0, u1, nu * nv).reshape(-1)
        v = np.random.uniform(v0, v1, nu * nv).reshape(-1)
        f = np.full_like(u, fixed_val)
        if fixed_axis == 0: return np.stack([f, u, v], 1)
        if fixed_axis == 1: return np.stack([u, f, v], 1)
        return np.stack([u, v, f], 1)

    half = np.array([lx, ly, lz]) / 2
    faces = [
        face(cy - ly/2, cy + ly/2, cz - lz/2, cz + lz/2, 0, cx - lx/2, 1, counts[0]),
        face(cy - ly/2, cy + ly/2, cz - lz/2, cz + lz/2, 0, cx + lx/2, 1, counts[1]),
        face(cx - lx/2, cx + lx/2, cz - lz/2, cz + lz/2, 1, cy - ly/2, 1, counts[2]),
        face(cx - lx/2, cx + lx/2, cz - lz/2, cz + lz/2, 1, cy + ly/2, 1, counts[3]),
        face(cx - lx/2, cx + lx/2, cy - ly/2, cy + ly/2, 2, cz - lz/2, 1, counts[4]),
        face(cx - lx/2, cx + lx/2, cy - ly/2, cy + ly/2, 2, cz + lz/2, 1, counts[5]),
    ]
    return np.concatenate(faces, axis=0)[:n]


def _cylinder_surface(cx, cy, base_z, radius, height, n):
    """Points on lateral surface + top cap of a cylinder."""
    n_side = int(n * 0.8); n_top = n - n_side
    phi = np.random.uniform(0, 2 * np.pi, n_side)
    z   = np.random.uniform(base_z, base_z + height, n_side)
    side = np.stack([cx + radius * np.cos(phi),
                     cy + radius * np.sin(phi), z], axis=1)

    r2  = np.sqrt(np.random.uniform(0, radius**2, n_top))
    a2  = np.random.uniform(0, 2 * np.pi, n_top)
    top = np.stack([cx + r2 * np.cos(a2),
                    cy + r2 * np.sin(a2),
                    np.full(n_top, base_z + height)], axis=1)
    return np.concatenate([side, top], axis=0)


def _sphere_surface(cx, cy, cz, radius, n):
    phi   = np.random.uniform(0, 2 * np.pi, n)
    theta = np.arccos(np.random.uniform(-1, 1, n))
    x = cx + radius * np.sin(theta) * np.cos(phi)
    y = cy + radius * np.sin(theta) * np.sin(phi)
    z = cz + radius * np.cos(theta)
    return np.stack([x, y, z], axis=1)


def _colorize(pts, base_rgb, jitter=12):
    """Add per-point colour jitter around a base RGB."""
    n = len(pts)
    r = np.clip(base_rgb[0] + np.random.randint(-jitter, jitter+1, n), 0, 255)
    g = np.clip(base_rgb[1] + np.random.randint(-jitter, jitter+1, n), 0, 255)
    b = np.clip(base_rgb[2] + np.random.randint(-jitter, jitter+1, n), 0, 255)
    return np.stack([pts[:,0], pts[:,1], pts[:,2],
                     r.astype(np.uint8), g.astype(np.uint8), b.astype(np.uint8)], axis=1)


def generate_urban(filename, num_points=500000):
    print("  Generating urban outdoor scan (XYZRGB)...")
    np.random.seed(7)

    parts = []
    budget = num_points

    # --- Ground / road ---
    # Asphalt road running along X axis
    n = int(budget * 0.12)
    road_x = np.random.uniform(-40, 40, n)
    road_y = np.random.uniform(-5, 5, n)
    road_z = np.random.normal(0, 0.04, n)
    road   = np.stack([road_x, road_y, road_z], axis=1)
    parts.append(_colorize(road, (50, 50, 55), jitter=8))

    # Lane markings (white dashes)
    n_marks = int(budget * 0.02)
    mx = np.random.uniform(-40, 40, n_marks)
    my = np.random.uniform(-0.15, 0.15, n_marks)
    mz = np.random.normal(0.01, 0.005, n_marks)
    marks = np.stack([mx, my, mz], axis=1)
    # Only keep points that fall within dashed segments
    keep = (np.floor(mx / 3) % 2 == 0)
    marks = marks[keep]
    parts.append(_colorize(marks, (230, 230, 220), jitter=5))

    # Pavement / dirt on both sides
    n = int(budget * 0.12)
    px = np.random.uniform(-40, 40, n)
    py = np.where(np.random.rand(n) > 0.5,
                  np.random.uniform(5, 12, n),
                  np.random.uniform(-12, -5, n))
    pz = np.random.normal(0, 0.05, n)
    pave = np.stack([px, py, pz], axis=1)
    parts.append(_colorize(pave, (120, 100, 80), jitter=15))

    # --- Buildings ---
    buildings = [
        # (cx, cy, cz_base, lx, ly, lz, wall_color)
        ( 15,  20, 0,  8, 10, 20, (180, 175, 168)),
        (-15,  20, 0,  6,  8, 14, (160, 155, 145)),
        ( 25, -22, 0, 10,  8, 18, (200, 195, 185)),
        (-25, -22, 0,  7, 10, 25, (140, 135, 128)),
        (  0,  30, 0, 12,  6, 30, (170, 165, 160)),
        ( 35,  15, 0,  6,  6, 12, (190, 182, 174)),
        (-35, -15, 0,  8,  8, 16, (155, 148, 140)),
        ( 10, -30, 0,  9,  7, 22, (175, 168, 158)),
        (-10,  28, 0,  7,  9, 17, (165, 158, 150)),
        ( 40, -20, 0,  8,  8, 28, (185, 178, 170)),
    ]
    pts_per_building = int(budget * 0.38) // len(buildings)
    for cx, cy, _, lx, ly, lz, col in buildings:
        cz = lz / 2
        surf = _box_surface(cx, cy, cz, lx, ly, lz, pts_per_building)
        surf += noise(len(surf), 0.025)
        colored = _colorize(surf, col, jitter=10)
        parts.append(colored)

        # Windows: darker patches on facade — add window-coloured points
        n_win = pts_per_building // 8
        wx = np.random.uniform(cx - lx/2 + 0.5, cx + lx/2 - 0.5, n_win)
        wy_front = np.full(n_win // 2, cy + ly / 2)
        wy_back  = np.full(n_win - n_win // 2, cy - ly / 2)
        wz = np.random.uniform(1.5, lz - 1.0, n_win)
        win_pts = np.concatenate([
            np.stack([wx[:n_win//2], wy_front, wz[:n_win//2]], 1),
            np.stack([wx[n_win//2:], wy_back,  wz[n_win//2:]], 1),
        ])
        parts.append(_colorize(win_pts, (80, 120, 160), jitter=15))

    # --- Trees ---
    tree_positions = [
        (-8, 8), (8, 8), (-8, -8), (8, -8),
        (-20, 9), (20, 9), (-20, -9), (20, -9),
        (0, -10), (0, 10),
    ]
    pts_per_tree = int(budget * 0.10) // len(tree_positions)
    for tx, ty in tree_positions:
        # Trunk
        n_trunk = pts_per_tree // 5
        trunk = _cylinder_surface(tx, ty, 0, 0.25, 3.0, n_trunk)
        parts.append(_colorize(trunk, (80, 50, 20), jitter=10))

        # Foliage (two overlapping spheres)
        n_foliage = pts_per_tree - n_trunk
        f1 = _sphere_surface(tx, ty, 4.5, 1.8, n_foliage // 2)
        f2 = _sphere_surface(tx + 0.5, ty + 0.3, 5.5, 1.4, n_foliage - n_foliage // 2)
        foliage = np.concatenate([f1, f2])
        parts.append(_colorize(foliage, (40, 130, 40), jitter=25))

    # --- Vehicles (parked cars) ---
    cars = [
        ( 10,  7.5, (180,  30,  30)),   # red
        (-10,  7.5, ( 30,  80, 180)),   # blue
        ( 10, -7.5, (220, 220, 220)),   # white
        (-10, -7.5, ( 60,  60,  60)),   # dark grey
        ( 30,  7.5, (180, 120,  30)),   # orange
        (-30, -7.5, ( 30, 160,  80)),   # green
    ]
    pts_per_car = int(budget * 0.06) // len(cars)
    for vx, vy, col in cars:
        body = _box_surface(vx, vy, 0.75, 4.2, 1.8, 1.5, int(pts_per_car * 0.7))
        roof = _box_surface(vx - 0.3, vy, 1.75, 2.8, 1.6, 0.9, int(pts_per_car * 0.2))
        wheels_pts = []
        for wox, woy in [(-1.2, -1.0), (1.2, -1.0), (-1.2, 1.0), (1.2, 1.0)]:
            w = _cylinder_surface(vx + wox, vy + woy, 0.0, 0.35, 0.2, pts_per_car // 20 + 1)
            wheels_pts.append(w)
        car_pts = np.concatenate([body, roof] + wheels_pts)
        car_pts += noise(len(car_pts), 0.015)
        parts.append(_colorize(car_pts, col, jitter=8))

    # --- Sky dome (simulates LiDAR returns off haze / low clouds) ---
    n_sky = int(budget * 0.06)
    sky_phi   = np.random.uniform(0, 2 * np.pi, n_sky)
    sky_theta = np.random.uniform(0, np.pi / 3, n_sky)   # upper hemisphere only
    sky_r     = np.random.uniform(45, 55, n_sky)
    sx = sky_r * np.sin(sky_theta) * np.cos(sky_phi)
    sy = sky_r * np.sin(sky_theta) * np.sin(sky_phi)
    sz = sky_r * np.cos(sky_theta)
    sky = np.stack([sx, sy, sz], axis=1)
    # Gradient: deeper blue at zenith, lighter near horizon
    t_zen = np.cos(sky_theta)
    sr = np.clip(100 + (1 - t_zen) * 80 + np.random.randint(-10, 10, n_sky), 0, 255)
    sg = np.clip(140 + (1 - t_zen) * 60 + np.random.randint(-10, 10, n_sky), 0, 255)
    sb = np.clip(200 + (1 - t_zen) * 40 + np.random.randint(-10, 10, n_sky), 0, 255)
    sky_c = np.stack([sx, sy, sz,
                      sr.astype(np.uint8), sg.astype(np.uint8), sb.astype(np.uint8)], axis=1)
    parts.append(sky_c)

    # --- Assemble ---
    cloud = np.concatenate(parts, axis=0)

    # Trim or pad to exactly num_points
    if len(cloud) > num_points:
        idx = np.random.choice(len(cloud), num_points, replace=False)
        cloud = cloud[idx]
    elif len(cloud) < num_points:
        extra = num_points - len(cloud)
        idx = np.random.choice(len(cloud), extra, replace=True)
        pad = cloud[idx].copy()
        # Tiny jitter so padded points aren't exact duplicates
        pad[:, :3] += np.random.normal(0, 0.02, (extra, 3))
        cloud = np.concatenate([cloud, pad])
    np.random.shuffle(cloud)

    xyz = cloud[:, :3].astype(np.float32)
    rgb = cloud[:, 3:].astype(np.uint8)
    save_csv(filename, ['x', 'y', 'z', 'r', 'g', 'b'], xyz, rgb)


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    N = 5_000_000
    print(f"Generating high-fidelity sensor mock data ({N:,} points each)...")
    generate_industrial('data/mock_sensor_xyz.csv',   num_points=N)
    generate_urban     ('data/mock_sensor_xyzrgb.csv', num_points=N)
    print("Done!")
