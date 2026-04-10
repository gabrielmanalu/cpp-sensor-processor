import csv
import math
import random
import os

def save_csv(filename, columns, data):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        writer.writerows(data)
    print(f"Generated {len(data)} points in {filename}")

def generate_torus(filename, num_points=100000):
    """Generates a 3D Ring (Torus) using parametric equations."""
    data = []
    R = 10.0  # Distance from center of tube to center of torus
    r = 3.0   # Radius of the tube
    
    for _ in range(num_points):
        # Parametric angles
        u = random.uniform(0, 2 * math.pi) # Angle around the ring
        v = random.uniform(0, 2 * math.pi) # Angle around the tube
        
        # Torus equations
        x = (R + r * math.cos(v)) * math.cos(u)
        y = (R + r * math.cos(v)) * math.sin(u)
        z = r * math.sin(v)
        
        # Add slight sensor noise (standard deviation of 0.02)
        x += random.gauss(0, 0.02)
        y += random.gauss(0, 0.02)
        z += random.gauss(0, 0.02)
        
        data.append([round(x, 4), round(y, 4), round(z, 4)])
    
    save_csv(filename, ['x', 'y', 'z'], data)

def generate_chair(filename, num_points=100000):
    """Generates a 3D Chair by sampling points on different box volumes."""
    data = []
    
    def sample_box(x_range, y_range, z_range, color, count):
        for _ in range(count):
            x = random.uniform(*x_range)
            y = random.uniform(*y_range)
            z = random.uniform(*z_range)
            data.append([round(x, 4), round(y, 4), round(z, 4), *color])

    # 1. Seat (Brownish)
    sample_box((-5, 5), (-5, 5), (4, 5), (139, 69, 19), int(num_points * 0.4))
    
    # 2. Backrest (Darker Brown)
    sample_box((-5, 5), (4, 5), (5, 12), (101, 67, 33), int(num_points * 0.3))
    
    # 3. Four Legs (Black/Dark Grey)
    leg_positions = [(-4.5, -3.5), (3.5, 4.5)]
    points_per_leg = int(num_points * 0.3) // 4
    for lx in leg_positions:
        for ly in leg_positions:
            sample_box(lx, ly, (0, 4), (30, 30, 30), points_per_leg)

    save_csv(filename, ['x', 'y', 'z', 'r', 'g', 'b'], data)

if __name__ == "__main__":
    print("Generating high-fidelity sensor mock data...")
    generate_torus('data/mock_sensor_xyz.csv', 100000)
    generate_chair('data/mock_sensor_xyzrgb.csv', 100000)
    print("Done!")