import re
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def parse_wrl(filename):
    """Parse a VRML .wrl file for Coordinate and coordIndex arrays."""
    with open(filename, "r") as f:
        data = f.read()

    # Remove comments starting with #
    data = re.sub(r"#.*", "", data)

    # Extract points
    point_block = re.search(r"point\s*\[([^\]]+)\]", data, re.MULTILINE | re.DOTALL)
    points = []
    if point_block:
        coords = point_block.group(1).strip().split(",")
        for c in coords:
            nums = c.strip().split()
            if len(nums) == 3:
                points.append([float(n) for n in nums])

    # Extract coordIndex
    index_block = re.search(r"coordIndex\s*\[([^\]]+)\]", data, re.MULTILINE | re.DOTALL)
    faces = []
    if index_block:
        raw = index_block.group(1).replace("\n", " ").split(",")
        face = []
        for val in raw:
            val = val.strip()
            if val == "":
                continue
            if val == "-1":
                if face:
                    faces.append(face)
                    face = []
            else:
                face.append(int(val))

    return np.array(points), faces

def compute_normal(face, points):
    """Compute normal vector for a face using first three vertices."""
    if len(face) < 3:
        return np.array([0,0,0])
    v0, v1, v2 = [points[i] for i in face[:3]]
    edge1 = v1 - v0
    edge2 = v2 - v0
    normal = np.cross(edge1, edge2)
    norm = np.linalg.norm(normal)
    return normal / norm if norm != 0 else np.array([0,0,0])

def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale so the model isn't distorted."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def render_model(points, faces):
    """Render the model with matplotlib and show numbered normals."""
    fig = plt.figure(figsize=(10,8))
    ax = fig.add_subplot(111, projection='3d')

    # Draw faces
    poly3d = [[points[idx] for idx in face] for face in faces]
    collection = Poly3DCollection(poly3d, alpha=0.5,
                                  facecolor=(0.8,0.8,0.9), edgecolor='k')
    ax.add_collection3d(collection)

    # Draw normals with labels
    for i, face in enumerate(faces):
        normal = compute_normal(face, points)
        centroid = np.mean([points[j] for j in face], axis=0)
        ax.quiver(centroid[0], centroid[1], centroid[2],
                  normal[0], normal[1], normal[2],
                  length=15, color='r')
        ax.text(centroid[0]+normal[0]*2,
                centroid[1]+normal[1]*2,
                centroid[2]+normal[2]*2,
                str(i), color='blue')

    # Number the corners (vertices)
    for i, p in enumerate(points):
        ax.text(p[0], p[1], p[2], str(i), color='green')

    # Equal scaling
    set_axes_equal(ax)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("VRML Model with Numbered Face Normals (Equal Axis Scaling)")

    plt.show()


if __name__ == "__main__":
    # Change filename to your .wrl file
    curdir = os.path.abspath(__file__)
    filename = os.path.join(curdir, r"..\WireModelDisplay\data\vrml\@testcube.wrl")
    points, faces = parse_wrl(filename)

    print(f"Loaded {len(points)} points and {len(faces)} faces.")

    # Print normals
    for i, face in enumerate(faces):
        normal = compute_normal(face, points)
        print(f"Face {i}: normal = {normal}")

    # Render
    render_model(points, faces)
