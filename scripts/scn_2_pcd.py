import xml.etree.ElementTree as ET
import numpy as np
import open3d as o3d
import trimesh
import os

from open3d import io

def parse_scn_file(scn_file):
    tree = ET.parse(scn_file)
    root = tree.getroot()
    objects = []
    
    for static in root.findall(".//static"):
        name = static.get("name")
        obj_type = static.get("type", "")

        transform_node = static.find(".//world_transform")
        material_node = static.find(".//material")
        look_node = static.find(".//look")

        if transform_node is not None:
            xyz = list(map(float, transform_node.get("xyz", "0 0 0").split()))
            rpy = list(map(float, transform_node.get("rpy", "0 0 0").split()))

            material = material_node.get("name") if material_node is not None else None
            uv_scale = float(look_node.get("uv_scale", 1.0)) if look_node is not None else 1.0

            if obj_type == "plane":
                # Define a default plane size (10x10 meters, change as needed)
                size = uv_scale * 20  # Scale the plane based on UV scale
                objects.append({
                    "type": "plane",
                    "name": name,
                    "size": size,
                    "xyz": xyz,
                    "rpy": rpy,
                    "material": material
                })
            else:
                mesh_node = static.find(".//visual/mesh")
                if mesh_node is not None:
                    mesh_file = mesh_node.get("filename")
                    scale = float(mesh_node.get("scale", 1.0))
                    objects.append({
                        "type": "mesh",
                        "name": name,
                        "mesh_file": mesh_file,
                        "scale": scale,
                        "xyz": xyz,
                        "rpy": rpy
                    })
    
    return objects

def load_mesh_to_pointcloud(mesh_file, scale, xyz, rpy):
    if not os.path.exists(mesh_file):
        print(f"Warning: {mesh_file} not found.")
        return None
    
    mesh = trimesh.load(mesh_file)
    mesh.apply_scale(scale)
    points = np.array(mesh.sample(10000))  # Sample points from the surface
    
    rotation_matrix = trimesh.transformations.euler_matrix(*rpy, axes='sxyz')[:3, :3]
    rotated_points = (rotation_matrix @ points.T).T
    translated_points = rotated_points + np.array(xyz)
    
    return translated_points

def generate_plane_pointcloud(size, xyz, rpy):
    """Generates a point cloud for a flat plane centered at xyz."""
    grid_x, grid_y = np.meshgrid(
        np.linspace(-size / 2, size / 2, 100), 
        np.linspace(-size / 2, size / 2, 100)
    )
    z = np.zeros_like(grid_x)
    points = np.vstack((grid_x.flatten(), grid_y.flatten(), z.flatten())).T
    
    rotation_matrix = trimesh.transformations.euler_matrix(*rpy, axes='sxyz')[:3, :3]
    rotated_points = (rotation_matrix @ points.T).T
    translated_points = rotated_points + np.array(xyz)
    
    return translated_points

def generate_pointcloud(scn_file):
    objects = parse_scn_file(scn_file)
    all_points = []
    
    for obj in objects:
        if obj["type"] == "mesh":
            points = load_mesh_to_pointcloud(obj["mesh_file"], obj["scale"], obj["xyz"], obj["rpy"])
        elif obj["type"] == "plane":
            points = generate_plane_pointcloud(obj["size"], obj["xyz"], obj["rpy"])
        else:
            points = None
        
        if points is not None:
            all_points.append(points)
    
    if all_points:
        final_points = np.vstack(all_points)
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(final_points)
        return cloud
    else:
        print("No valid points found.")
        return None

if __name__ == "__main__":
    scn_file = "sand.scn"
    pointcloud:o3d.geometry.PointCloud = generate_pointcloud(scn_file)
    if pointcloud is not None:
        o3d.io.write_point_cloud(scn_file + ".ply", pointcloud)

        pointcloud.estimate_normals()

        print(pointcloud.normals[0])

        print("Point cloud saved")
