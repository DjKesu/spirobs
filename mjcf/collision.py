# uses coacd to generate convex hulls from stl files

import os
import trimesh
import coacd

# read in each asset from 60 degrees and create a folder with the convex hull collision meshes
def create_collision_meshes():
    for asset in os.listdir("60 degrees"):
        if asset.endswith(".stl"):
            base_name, _ = os.path.splitext(asset)
            mesh = trimesh.load(os.path.join("60 degrees", asset))
            mesh = coacd.Mesh(mesh.vertices, mesh.faces)
            parts = coacd.run_coacd(mesh)
            collision_dir = os.path.join("60 degrees", f"{base_name}_collision")
            os.makedirs(collision_dir, exist_ok=True)
            for i, part in enumerate(parts):
                # support both coacd.Mesh objects and [vertices, faces] lists
                if hasattr(part, 'vertices') and hasattr(part, 'faces'):
                    vertices = part.vertices
                    faces = part.faces
                else:
                    vertices, faces = part
                part_mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
                output_path = os.path.join(collision_dir, f"{base_name}_part_{i}.stl")
                part_mesh.export(output_path)

create_collision_meshes()
