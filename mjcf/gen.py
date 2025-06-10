#!/usr/bin/env python3
"""
Generate MJCF for a two-tendon SpiRob built from the STL directory
structure shown in the screenshot (visual mesh + *_collision/ subdir).
"""
import os, math, uuid, logging, argparse
from pathlib import Path
from lxml import etree as ET
import numpy as np
import trimesh

ASSET_DIR          = "60 degrees"
UNIT_ANGLE_DEG     = 60
LINK_LENGTH_M      = 0.025
K_HINGE            = 25.0        # N·m/rad
C_HINGE            = 0.20        # N·m·s/rad
K_TENDON           = 2_000.0     # N
D_TENDON           = 5.0         # N·s/m
SPOOL_RADIUS_M     = 0.012
SERVO_TORQUE_NM    = 3.5
SERVO_SPEED_RAD_S  = 5.0
RGBA_LINK          = "1.0 0.41 0.7 1"  
MASS_PER_LINK      = 0.03      # kg
I_LINK             = 2e-5      # kg·m²
REELS_MASS         = 0.02      # kg, mass for the reels body
REELS_DIAGINERTIA  = 1e-5      # kg·m², diagonal inertia for the reels body


def rel_path(path: Path, target_xml: Path) -> str:
    return os.path.relpath(path.resolve(), target_xml.parent.resolve()).replace("\\", "/")


def build_spirob(asset_dir: Path, xml_out: Path):
    model_name = f"spirob_{uuid.uuid4().hex[:6]}"
    root   = ET.Element("mujoco", model=model_name)
    ET.SubElement(root, "compiler", angle="degree", meshdir=".", autolimits="true")
    asset  = ET.SubElement(root, "asset")
    world  = ET.SubElement(root, "worldbody")
    defaults = ET.SubElement(root, "default")

    # visual default (non-colliding)
    ET.SubElement(defaults, "default", **{"class":"vis"}).append(
        ET.Element("geom", contype="0", conaffinity="0",
                   rgba=RGBA_LINK, type="mesh", group="1"))
    # collision default
    fric = f"{K_TENDON/4000:.3f} {K_TENDON/4000:.3f}"
    ET.SubElement(defaults, "default", **{"class":"col"}).append(
        ET.Element("geom", contype="1", conaffinity="1",
                   density="1000", friction=fric, rgba="0.5 0.6 0.7 0.5"))

    # ---------------- Assets ---------------- #
    link_info = []           # [ (vis_name, [col_name...], vis_mesh_Path) , ... ]
    for fname in sorted(os.listdir(asset_dir)):
        f = Path(asset_dir)/fname
        if f.suffix.lower() != ".stl": continue
        base = f.stem.replace(" ", "_")
        # visual mesh
        vis_name = f"vis_{base}"
        ET.SubElement(asset, "mesh", name=vis_name,
                      file=rel_path(f, xml_out))
        # collision parts
        col_names = []
        col_dir = f.with_name(f.stem + "_collision")
        if col_dir.is_dir():
            for part in sorted(col_dir.glob("*.stl")):
                col = f"col_{part.stem.replace(' ', '_')}"
                ET.SubElement(asset, "mesh", name=col,
                              file=rel_path(part, xml_out))
                col_names.append(col)
        else:
            col_names.append(vis_name)   # fallback
        link_info.append( (vis_name, col_names, f) ) # Store Path object 'f' for trimesh

    # ---------------- Chain bodies ---------------- #
    parent = world
    z_offset = 0.0 


    for i,(vis_name, col_mesh_names, vis_mesh_path) in enumerate(link_info):
        current_link_stacking_height = LINK_LENGTH_M 
        mesh_z_min = 0.0  
        
        try:
            mesh = trimesh.load_mesh(str(vis_mesh_path))
            # Get the bounding box bounds [min_x, min_y, min_z], [max_x, max_y, max_z]
            bounds = mesh.bounds
            z_min, z_max = bounds[0][2], bounds[1][2]
            
            # The stacking height should be from the bottom of the mesh to the top
            height = z_max - z_min
            
            print(f"Mesh {vis_mesh_path} has height {height:.4f} (raw units)")
            
            if height > 1e-6:
                # Convert with adaptive stacking height
                full_height = height / 5.0
                mesh_z_min = z_min / 5.0
                
                # Use aggressive overlap so edges connect properly
                overlap_distance = 0.008  # 8mm overlap distance
                current_link_stacking_height = full_height - overlap_distance
                
                # Ensure we don't get negative or too small spacing
                min_spacing = 0.005  # 5mm minimum spacing
                current_link_stacking_height = max(current_link_stacking_height, min_spacing)
                print(f"  -> Converted to {current_link_stacking_height:.6f} m")
            else:
                logging.warning(f"Mesh {vis_mesh_path} has non-positive Z-extent ({height:.4f}). Using fallback LINK_LENGTH_M={LINK_LENGTH_M}.")
        except Exception as e:
            logging.warning(f"Could not load mesh {vis_mesh_path} or get extents: {e}. Using fallback LINK_LENGTH_M={LINK_LENGTH_M}.")

        if i == 0:
            body_z_pos = -mesh_z_min  # Position body so mesh bottom is at z=0
        else:
            body_z_pos = z_offset
            
        body = ET.SubElement(parent, "body",
                             name=f"link_{i}", pos=f"0 0 {body_z_pos:.6f}")
        ET.SubElement(body, "inertial",
                      mass=str(MASS_PER_LINK),
                      diaginertia=f"{I_LINK} {I_LINK} {I_LINK}",
                      pos="0 0 0")
        # visual geom
        ET.SubElement(body, "geom", type="mesh", mesh=vis_name,
                      **{"class":"vis"}, name=f"g_vis_{i}")
        # collision geoms
        for j,col_name in enumerate(col_mesh_names):
            ET.SubElement(body, "geom", type="mesh", mesh=col_name,
                          **{"class":"col"}, name=f"g_col_{i}_{j}")
        # hinge
        ET.SubElement(body, "joint", name=f"hinge_{i}", type="hinge",
                      axis="0 1 0",
                      stiffness=str(K_HINGE), damping=str(C_HINGE),
                       range=f"{-UNIT_ANGLE_DEG} {UNIT_ANGLE_DEG}")
        # cable routing sites
        ET.SubElement(body, "site", name=f"s_{i}_L", pos="0.01 0.005 0")
        ET.SubElement(body, "site", name=f"s_{i}_R", pos="-0.01 0.005 0")
        parent = body
        
        print(f"body_z_pos: {body_z_pos:.6f}")
        print(f"current_link_stacking_height: {current_link_stacking_height:.6f}")
        
        # Update z_offset for the next link - simply add the mesh height
        z_offset = body_z_pos + current_link_stacking_height

    # ---------------- Tendons ---------------- #
    tendon_root = ET.SubElement(root, "tendon")
    for side in ("L","R"):
        t = ET.SubElement(tendon_root, "spatial",
                          name=f"t_{side.lower()}",
                          stiffness=str(K_TENDON),
                          damping=str(D_TENDON))
        for i in range(len(link_info)):
            ET.SubElement(t, "site", site=f"s_{i}_{side}")

    # ---------------- Reels & actuators ---------------- #
    reel_body = ET.SubElement(world, "body", name="reels", pos="0 0.04 0")
    ET.SubElement(reel_body, "inertial",
                  mass=str(REELS_MASS),
                  diaginertia=f"{REELS_DIAGINERTIA} {REELS_DIAGINERTIA} {REELS_DIAGINERTIA}",
                  pos="0 0 0")
    for side in ("L","R"):
        ET.SubElement(reel_body, "joint", name=f"reel_{side}", axis="0 0 1",
                      type="hinge")
    act = ET.SubElement(root, "actuator")
    gear = 2*math.pi*SPOOL_RADIUS_M
    for side in ("L","R"):
        ET.SubElement(act, "motor", joint=f"reel_{side}",
                      name=f"servo_{side.lower()}",
                      gear=str(gear),
                      ctrllimited="true", ctrlrange=f"{-SERVO_SPEED_RAD_S} {SERVO_SPEED_RAD_S}",
                      forcelimited="true", forcerange=f"0 {SERVO_TORQUE_NM}")


    tree = ET.ElementTree(root)
    tree.write(xml_out, pretty_print=True, xml_declaration=True, encoding="utf-8")
    logging.info(f"Wrote {xml_out}")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser()
    parser.add_argument("--meshdir", default=ASSET_DIR)
    parser.add_argument("--output",  default="spirob.xml")
    opts = parser.parse_args()
    build_spirob(Path(opts.meshdir), Path(opts.output))
