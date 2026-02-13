#!/usr/bin/env python3
import sys
import xml.etree.ElementTree as ET

if len(sys.argv) != 3:
    print("Usage: python3 scale_urdf.py input.urdf output.urdf")
    sys.exit(1)

INPUT_FILE = sys.argv[1]
OUTPUT_FILE = sys.argv[2]

# mm → m
k = 0.01
k3 = k ** 3
k5 = k ** 5

tree = ET.parse(INPUT_FILE)
root = tree.getroot()

def scale_xyz(attr_value):
    vals = [float(v) for v in attr_value.split()]
    scaled = [v * k for v in vals]
    return " ".join(f"{v:.9g}" for v in scaled)

def scale_inertia(attr_value):
    return f"{float(attr_value) * k5:.12g}"

def scale_mass(attr_value):
    return f"{float(attr_value) * k3:.12g}"

def scale_size(attr_value):
    vals = [float(v) for v in attr_value.split()]
    scaled = [v * k for v in vals]
    return " ".join(f"{v:.9g}" for v in scaled)

# --- Scale all origins ---
for origin in root.findall(".//origin"):
    if "xyz" in origin.attrib:
        origin.attrib["xyz"] = scale_xyz(origin.attrib["xyz"])

# --- Scale collision box sizes ---
for box in root.findall(".//box"):
    if "size" in box.attrib:
        box.attrib["size"] = scale_size(box.attrib["size"])

# --- Scale mesh scale attributes (if present) ---
for mesh in root.findall(".//mesh"):
    if "scale" in mesh.attrib:
        mesh.attrib["scale"] = scale_size(mesh.attrib["scale"])
    else:
        # If no scale exists, inject one
        mesh.attrib["scale"] = f"{k} {k} {k}"

# --- Scale masses ---
for mass in root.findall(".//mass"):
    if "value" in mass.attrib:
        mass.attrib["value"] = scale_mass(mass.attrib["value"])

# --- Scale inertia tensors ---
for inertia in root.findall(".//inertia"):
    for attr in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
        if attr in inertia.attrib:
            inertia.attrib[attr] = scale_inertia(inertia.attrib[attr])

tree.write(OUTPUT_FILE, encoding="utf-8", xml_declaration=True)

print("Scaling complete.")
print("Length scale factor:", k)
print("Mass scale factor:", k3)
print("Inertia scale factor:", k5)
