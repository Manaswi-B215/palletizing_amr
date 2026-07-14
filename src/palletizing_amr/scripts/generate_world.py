#!/usr/bin/env python3

import argparse
import os
import sys
import tempfile
import xml.etree.ElementTree as ET


# Supported colors
LINE_COLORS = {
    "dark_red": ("0.55 0.0 0.0 1", "0.55 0.0 0.0 1"),
    "green": ("0.0 0.6 0.0 1", "0.0 0.6 0.0 1"),
    "blue": ("0.0 0.0 1.0 1", "0.0 0.0 1.0 1"),
}


def main():

    parser = argparse.ArgumentParser(
        description="Generate a temporary warehouse world with colored lines."
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Path to warehouse.sdf"
    )

    parser.add_argument(
        "--color",
        default="dark_red",
        choices=LINE_COLORS.keys(),
        help="Line color"
    )

    args = parser.parse_args()

    ambient, diffuse = LINE_COLORS[args.color]

    tree = ET.parse(args.input)
    root = tree.getroot()

    # Change only the models that contain a material tag
    for material in root.iter("material"):

        ambient_tag = material.find("ambient")
        diffuse_tag = material.find("diffuse")

        if ambient_tag is not None:
            ambient_tag.text = ambient

        if diffuse_tag is not None:
            diffuse_tag.text = diffuse

    temp_file = tempfile.NamedTemporaryFile(
        delete=False,
        suffix=".sdf"
    )

    tree.write(
        temp_file.name,
        encoding="utf-8",
        xml_declaration=True
    )

    temp_file.close()

    # IMPORTANT:
    # Print only the generated filename.
    print(temp_file.name)


if __name__ == "__main__":
    main()
