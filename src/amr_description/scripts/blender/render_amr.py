#!/usr/bin/env python3
"""Render an existing generated scene with the same procedural inspection setup."""
from pathlib import Path
import sys
import bpy
sys.path.insert(0,str(Path(__file__).resolve().parent))
from generate_amr import ROOT, create_inspection_renders, load_parameters, save_blend
from validate_amr import validate_scene


if __name__ == '__main__':
    scene = bpy.data.scenes['AMR_Generation']
    parameters = load_parameters(ROOT)
    validate_scene(scene,parameters)
    create_inspection_renders(scene,parameters,ROOT)
    save_blend(scene,ROOT)
