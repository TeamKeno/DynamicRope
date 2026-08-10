# -*- coding: utf-8 -*-
#
# Adds a brand-new, texture-free rope material asset on every run.
#
# Run from Unreal Editor:
#   Tools -> Execute Python Script... -> make_new_plain_rope_material.py
#
# Output:
#   /DynamicRope/DynamicRope/Materials/M_RopePlain
#   /DynamicRope/DynamicRope/Materials/M_RopePlain_01
#   /DynamicRope/DynamicRope/Materials/M_RopePlain_02
#   ...
#
# Existing assets are never deleted or overwritten.

import unreal


MAT_DIR = "/DynamicRope/DynamicRope/Materials"
BASE_NAME = "M_RopePlain"

eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()


def find_unused_name():
    """Return a material name that does not already exist in MAT_DIR."""
    candidate = BASE_NAME
    suffix = 1

    while eal.does_asset_exist("{}/{}".format(MAT_DIR, candidate)):
        candidate = "{}_{:02d}".format(BASE_NAME, suffix)
        suffix += 1

    return candidate


material_name = find_unused_name()
material_path = "{}/{}".format(MAT_DIR, material_name)

material = asset_tools.create_asset(
    material_name,
    MAT_DIR,
    unreal.Material,
    unreal.MaterialFactoryNew(),
)

if material is None:
    raise RuntimeError("Could not create material: {}".format(material_path))

# A plain, texture-free material. These parameters can be changed directly in
# the Material Editor or from a Material Instance made from this material.
tint = mel.create_material_expression(
    material, unreal.MaterialExpressionVectorParameter, -300, -80
)
tint.set_editor_property("parameter_name", "Tint")
tint.set_editor_property(
    "default_value", unreal.LinearColor(0.62, 0.44, 0.24, 1.0)
)
mel.connect_material_property(
    tint, "", unreal.MaterialProperty.MP_BASE_COLOR
)

roughness = mel.create_material_expression(
    material, unreal.MaterialExpressionScalarParameter, -300, 80
)
roughness.set_editor_property("parameter_name", "Roughness")
roughness.set_editor_property("default_value", 0.82)
mel.connect_material_property(
    roughness, "", unreal.MaterialProperty.MP_ROUGHNESS
)

mel.recompile_material(material)

if not eal.save_asset(material_path):
    raise RuntimeError("Could not save material: {}".format(material_path))

# Focus the Content Browser on the newly created asset when the API is
# available in the current Unreal version.
try:
    eal.sync_browser_to_objects([material_path])
except Exception:
    pass

unreal.log("[DynamicRope] Created new texture-free material: {}".format(material_path))
