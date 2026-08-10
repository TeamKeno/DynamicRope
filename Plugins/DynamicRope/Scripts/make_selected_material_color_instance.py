# -*- coding: utf-8 -*-
#
# Creates a new coloured Material Instance from the Material or Material
# Instance currently selected in the Unreal Content Browser.
#
# Usage:
#   1. Change the settings below if needed.
#   2. Select one parent Material or Material Instance in the Content Browser.
#   3. Tools -> Execute Python Script...
#   4. Select this file.
#
# Existing assets are never deleted or overwritten.

import unreal


# ------------------------------------------------------------------------------
# User settings
# ------------------------------------------------------------------------------

# Name of the vector parameter exposed by the parent material.
COLOR_PARAMETER_NAME = "Tint"

# RGB values use the same 0.0 to 1.0 range as Unreal's Linear Color.
INSTANCE_COLOR = (0.15, 0.45, 1.0)

# Used in the new asset name:
#   MI_<ParentName>_<INSTANCE_LABEL>
INSTANCE_LABEL = "Blue"


eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()


def fail(message):
    unreal.log_error("[DynamicRope] {}".format(message))
    unreal.EditorDialog.show_message(
        "Create Material Instance",
        message,
        unreal.AppMsgType.OK,
    )
    raise RuntimeError(message)


def get_selected_parent():
    selected_assets = unreal.EditorUtilityLibrary.get_selected_assets()
    material_assets = [
        asset for asset in selected_assets
        if isinstance(asset, unreal.MaterialInterface)
    ]

    if len(material_assets) != 1:
        fail(
            "Select exactly one parent Material or Material Instance "
            "in the Content Browser, then run the script again."
        )

    return material_assets[0]


def get_asset_directory(asset):
    # Example:
    #   /DynamicRope/DynamicRope/Materials/M_RopeDefault.M_RopeDefault
    # becomes:
    #   /DynamicRope/DynamicRope/Materials
    object_path = asset.get_path_name()
    package_path = object_path.split(".", 1)[0]
    return package_path.rsplit("/", 1)[0]


def make_base_instance_name(parent):
    parent_name = parent.get_name()

    # Avoid names such as MI_M_RopeDefault_Blue.
    if parent_name.startswith("MI_"):
        parent_name = parent_name[3:]
    elif parent_name.startswith("M_"):
        parent_name = parent_name[2:]

    return "MI_{}_{}".format(parent_name, INSTANCE_LABEL)


def find_unused_name(directory, base_name):
    candidate = base_name
    suffix = 1

    while eal.does_asset_exist("{}/{}".format(directory, candidate)):
        candidate = "{}_{:02d}".format(base_name, suffix)
        suffix += 1

    return candidate


parent = get_selected_parent()
destination_directory = get_asset_directory(parent)
base_name = make_base_instance_name(parent)
instance_name = find_unused_name(destination_directory, base_name)
instance_path = "{}/{}".format(destination_directory, instance_name)

instance = asset_tools.create_asset(
    instance_name,
    destination_directory,
    unreal.MaterialInstanceConstant,
    unreal.MaterialInstanceConstantFactoryNew(),
)

if instance is None:
    fail("Could not create Material Instance: {}".format(instance_path))

instance.set_editor_property("parent", parent)

r, g, b = INSTANCE_COLOR
mel.set_material_instance_vector_parameter_value(
    instance,
    COLOR_PARAMETER_NAME,
    unreal.LinearColor(float(r), float(g), float(b), 1.0),
)
mel.update_material_instance(instance)

if not eal.save_asset(instance_path):
    fail("Could not save Material Instance: {}".format(instance_path))

try:
    eal.sync_browser_to_objects([instance_path])
except Exception:
    pass

unreal.log(
    "[DynamicRope] Created {} from {} with {} = ({:.3f}, {:.3f}, {:.3f})".format(
        instance_path,
        parent.get_path_name(),
        COLOR_PARAMETER_NAME,
        r,
        g,
        b,
    )
)
