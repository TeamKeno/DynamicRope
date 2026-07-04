# -*- coding: utf-8 -*-
#
# DynamicRope 밧줄 머티리얼 인스턴스 프리셋 생성기
# ------------------------------------------------------------------------------
# M_RopeDefault(부모)의 노출 파라미터(Tint / StrandCount / TwistTurns / Roughness /
# NormalStrength)만 오버라이드하는 MaterialInstanceConstant 프리셋들을 만든다.
# 부모 머티리얼 그래프는 건드리지 않는다 — 색감/꼬임/거칠기만 바꾼다.
#
# 실행: make_default_material.py 와 동일(Tools → Execute Python Script...).
#       부모 M_RopeDefault 가 먼저 존재해야 한다.
# 재실행하면 기존 프리셋을 지우고 다시 만든다(idempotent).

import unreal

MAT_DIR = "/DynamicRope/Materials"
PARENT_PATH = MAT_DIR + "/M_RopeDefault"

eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()

parent = eal.load_asset(PARENT_PATH)
if parent is None:
    raise Exception("부모 머티리얼이 없습니다: {} — 먼저 make_default_material.py 실행".format(PARENT_PATH))

# name -> (Tint RGB, StrandCount, TwistTurns, Roughness, NormalStrength)
PRESETS = {
    # 밝은 천연 마닐라/사이잘
    "MI_Rope_Manila":       ((0.74, 0.60, 0.36), 3.0,  8.0, 0.88, 0.42),
    # 오래된 짙은 황마
    "MI_Rope_JuteDark":     ((0.34, 0.24, 0.13), 3.0, 10.0, 0.90, 0.50),
    # 검은 나일론 파라코드 — 촘촘한 꼬임 + 약간 광택
    "MI_Rope_ParacordBlack":((0.02, 0.02, 0.025),5.0, 16.0, 0.50, 0.22),
    # 장식용 붉은 밧줄
    "MI_Rope_Crimson":      ((0.42, 0.05, 0.05), 3.0,  9.0, 0.65, 0.35),
}


def make_instance(name, tint_rgb, strand, twist, rough, nstr):
    path = MAT_DIR + "/" + name
    if eal.does_asset_exist(path):
        eal.delete_asset(path)

    mic = tools.create_asset(name, MAT_DIR, unreal.MaterialInstanceConstant,
                             unreal.MaterialInstanceConstantFactoryNew())
    mic.set_editor_property("parent", parent)

    r, g, b = tint_rgb
    mel.set_material_instance_vector_parameter_value(mic, "Tint", unreal.LinearColor(r, g, b, 1.0))
    mel.set_material_instance_scalar_parameter_value(mic, "StrandCount", strand)
    mel.set_material_instance_scalar_parameter_value(mic, "TwistTurns", twist)
    mel.set_material_instance_scalar_parameter_value(mic, "Roughness", rough)
    mel.set_material_instance_scalar_parameter_value(mic, "NormalStrength", nstr)

    mel.update_material_instance(mic)
    eal.save_asset(path)
    unreal.log("[DynamicRope] Created {}".format(path))


for preset_name, args in PRESETS.items():
    make_instance(preset_name, *args)

unreal.log("[DynamicRope] {} rope material instance(s) created.".format(len(PRESETS)))
