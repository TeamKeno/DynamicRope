# -*- coding: utf-8 -*-
#
# DynamicRope 기본 밧줄 머티리얼 생성기 (M_RopeDefault)
# ------------------------------------------------------------------------------
# 헴프/황마 절차적 밧줄 룩. 텍스처 에셋 없이 UV만으로 꼬임(strand) 패턴을 만든다.
#   - U(=길이) / V(=원주) 로 나선 위상 phase = V*StrandCount + U*TwistTurns
#   - sin(phase) 로 strand 마스크 → BaseColor 2톤 + Roughness 홈 변화
#   - cos(phase) 로 탄젠트공간 노멀 섭동(홈 음영)
# 노출 파라미터: Tint / StrandCount / TwistTurns / Roughness / NormalStrength
#
# 실행 방법:
#   1) Edit → Plugins → "Python Editor Script Plugin" 활성화 후 에디터 재시작
#   2) Tools(또는 Window) → Execute Python Script... 로 이 파일 선택
#      또는 Output Log 하단 콘솔을 Python 모드로 두고:
#         exec(open(r"<이 파일 경로>").read())
#   3) /DynamicRope/Materials/M_RopeDefault 생성 확인 후 저장/서브밋
#
# 재실행하면 기존 에셋을 지우고 다시 만든다(idempotent).

import unreal

MAT_DIR = "/DynamicRope/Materials"
MAT_NAME = "M_RopeDefault"
FULL = MAT_DIR + "/" + MAT_NAME

mel = unreal.MaterialEditingLibrary
eal = unreal.EditorAssetLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()

if eal.does_asset_exist(FULL):
    eal.delete_asset(FULL)

mat = tools.create_asset(MAT_NAME, MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property("two_sided", False)


def node(cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)


def wire(a, b, b_in, a_out=""):
    mel.connect_material_expressions(a, a_out, b, b_in)


def constant(x, y, v):
    c = node(unreal.MaterialExpressionConstant, x, y)
    c.set_editor_property("r", float(v))
    return c


# ---- 파라미터 -----------------------------------------------------------------
tint = node(unreal.MaterialExpressionVectorParameter, -1500, -260)
tint.set_editor_property("parameter_name", "Tint")
tint.set_editor_property("default_value", unreal.LinearColor(0.62, 0.44, 0.24, 1.0))  # 따뜻한 헴프 탄

strand = node(unreal.MaterialExpressionScalarParameter, -1700, 60)
strand.set_editor_property("parameter_name", "StrandCount")
strand.set_editor_property("default_value", 3.0)  # 원주 방향 가닥 수

twist = node(unreal.MaterialExpressionScalarParameter, -1700, 160)
twist.set_editor_property("parameter_name", "TwistTurns")
twist.set_editor_property("default_value", 9.0)  # 길이 방향 꼬임 회전 수

rough_p = node(unreal.MaterialExpressionScalarParameter, -520, 300)
rough_p.set_editor_property("parameter_name", "Roughness")
rough_p.set_editor_property("default_value", 0.82)

nstr = node(unreal.MaterialExpressionScalarParameter, -760, 560)
nstr.set_editor_property("parameter_name", "NormalStrength")
nstr.set_editor_property("default_value", 0.35)

# ---- UV 분해: U(길이), V(원주) ------------------------------------------------
uv = node(unreal.MaterialExpressionTextureCoordinate, -1900, 20)
u = node(unreal.MaterialExpressionComponentMask, -1700, -60)
u.set_editor_property("r", True); u.set_editor_property("g", False)
u.set_editor_property("b", False); u.set_editor_property("a", False)
v = node(unreal.MaterialExpressionComponentMask, -1700, 20)
v.set_editor_property("r", False); v.set_editor_property("g", True)
v.set_editor_property("b", False); v.set_editor_property("a", False)
wire(uv, u, ""); wire(uv, v, "")

# ---- phase = V*StrandCount + U*TwistTurns -------------------------------------
v_mul = node(unreal.MaterialExpressionMultiply, -1400, 20)
wire(v, v_mul, "A"); wire(strand, v_mul, "B")
u_mul = node(unreal.MaterialExpressionMultiply, -1400, 140)
wire(u, u_mul, "A"); wire(twist, u_mul, "B")
phase = node(unreal.MaterialExpressionAdd, -1180, 70)
wire(v_mul, phase, "A"); wire(u_mul, phase, "B")

# ---- strand 마스크: sin(phase)*0.5 + 0.5 (0..1) -------------------------------
sine = node(unreal.MaterialExpressionSine, -980, 20)
sine.set_editor_property("period", 1.0)  # sin(2π*phase)
wire(phase, sine, "")
half = constant(-980, -110, 0.5)
mask_mul = node(unreal.MaterialExpressionMultiply, -800, 20)
wire(sine, mask_mul, "A"); wire(half, mask_mul, "B")
mask = node(unreal.MaterialExpressionAdd, -640, 20)
wire(mask_mul, mask, "A"); wire(half, mask, "B")

# ---- BaseColor: lerp(Tint*0.5, Tint, mask) -----------------------------------
darkc = constant(-1160, -280, 0.5)
dark = node(unreal.MaterialExpressionMultiply, -980, -240)
wire(tint, dark, "A"); wire(darkc, dark, "B")
basecol = node(unreal.MaterialExpressionLinearInterpolate, -300, -180)
wire(dark, basecol, "A"); wire(tint, basecol, "B"); wire(mask, basecol, "Alpha")
mel.connect_material_property(basecol, "", unreal.MaterialProperty.MP_BASE_COLOR)

# ---- Roughness: Roughness + (1-mask)*0.08 (홈이 더 거칠다) --------------------
one = constant(-520, 440, 1.0)
inv = node(unreal.MaterialExpressionSubtract, -360, 400)
wire(one, inv, "A"); wire(mask, inv, "B")
amt = constant(-360, 500, 0.08)
inv_amt = node(unreal.MaterialExpressionMultiply, -200, 420)
wire(inv, inv_amt, "A"); wire(amt, inv_amt, "B")
rough = node(unreal.MaterialExpressionAdd, -40, 340)
wire(rough_p, rough, "A"); wire(inv_amt, rough, "B")
mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)

# ---- Normal: cos(phase) 로 탄젠트공간 섭동 (x=길이, y=원주, z=1) --------------
cosine = node(unreal.MaterialExpressionCosine, -980, 620)
cosine.set_editor_property("period", 1.0)
wire(phase, cosine, "")
neg = constant(-980, 740, -1.0)
ns_neg = node(unreal.MaterialExpressionMultiply, -800, 660)  # -NormalStrength
wire(nstr, ns_neg, "A"); wire(neg, ns_neg, "B")
# nx = cos * (-NormalStrength * TwistTurns)
nx_s = node(unreal.MaterialExpressionMultiply, -620, 600)
wire(ns_neg, nx_s, "A"); wire(twist, nx_s, "B")
nx = node(unreal.MaterialExpressionMultiply, -460, 620)
wire(cosine, nx, "A"); wire(nx_s, nx, "B")
# ny = cos * (-NormalStrength * StrandCount)
ny_s = node(unreal.MaterialExpressionMultiply, -620, 740)
wire(ns_neg, ny_s, "A"); wire(strand, ny_s, "B")
ny = node(unreal.MaterialExpressionMultiply, -460, 760)
wire(cosine, ny, "A"); wire(ny_s, ny, "B")
# append (nx, ny, 1) → float3 → normalize
onez = constant(-460, 880, 1.0)
app1 = node(unreal.MaterialExpressionAppendVector, -280, 680)
wire(nx, app1, "A"); wire(ny, app1, "B")
app2 = node(unreal.MaterialExpressionAppendVector, -120, 720)
wire(app1, app2, "A"); wire(onez, app2, "B")
norm = node(unreal.MaterialExpressionNormalize, 40, 720)
wire(app2, norm, "")
mel.connect_material_property(norm, "", unreal.MaterialProperty.MP_NORMAL)

# ---- 컴파일 + 저장 ------------------------------------------------------------
mel.recompile_material(mat)
eal.save_asset(FULL)
unreal.log("[DynamicRope] Created {}".format(FULL))
