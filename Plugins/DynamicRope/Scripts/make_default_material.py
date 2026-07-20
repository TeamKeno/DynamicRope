# -*- coding: utf-8 -*-
#
# DynamicRope 기본 밧줄 머티리얼 생성기 (M_RopeDefault)
# ------------------------------------------------------------------------------
# 헴프/황마 절차적 밧줄 룩. 텍스처 에셋 없이 UV만으로 표현한다.
#   1) 꼬임(strand) 패턴:  phase = V*StrandCount + U*TwistTurns
#      (U=길이축 UV=누적 호길이/원주, V=원주 UV. 튜브 빌더가 U,V를 같은 물리 스케일로 채워
#       TwistTurns=원주-길이당 회전 수 → 로프 길이 무관 밀도 일정. StrandCount≈TwistTurns면 ~45° 레이.)
#      - sin(phase) → strand 마스크 → BaseColor 2톤 + Roughness 홈 변화
#      - cos(phase) → 탄젠트공간 노말 섭동(꼬임선 음영). NormalStrength로 세기.
#   2) 불규칙 섬유 거침:  Custom HLSL 값노이즈(UV 고정, 로프 움직여도 안 헤엄침)
#      - float4(섬유노말.xyz, 높이) 1회 평가 → 미세 노말 + Roughness 얼룩에 공용.
#      - FiberScale(주파수) / FiberNormalStrength(범프) / FiberRoughness(거칠기 얼룩).
# 노출 파라미터: Tint / StrandCount / TwistTurns / Roughness / NormalStrength /
#                FiberScale / FiberNormalStrength / FiberRoughness
#
# 실행: Tools → Execute Python Script... 로 이 파일 선택.
# 재실행 안전: 기존 에셋을 지우고 같은 경로에 다시 만든다(idempotent). 자식 MI_* 인스턴스는
# 이 스크립트에서 로드/저장하지 않으므로 .uasset이 그대로라 부모 경로 참조가 유지된다.

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


def scalar_param(name, default, x, y):
    p = node(unreal.MaterialExpressionScalarParameter, x, y)
    p.set_editor_property("parameter_name", name)
    p.set_editor_property("default_value", float(default))
    return p


# ---- 파라미터 -----------------------------------------------------------------
tint = node(unreal.MaterialExpressionVectorParameter, -1700, -320)
tint.set_editor_property("parameter_name", "Tint")
tint.set_editor_property("default_value", unreal.LinearColor(0.62, 0.44, 0.24, 1.0))  # 따뜻한 헴프 탄

strand = scalar_param("StrandCount", 3.0, -1900, 60)   # 원주 방향 가닥 수
twist = scalar_param("TwistTurns", 3.0, -1900, 160)    # 원주-길이(2πR)당 꼬임 회전 수(UV.x=호길이/원주)
rough_p = scalar_param("Roughness", 0.82, -520, 300)
nstr = scalar_param("NormalStrength", 0.55, -900, 560)  # 꼬임선 노말 세기(도드라지게 ↑)

fiber_scale = scalar_param("FiberScale", 40.0, -1500, 900)          # 섬유 노이즈 주파수
fiber_nstr = scalar_param("FiberNormalStrength", 0.30, -1500, 1000)  # 섬유 미세 범프 세기
fiber_rough = scalar_param("FiberRoughness", 0.15, -520, 460)        # 섬유 거칠기 얼룩 양

# ---- UV 분해: U(길이), V(원주) ------------------------------------------------
uv = node(unreal.MaterialExpressionTextureCoordinate, -2100, 20)
u = node(unreal.MaterialExpressionComponentMask, -1900, -60)
u.set_editor_property("r", True); u.set_editor_property("g", False)
u.set_editor_property("b", False); u.set_editor_property("a", False)
v = node(unreal.MaterialExpressionComponentMask, -1900, 20)
v.set_editor_property("r", False); v.set_editor_property("g", True)
v.set_editor_property("b", False); v.set_editor_property("a", False)
wire(uv, u, ""); wire(uv, v, "")

# ---- phase = V*StrandCount + U*TwistTurns -------------------------------------
v_mul = node(unreal.MaterialExpressionMultiply, -1600, 20)
wire(v, v_mul, "A"); wire(strand, v_mul, "B")
u_mul = node(unreal.MaterialExpressionMultiply, -1600, 140)
wire(u, u_mul, "A"); wire(twist, u_mul, "B")
phase = node(unreal.MaterialExpressionAdd, -1380, 70)
wire(v_mul, phase, "A"); wire(u_mul, phase, "B")

# ---- strand 마스크: sin(phase)*0.5 + 0.5 (0..1) -------------------------------
sine = node(unreal.MaterialExpressionSine, -1180, 20)
sine.set_editor_property("period", 1.0)  # sin(2π*phase)
wire(phase, sine, "")
half = constant(-1180, -110, 0.5)
mask_mul = node(unreal.MaterialExpressionMultiply, -1000, 20)
wire(sine, mask_mul, "A"); wire(half, mask_mul, "B")
mask = node(unreal.MaterialExpressionAdd, -840, 20)
wire(mask_mul, mask, "A"); wire(half, mask, "B")

# ---- BaseColor: lerp(Tint*0.5, Tint, mask) -----------------------------------
darkc = constant(-1360, -280, 0.5)
dark = node(unreal.MaterialExpressionMultiply, -1180, -240)
wire(tint, dark, "A"); wire(darkc, dark, "B")
basecol = node(unreal.MaterialExpressionLinearInterpolate, -300, -180)
wire(dark, basecol, "A"); wire(tint, basecol, "B"); wire(mask, basecol, "Alpha")
mel.connect_material_property(basecol, "", unreal.MaterialProperty.MP_BASE_COLOR)

# ---- 불규칙 섬유 노이즈: Custom HLSL, float4(노말.xyz, 높이) ------------------
# UV*Scale 값노이즈(smoothstep 보간) + 해석적 미분으로 탄젠트공간 노말. UV 기준이라
# 로프가 움직여도 표면에 고정된다(월드 노이즈의 헤엄 현상 없음).
fiber = node(unreal.MaterialExpressionCustom, -1000, 900)
fiber.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT4)
fiber.set_editor_property("description", "RopeFiberDetail")
def custom_input(name):
    ci = unreal.CustomInput()
    ci.set_editor_property("input_name", name)
    return ci

fiber.set_editor_property("inputs", [custom_input("UV"), custom_input("Scale"), custom_input("Strength")])
fiber.set_editor_property("code", (
    "float2 P = UV * Scale;\n"
    "float2 I = floor(P);\n"
    "float2 F = frac(P);\n"
    "float2 W = F*F*(3.0-2.0*F);\n"
    "float h00 = frac(sin(dot(I+float2(0.0,0.0), float2(127.1,311.7)))*43758.5453);\n"
    "float h10 = frac(sin(dot(I+float2(1.0,0.0), float2(127.1,311.7)))*43758.5453);\n"
    "float h01 = frac(sin(dot(I+float2(0.0,1.0), float2(127.1,311.7)))*43758.5453);\n"
    "float h11 = frac(sin(dot(I+float2(1.0,1.0), float2(127.1,311.7)))*43758.5453);\n"
    "float b = lerp(h00, h10, W.x);\n"
    "float t = lerp(h01, h11, W.x);\n"
    "float hgt = lerp(b, t, W.y);\n"
    "float2 dW = 6.0*F*(1.0-F);\n"
    "float dHx = lerp(h10-h00, h11-h01, W.y) * dW.x;\n"
    "float dHy = (t - b) * dW.y;\n"
    "float3 N = normalize(float3(-dHx*Strength, -dHy*Strength, 1.0));\n"
    "return float4(N, hgt);\n"
))
wire(uv, fiber, "UV")
wire(fiber_scale, fiber, "Scale")
wire(fiber_nstr, fiber, "Strength")

fiber_n = node(unreal.MaterialExpressionComponentMask, -760, 860)  # 섬유 노말(xyz)
fiber_n.set_editor_property("r", True); fiber_n.set_editor_property("g", True)
fiber_n.set_editor_property("b", True); fiber_n.set_editor_property("a", False)
wire(fiber, fiber_n, "")
fiber_h = node(unreal.MaterialExpressionComponentMask, -760, 1000)  # 섬유 높이(a)
fiber_h.set_editor_property("r", False); fiber_h.set_editor_property("g", False)
fiber_h.set_editor_property("b", False); fiber_h.set_editor_property("a", True)
wire(fiber, fiber_h, "")

# ---- Roughness: Roughness + (1-mask)*0.08 + (fiberHeight-0.5)*FiberRoughness --
one = constant(-520, 620, 1.0)
inv = node(unreal.MaterialExpressionSubtract, -360, 400)
wire(one, inv, "A"); wire(mask, inv, "B")
amt = constant(-360, 500, 0.08)
inv_amt = node(unreal.MaterialExpressionMultiply, -200, 420)
wire(inv, inv_amt, "A"); wire(amt, inv_amt, "B")
rough_strand = node(unreal.MaterialExpressionAdd, -40, 340)
wire(rough_p, rough_strand, "A"); wire(inv_amt, rough_strand, "B")
# 섬유 거칠기 얼룩: (height-0.5)*FiberRoughness
fh_half = constant(-520, 1080, 0.5)
fh_c = node(unreal.MaterialExpressionSubtract, -360, 1000)
wire(fiber_h, fh_c, "A"); wire(fh_half, fh_c, "B")
fh_amt = node(unreal.MaterialExpressionMultiply, -200, 1000)
wire(fh_c, fh_amt, "A"); wire(fiber_rough, fh_amt, "B")
rough = node(unreal.MaterialExpressionAdd, 120, 400)
wire(rough_strand, rough, "A"); wire(fh_amt, rough, "B")
mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)

# ---- Normal: 꼬임선(strand) 노말 + 섬유 미세 노말 = Add → Normalize -----------
# 꼬임선 노말: cos(phase) 로 탄젠트공간 섭동(x=길이, y=원주, z=1)
cosine = node(unreal.MaterialExpressionCosine, -1180, 620)
cosine.set_editor_property("period", 1.0)
wire(phase, cosine, "")
neg = constant(-1180, 740, -1.0)
ns_neg = node(unreal.MaterialExpressionMultiply, -1000, 660)  # -NormalStrength
wire(nstr, ns_neg, "A"); wire(neg, ns_neg, "B")
# nx = cos * (-NormalStrength * TwistTurns)
nx_s = node(unreal.MaterialExpressionMultiply, -820, 600)
wire(ns_neg, nx_s, "A"); wire(twist, nx_s, "B")
nx = node(unreal.MaterialExpressionMultiply, -660, 620)
wire(cosine, nx, "A"); wire(nx_s, nx, "B")
# ny = cos * (-NormalStrength * StrandCount)
ny_s = node(unreal.MaterialExpressionMultiply, -820, 740)
wire(ns_neg, ny_s, "A"); wire(strand, ny_s, "B")
ny = node(unreal.MaterialExpressionMultiply, -660, 760)
wire(cosine, ny, "A"); wire(ny_s, ny, "B")
onez = constant(-660, 880, 1.0)
app1 = node(unreal.MaterialExpressionAppendVector, -480, 680)
wire(nx, app1, "A"); wire(ny, app1, "B")
app2 = node(unreal.MaterialExpressionAppendVector, -320, 720)
wire(app1, app2, "A"); wire(onez, app2, "B")
strand_n = node(unreal.MaterialExpressionNormalize, -160, 720)
wire(app2, strand_n, "")
# 꼬임선 + 섬유 노말 합성 후 정규화(디테일 노말 레이어링)
norm_add = node(unreal.MaterialExpressionAdd, 60, 800)
wire(strand_n, norm_add, "A"); wire(fiber_n, norm_add, "B")
norm = node(unreal.MaterialExpressionNormalize, 240, 800)
wire(norm_add, norm, "")
mel.connect_material_property(norm, "", unreal.MaterialProperty.MP_NORMAL)

# ---- 컴파일 + 저장 ------------------------------------------------------------
mel.recompile_material(mat)
eal.save_asset(FULL)
unreal.log("[DynamicRope] Rebuilt {}".format(FULL))
