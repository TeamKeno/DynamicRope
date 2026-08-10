# -*- coding: utf-8 -*-
#
# DynamicRope demo elevator metal material generator (M_ElevatorMetal)
# ------------------------------------------------------------------------------
# A brushed stainless-steel look for the elevator gimmick, expressed from the UVs alone with no
# texture assets, the same way M_RopeDefault is. Three layers:
#   1) The brush grain: a custom HLSL value noise sampled with an anisotropic UV scale — stretched
#      along U, compressed along V — so the cells become long thin streaks running along U (rotate
#      the mesh UVs, not the material, if the grain should run the other way).
#      - The noise height drives a subtle two-tone base colour and the roughness mottling, which is
#        what actually reads as "brushed" under a moving light.
#      - The analytic derivative of the same evaluation gives a fine tangent-space normal, so the
#        streaks catch the light as shallow grooves rather than as a flat tint.
#   2) The anisotropic highlight: Metallic 1 with the Anisotropy pin driven by a parameter. Brushed
#      metal's signature is the highlight smearing perpendicular to the grain; the pin does that
#      directly from the mesh tangent basis, no extra graph work needed.
#   3) The wear smudge: a second, isotropic, low-frequency evaluation of the same noise mottles the
#      roughness at panel scale — the faint hand-smudge / patina unevenness that keeps a large flat
#      elevator panel from looking like a perfect mirror. Height only; its normal is discarded.
# The final roughness is clamped so neither layer can push it into a dead mirror or full chalk.
#
# Exposed parameters: Tint / Metallic / Roughness / Anisotropy /
#                BrushScale / BrushStretch / BrushRoughness / BrushNormalStrength / BrushTint /
#                WearScale / WearRoughness
#
# To run: select this file through Tools -> Execute Python Script...
# Safe to re-run: it deletes the existing asset and recreates it at the same path, so it is
# idempotent. Child MI_* instances are neither loaded nor saved here, so their .uasset files are
# untouched and their references to the parent path survive.

import unreal

MAT_DIR = "/DynamicRope/DynamicRope/Materials"
MAT_NAME = "M_ElevatorMetal"
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


# ---- Parameters ---------------------------------------------------------------
tint = node(unreal.MaterialExpressionVectorParameter, -1700, -320)
tint.set_editor_property("parameter_name", "Tint")
tint.set_editor_property("default_value", unreal.LinearColor(0.55, 0.56, 0.58, 1.0))  # Neutral stainless, a touch cool.

metallic = scalar_param("Metallic", 1.0, -520, -80)
rough_p = scalar_param("Roughness", 0.35, -520, 300)
aniso = scalar_param("Anisotropy", 0.6, -520, 40)          # Highlight smear across the grain; 0 falls back to isotropic.

brush_scale = scalar_param("BrushScale", 250.0, -1900, 700)      # Streak frequency across the grain (along V).
brush_stretch = scalar_param("BrushStretch", 40.0, -1900, 800)   # How elongated the streaks are along U.
brush_rough = scalar_param("BrushRoughness", 0.25, -520, 460)    # How much the streaks mottle the roughness.
brush_nstr = scalar_param("BrushNormalStrength", 0.35, -1500, 900)  # Streak micro-groove strength.
brush_tint = scalar_param("BrushTint", 0.15, -1500, -160)        # Streak two-tone amount on the base colour.

wear_scale = scalar_param("WearScale", 3.0, -1900, 1100)         # Smudge frequency, panel-sized.
wear_rough = scalar_param("WearRoughness", 0.12, -520, 560)      # How much the smudges mottle the roughness.

# ---- Anisotropic value noise: custom HLSL returning the normal and a height ----
# The same smoothstep value noise with an analytic derivative as M_RopeDefault's fibre node, but the
# UV scale is a float2 input, so one node serves both the stretched brush grain and the isotropic
# wear smudge. Being UV-based it stays fixed to the surface, with none of the swimming of
# world-space noise.
def noise_node(x, y):
    n = node(unreal.MaterialExpressionCustom, x, y)
    n.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT4)
    n.set_editor_property("description", "ElevatorMetalNoise")

    def custom_input(name):
        ci = unreal.CustomInput()
        ci.set_editor_property("input_name", name)
        return ci

    n.set_editor_property("inputs", [custom_input("UV"), custom_input("ScaleU"),
                                     custom_input("ScaleV"), custom_input("Strength")])
    n.set_editor_property("code", (
        "float2 P = UV * float2(ScaleU, ScaleV);\n"
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
    return n


uv = node(unreal.MaterialExpressionTextureCoordinate, -2100, 20)

# ---- The brush grain: streaks along U, so a low U frequency and a high V frequency ----
brush_u = node(unreal.MaterialExpressionDivide, -1700, 720)  # BrushScale / BrushStretch
wire(brush_scale, brush_u, "A"); wire(brush_stretch, brush_u, "B")
brush = noise_node(-1000, 800)
wire(uv, brush, "UV")
wire(brush_u, brush, "ScaleU")
wire(brush_scale, brush, "ScaleV")
wire(brush_nstr, brush, "Strength")

brush_n = node(unreal.MaterialExpressionComponentMask, -760, 760)  # The brush groove normal.
brush_n.set_editor_property("r", True); brush_n.set_editor_property("g", True)
brush_n.set_editor_property("b", True); brush_n.set_editor_property("a", False)
wire(brush, brush_n, "")
brush_h = node(unreal.MaterialExpressionComponentMask, -760, 900)  # The brush streak height.
brush_h.set_editor_property("r", False); brush_h.set_editor_property("g", False)
brush_h.set_editor_property("b", False); brush_h.set_editor_property("a", True)
wire(brush, brush_h, "")

# ---- The wear smudge: isotropic and panel-sized; only the height is used ------
wear = noise_node(-1000, 1200)
wire(uv, wear, "UV")
wire(wear_scale, wear, "ScaleU")
wire(wear_scale, wear, "ScaleV")
wear_zero = constant(-1200, 1300, 0.0)  # The wear normal is discarded, so its strength is moot.
wire(wear_zero, wear, "Strength")
wear_h = node(unreal.MaterialExpressionComponentMask, -760, 1240)
wear_h.set_editor_property("r", False); wear_h.set_editor_property("g", False)
wear_h.set_editor_property("b", False); wear_h.set_editor_property("a", True)
wire(wear, wear_h, "")

# ---- BaseColor: Tint * (1 + (brushH - 0.5) * BrushTint) -----------------------
# A subtle streak two-tone. On a metal the base colour is the reflectance tint, so the variation is
# kept small — the roughness and normal carry the brushed read; this only stops the tint being flat.
half = constant(-1180, -60, 0.5)
bh_c = node(unreal.MaterialExpressionSubtract, -1000, -80)
wire(brush_h, bh_c, "A"); wire(half, bh_c, "B")
bh_amt = node(unreal.MaterialExpressionMultiply, -840, -100)
wire(bh_c, bh_amt, "A"); wire(brush_tint, bh_amt, "B")
one = constant(-840, -180, 1.0)
tint_scale = node(unreal.MaterialExpressionAdd, -680, -140)
wire(one, tint_scale, "A"); wire(bh_amt, tint_scale, "B")
basecol = node(unreal.MaterialExpressionMultiply, -300, -240)
wire(tint, basecol, "A"); wire(tint_scale, basecol, "B")
mel.connect_material_property(basecol, "", unreal.MaterialProperty.MP_BASE_COLOR)

# ---- Metallic and Anisotropy --------------------------------------------------
mel.connect_material_property(metallic, "", unreal.MaterialProperty.MP_METALLIC)
mel.connect_material_property(aniso, "", unreal.MaterialProperty.MP_ANISOTROPY)

# ---- Roughness: Roughness + (brushH-0.5)*BrushRoughness + (wearH-0.5)*WearRoughness, clamped ----
bh_r = node(unreal.MaterialExpressionSubtract, -360, 420)
wire(brush_h, bh_r, "A"); wire(half, bh_r, "B")
bh_r_amt = node(unreal.MaterialExpressionMultiply, -200, 440)
wire(bh_r, bh_r_amt, "A"); wire(brush_rough, bh_r_amt, "B")
wh_r = node(unreal.MaterialExpressionSubtract, -360, 560)
wire(wear_h, wh_r, "A"); wire(half, wh_r, "B")
wh_r_amt = node(unreal.MaterialExpressionMultiply, -200, 580)
wire(wh_r, wh_r_amt, "A"); wire(wear_rough, wh_r_amt, "B")
rough_brush = node(unreal.MaterialExpressionAdd, -40, 380)
wire(rough_p, rough_brush, "A"); wire(bh_r_amt, rough_brush, "B")
rough_sum = node(unreal.MaterialExpressionAdd, 120, 420)
wire(rough_brush, rough_sum, "A"); wire(wh_r_amt, rough_sum, "B")
# Clamped so the mottling can neither reach a dead mirror nor full chalk.
rough = node(unreal.MaterialExpressionClamp, 280, 420)
rough.set_editor_property("min_default", 0.04)
rough.set_editor_property("max_default", 0.9)
wire(rough_sum, rough, "")
mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)

# ---- Normal: the brush groove normal alone, already unit length from the HLSL ----
mel.connect_material_property(brush_n, "", unreal.MaterialProperty.MP_NORMAL)

# ---- Compile and save ----------------------------------------------------------
mel.recompile_material(mat)
eal.save_asset(FULL)
unreal.log("[DynamicRope] Rebuilt {}".format(FULL))
