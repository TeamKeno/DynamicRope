# -*- coding: utf-8 -*-
#
# DynamicRope default rope material generator (M_RopeDefault)
# ------------------------------------------------------------------------------
# A procedural hemp or jute rope look, expressed from the UVs alone with no texture assets.
#   1) The strand pattern: phase = V * StrandCount + U * TwistTurns.
#      U is the length axis, as the accumulated arc length over the circumference, and V runs around the
#      circumference. The tube builder fills U and V at the same physical scale, so TwistTurns is turns
#      per circumference-length and the density is independent of the rope's length. With StrandCount
#      close to TwistTurns the lay is roughly 45 degrees.
#      - The sine of the phase gives the strand mask, which drives a two-tone base colour, the cavity
#        occlusion and specular falloff in the grooves, and the roughness variation there.
#      - The cosine of the phase perturbs the tangent-space normal, which shades the twist lines, scaled
#        by NormalStrength.
#      - A second level of strands adds the fine yarn grain inside each strand as another normal layer,
#        with a reversed lay. Its amplitude is modulated by the first-level strand mask so it is distinct
#        only on the crests, scaled by SubStrength.
#   2) Irregular fibre roughness: a custom HLSL value noise fixed to the UVs, so it does not swim as the
#      rope moves.
#      - One evaluation returns the fibre normal and a height, shared by the fine normal and the
#        roughness mottling.
#      - Controlled by FiberScale for the frequency, FiberNormalStrength for the bump, and FiberRoughness
#        for the roughness mottling.
#   3) The pull feedback emissive: PullGlow multiplied by PullGlowColor, modulated by the strand mask.
#      - Only the strand crests brighten, which reads as the rope heating along its twist, and looks more
#        like a rope than a uniform glow would.
#      - At runtime URopeWielderComponent drives PullGlow through a dynamic material instance with the
#        arming-to-engagement progress, which exceeds one on engagement.
#        It defaults to 0, so a project not using the feature sees exactly the original look.
# Exposed parameters: Tint / StrandCount / TwistTurns / SubStrandCount / SubTwistTurns / SubStrength /
#                Roughness / NormalStrength / CavityStrength /
#                FiberScale / FiberNormalStrength / FiberRoughness /
#                PullGlow / PullGlowColor
#
# To run: select this file through Tools -> Execute Python Script...
# Safe to re-run: it deletes the existing asset and recreates it at the same path, so it is idempotent.
# The child MI_* instances are neither loaded nor saved here, so their .uasset files are untouched and
# their references to the parent path survive.

import unreal

MAT_DIR = "/DynamicRope/DynamicRope/Materials"
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


# ---- Parameters ---------------------------------------------------------------
tint = node(unreal.MaterialExpressionVectorParameter, -1700, -320)
tint.set_editor_property("parameter_name", "Tint")
tint.set_editor_property("default_value", unreal.LinearColor(0.62, 0.44, 0.24, 1.0))  # A warm hemp tan.

strand = scalar_param("StrandCount", 3.0, -1900, 60)   # Strands around the circumference.
twist = scalar_param("TwistTurns", 3.0, -1900, 160)    # Twist turns per circumference-length, where U is the arc length over the circumference.
rough_p = scalar_param("Roughness", 0.82, -520, 300)
nstr = scalar_param("NormalStrength", 0.55, -900, 560)  # How pronounced the twist line normals are.

fiber_scale = scalar_param("FiberScale", 40.0, -1500, 900)          # Fibre noise frequency.
fiber_nstr = scalar_param("FiberNormalStrength", 0.30, -1500, 1000)  # Fibre micro-bump strength.
fiber_rough = scalar_param("FiberRoughness", 0.15, -520, 460)        # How much the fibres mottle the roughness.
cavity_str = scalar_param("CavityStrength", 0.5, -1500, 1100)        # Strand groove self-shadowing: how much the albedo and specular fall off in the grooves.
sub_strand = scalar_param("SubStrandCount", 12.0, -1900, 260)        # Second level: yarns around the circumference within a strand.
sub_twist = scalar_param("SubTwistTurns", -9.0, -1900, 360)          # Second level: sub-yarn twist, where a negative value reverses the lay.
sub_str = scalar_param("SubStrength", 0.4, -900, 660)                # Second level: sub-yarn normal strength, relative to the first.

# ---- UV split: U along the length, V around the circumference -----------------
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

# ---- The strand mask, from the sine of the phase mapped into 0 to 1 -----------
sine = node(unreal.MaterialExpressionSine, -1180, 20)
sine.set_editor_property("period", 1.0)  # sin(2π*phase)
wire(phase, sine, "")
half = constant(-1180, -110, 0.5)
mask_mul = node(unreal.MaterialExpressionMultiply, -1000, 20)
wire(sine, mask_mul, "A"); wire(half, mask_mul, "B")
mask = node(unreal.MaterialExpressionAdd, -840, 20)
wire(mask_mul, mask, "A"); wire(half, mask, "B")

# ---- Strand groove cavity: interpolated so the grooves, where the mask is zero, darken ----
# It imitates the self-shadowing between strands, pushing the base colour down and killing the specular
# so the highlight sits only on the crests.
cav_one = constant(-880, 60, 1.0)
cav_lo = node(unreal.MaterialExpressionSubtract, -700, 120)   # 1 - CavityStrength
wire(cav_one, cav_lo, "A"); wire(cavity_str, cav_lo, "B")
cavity = node(unreal.MaterialExpressionLinearInterpolate, -520, 120)
wire(cav_lo, cavity, "A"); wire(cav_one, cavity, "B"); wire(mask, cavity, "Alpha")

# ---- BaseColor: lerp(Tint*0.5, Tint, mask) × Cavity ---------------------------
darkc = constant(-1360, -280, 0.5)
dark = node(unreal.MaterialExpressionMultiply, -1180, -240)
wire(tint, dark, "A"); wire(darkc, dark, "B")
basecol = node(unreal.MaterialExpressionLinearInterpolate, -300, -180)
wire(dark, basecol, "A"); wire(tint, basecol, "B"); wire(mask, basecol, "Alpha")
basecol_cav = node(unreal.MaterialExpressionMultiply, -120, -180)
wire(basecol, basecol_cav, "A"); wire(cavity, basecol_cav, "B")
mel.connect_material_property(basecol_cav, "", unreal.MaterialProperty.MP_BASE_COLOR)

# ---- Specular: the default multiplied by the cavity, so it falls off in the grooves and the highlight
# ---- concentrates on the crests ------------
spec_base = constant(-300, 40, 0.5)
spec = node(unreal.MaterialExpressionMultiply, -120, 40)
wire(spec_base, spec, "A"); wire(cavity, spec, "B")
mel.connect_material_property(spec, "", unreal.MaterialProperty.MP_SPECULAR)

# ---- The pull feedback emissive: PullGlowColor multiplied by PullGlow and the mask ----------------
# The channel that carries the gameplay pull state onto the rope itself. PullGlow defaults to 0, giving
# no emission, so a project not using the feature sees an unchanged look. Multiplying by the mask
# brightens only the strand crests.
pull_glow = scalar_param("PullGlow", 0.0, -700, 200)
pull_color = node(unreal.MaterialExpressionVectorParameter, -700, 240)
pull_color.set_editor_property("parameter_name", "PullGlowColor")
pull_color.set_editor_property("default_value", unreal.LinearColor(0.25, 1.0, 0.55, 1.0))  # The same tone as the gauge's engaged colour.
glow_amt = node(unreal.MaterialExpressionMultiply, -520, 220)
wire(pull_color, glow_amt, "A"); wire(pull_glow, glow_amt, "B")
glow_masked = node(unreal.MaterialExpressionMultiply, -340, 220)
wire(glow_amt, glow_masked, "A"); wire(mask, glow_masked, "B")
mel.connect_material_property(glow_masked, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)

# ---- Irregular fibre noise: custom HLSL returning the normal and a height ------------------
# A value noise over the scaled UVs with smoothstep interpolation, plus an analytic derivative for the
# tangent-space normal. Being UV-based it stays fixed to the surface as the rope moves, with none of the
# swimming of world-space noise.
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

fiber_n = node(unreal.MaterialExpressionComponentMask, -760, 860)  # The fibre normal.
fiber_n.set_editor_property("r", True); fiber_n.set_editor_property("g", True)
fiber_n.set_editor_property("b", True); fiber_n.set_editor_property("a", False)
wire(fiber, fiber_n, "")
fiber_h = node(unreal.MaterialExpressionComponentMask, -760, 1000)  # The fibre height.
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
# The fibre roughness mottling, centred on zero and scaled by FiberRoughness.
fh_half = constant(-520, 1080, 0.5)
fh_c = node(unreal.MaterialExpressionSubtract, -360, 1000)
wire(fiber_h, fh_c, "A"); wire(fh_half, fh_c, "B")
fh_amt = node(unreal.MaterialExpressionMultiply, -200, 1000)
wire(fh_c, fh_amt, "A"); wire(fiber_rough, fh_amt, "B")
rough = node(unreal.MaterialExpressionAdd, 120, 400)
wire(rough_strand, rough, "A"); wire(fh_amt, rough, "B")
mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)

# ---- Normal: the twist lines, the second-level sub-yarns and the fine fibre normal, whiteout blended --
# The twist line normal perturbs tangent space from the cosine of the phase, along the length and the
# circumference.
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

# ---- Second-level strands, the sub-yarns: the fine grain twisted the opposite way inside each strand --
# The sub-phase uses the sub-strand count and a reversed sub-twist. Its amplitude is modulated by the
# first-level strand mask, so it is distinct on the crests and disappears in the grooves, where it would
# be hidden. It is added as a normal layer alone.
vs_mul = node(unreal.MaterialExpressionMultiply, -1600, 1040)
wire(v, vs_mul, "A"); wire(sub_strand, vs_mul, "B")
us_mul = node(unreal.MaterialExpressionMultiply, -1600, 1160)
wire(u, us_mul, "A"); wire(sub_twist, us_mul, "B")
phase_sub = node(unreal.MaterialExpressionAdd, -1380, 1080)
wire(vs_mul, phase_sub, "A"); wire(us_mul, phase_sub, "B")
cos_sub = node(unreal.MaterialExpressionCosine, -1180, 1080)
cos_sub.set_editor_property("period", 1.0)
wire(phase_sub, cos_sub, "")
# The sub-yarn strength is scaled by the first-level strand mask, which confines the fine grain to the crests.
ns_sub0 = node(unreal.MaterialExpressionMultiply, -1000, 1120)
wire(ns_neg, ns_sub0, "A"); wire(sub_str, ns_sub0, "B")
ns_sub = node(unreal.MaterialExpressionMultiply, -900, 1200)
wire(ns_sub0, ns_sub, "A"); wire(mask, ns_sub, "B")
nx2_s = node(unreal.MaterialExpressionMultiply, -820, 1060)
wire(ns_sub, nx2_s, "A"); wire(sub_twist, nx2_s, "B")
nx2 = node(unreal.MaterialExpressionMultiply, -660, 1080)
wire(cos_sub, nx2, "A"); wire(nx2_s, nx2, "B")
ny2_s = node(unreal.MaterialExpressionMultiply, -820, 1200)
wire(ns_sub, ny2_s, "A"); wire(sub_strand, ny2_s, "B")
ny2 = node(unreal.MaterialExpressionMultiply, -660, 1220)
wire(cos_sub, ny2, "A"); wire(ny2_s, ny2, "B")
onez2 = constant(-660, 1320, 1.0)
sub_app1 = node(unreal.MaterialExpressionAppendVector, -480, 1140)
wire(nx2, sub_app1, "A"); wire(ny2, sub_app1, "B")
sub_app2 = node(unreal.MaterialExpressionAppendVector, -320, 1180)
wire(sub_app1, sub_app2, "A"); wire(onez2, sub_app2, "B")
sub_n = node(unreal.MaterialExpressionNormalize, -160, 1180)
wire(sub_app2, sub_n, "")

# The twist line, sub-yarn and fibre normals are blended as three whiteout layers, summing the tangential
# components and multiplying the vertical ones. Unlike a plain add and normalize, that keeps the vertical
# component from being diluted and all three details survive; it is analogous to blending partial
# derivatives.
sn_xy = node(unreal.MaterialExpressionComponentMask, 60, 700)   # The strand normal's tangential part.
sn_xy.set_editor_property("r", True); sn_xy.set_editor_property("g", True)
sn_xy.set_editor_property("b", False); sn_xy.set_editor_property("a", False)
wire(strand_n, sn_xy, "")
fn_xy = node(unreal.MaterialExpressionComponentMask, 60, 780)   # The fibre normal's tangential part.
fn_xy.set_editor_property("r", True); fn_xy.set_editor_property("g", True)
fn_xy.set_editor_property("b", False); fn_xy.set_editor_property("a", False)
wire(fiber_n, fn_xy, "")
sn_z = node(unreal.MaterialExpressionComponentMask, 60, 860)    # The strand normal's vertical part.
sn_z.set_editor_property("r", False); sn_z.set_editor_property("g", False)
sn_z.set_editor_property("b", True); sn_z.set_editor_property("a", False)
wire(strand_n, sn_z, "")
fn_z = node(unreal.MaterialExpressionComponentMask, 60, 940)    # The fibre normal's vertical part.
fn_z.set_editor_property("r", False); fn_z.set_editor_property("g", False)
fn_z.set_editor_property("b", True); fn_z.set_editor_property("a", False)
wire(fiber_n, fn_z, "")
sub_xy = node(unreal.MaterialExpressionComponentMask, 60, 1020)  # The sub-yarn normal's tangential part.
sub_xy.set_editor_property("r", True); sub_xy.set_editor_property("g", True)
sub_xy.set_editor_property("b", False); sub_xy.set_editor_property("a", False)
wire(sub_n, sub_xy, "")
sub_z = node(unreal.MaterialExpressionComponentMask, 60, 1100)   # The sub-yarn normal's vertical part.
sub_z.set_editor_property("r", False); sub_z.set_editor_property("g", False)
sub_z.set_editor_property("b", True); sub_z.set_editor_property("a", False)
wire(sub_n, sub_z, "")
xy_sum0 = node(unreal.MaterialExpressionAdd, 240, 720)
wire(sn_xy, xy_sum0, "A"); wire(sub_xy, xy_sum0, "B")
xy_sum = node(unreal.MaterialExpressionAdd, 360, 740)
wire(xy_sum0, xy_sum, "A"); wire(fn_xy, xy_sum, "B")
z_mul0 = node(unreal.MaterialExpressionMultiply, 240, 900)
wire(sn_z, z_mul0, "A"); wire(sub_z, z_mul0, "B")
z_mul = node(unreal.MaterialExpressionMultiply, 360, 920)
wire(z_mul0, z_mul, "A"); wire(fn_z, z_mul, "B")
norm_app = node(unreal.MaterialExpressionAppendVector, 480, 820)
wire(xy_sum, norm_app, "A"); wire(z_mul, norm_app, "B")
norm = node(unreal.MaterialExpressionNormalize, 620, 820)
wire(norm_app, norm, "")
mel.connect_material_property(norm, "", unreal.MaterialProperty.MP_NORMAL)

# ---- Compile and save ----------------------------------------------------------
mel.recompile_material(mat)
eal.save_asset(FULL)
unreal.log("[DynamicRope] Rebuilt {}".format(FULL))
