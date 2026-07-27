# -*- coding: utf-8 -*-
#
# DynamicRope preset gate "portal curtain" material generator (M_GatePortal)
# ------------------------------------------------------------------------------
# A translucent energy curtain to stand in the opening of a preset volume gate. It is built from the UVs
# and procedural noise alone, with no texture assets, the same way M_RopeDefault is. It is two-sided, so
# it looks the same after walking through the gate and turning round.
#
# The look is five layers added together, each of which can be disabled by setting its strength to zero.
#   1) The flowing energy grain: a custom HLSL two-octave value noise is scrolled upwards and passed
#      through a ridge transform, raising one minus the folded height to a power, which turns it into
#      thin filaments. Instead of the whole surface glowing uniformly, fine filaments flow across it.
#      The analytic derivative of the same noise also produces a tangent normal, used by the second
#      layer so the sheen slides along that grain.
#   2) The sheen: only the combination of a translucent blend mode, the default lit shading model and
#      surface forward shading translucency gives a translucent surface real specular and reflections;
#      the volumetric modes have no specular at all. A specular of 1 and a low roughness, together with
#      the normal from the first layer, give the sliding highlight characteristic of glass or an energy
#      field.
#      Enabling pixel normal offset refraction additionally makes the background shimmer along that
#      normal.
#   3) The Fresnel rim, computed directly from the magnitude of the dot product of the camera vector and
#      the vertex normal. Taking the magnitude is the key point: the material is two-sided, so the sign
#      of that dot product flips when viewed from behind, and using the Fresnel node directly would make
#      the rim disappear on one side.
#   4) The pass-through sweep: a Gaussian band travelling periodically along V, which signals that the
#      gate is alive.
#   5) The intersection glow: a depth fade brightens the line where the curtain meets the gate frame and
#      the floor. It hides the hard cut where a translucent surface slices through geometry, and on its
#      own it raises the perceived quality considerably.
#
# Exposed parameters: PortalColor / GlowIntensity / BaseOpacity / OpacityGain /
#                FresnelPower / FresnelIntensity /
#                FlowScale / FlowSpeed / FlowContrast / PatternIntensity / NormalStrength /
#                SweepSpeed / SweepWidth / SweepIntensity /
#                EdgeGlowDistance / EdgeGlowIntensity / Roughness
#
# To run: select this file through Tools -> Execute Python Script...
# Safe to re-run: it deletes the parent and the instances and recreates them at the same paths, so it is
# idempotent.

import unreal

MAT_DIR = "/DynamicRope/Materials"
MAT_NAME = "M_GatePortal"
FULL = MAT_DIR + "/" + MAT_NAME

mel = unreal.MaterialEditingLibrary
eal = unreal.EditorAssetLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()

if eal.does_asset_exist(FULL):
    eal.delete_asset(FULL)

mat = tools.create_asset(MAT_NAME, MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())


def try_set(obj, prop, value, note=""):
    """For properties whose names changed between engine versions. A failure still leaves the rest of the graph intact."""
    try:
        obj.set_editor_property(prop, value)
        return True
    except Exception as e:
        unreal.log_warning("[DynamicRope] failed to set {} ({}) - {}".format(prop, e, note))
        return False


# ---- Material domain settings ---------------------------------------------------
mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
mat.set_editor_property("two_sided", True)          # Two-sided, so it is visible after walking through and turning round.
# The only mode that gives a translucent surface specular and reflections. Without it the sheen disappears entirely.
try_set(mat, "translucency_lighting_mode",
        unreal.TranslucencyLightingMode.TLM_SURFACE_FORWARD_SHADING,
        "translucent specular will be missing; set Surface ForwardShading by hand in the material editor")

# Refraction, optional: pixel normal offset makes the background shimmer from the normal input alone, so
# no pin needs connecting.
# The property and enum names differ between engine versions, so this tries the known spellings
# permissively.
_refraction_done = False
for _prop in ("refraction_method", "refraction_mode"):
    if _refraction_done:
        break
    for _enum_name in ("RefractionMode", "RefractionMethod"):
        _enum = getattr(unreal, _enum_name, None)
        if _enum is None:
            continue
        for _val_name in ("RM_PIXEL_NORMAL_OFFSET", "REFRACTION_MODE_PIXEL_NORMAL_OFFSET"):
            _val = getattr(_enum, _val_name, None)
            if _val is not None and try_set(mat, _prop, _val, "continuing without refraction"):
                _refraction_done = True
                break
        if _refraction_done:
            break


def node(cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)


def wire(a, b, b_in, a_out=""):
    mel.connect_material_expressions(a, a_out, b, b_in)


def constant(x, y, v):
    c = node(unreal.MaterialExpressionConstant, x, y)
    c.set_editor_property("r", float(v))
    return c


def scalar_param(name, default, x, y, group="Portal"):
    p = node(unreal.MaterialExpressionScalarParameter, x, y)
    p.set_editor_property("parameter_name", name)
    p.set_editor_property("default_value", float(default))
    p.set_editor_property("group", group)
    return p


def custom_input(name):
    ci = unreal.CustomInput()
    ci.set_editor_property("input_name", name)
    return ci


# ---- Parameters ---------------------------------------------------------------
portal_color = node(unreal.MaterialExpressionVectorParameter, -1900, -420)
portal_color.set_editor_property("parameter_name", "PortalColor")
portal_color.set_editor_property("default_value", unreal.LinearColor(0.10, 0.62, 1.0, 1.0))  # Cyan.
portal_color.set_editor_property("group", "Portal")

glow_int = scalar_param("GlowIntensity", 6.0, -1900, -300)      # Total emissive amount, which drives the bloom.
base_op = scalar_param("BaseOpacity", 0.16, -1900, -220)        # The opacity where no effect is present.
op_gain = scalar_param("OpacityGain", 0.35, -1900, -140)        # How much brighter areas also become more opaque.

fres_pow = scalar_param("FresnelPower", 3.0, -1900, 40, "Portal|Fresnel")
fres_int = scalar_param("FresnelIntensity", 1.3, -1900, 120, "Portal|Fresnel")

flow_scale = scalar_param("FlowScale", 6.0, -1900, 640, "Portal|Flow")        # How fine the grain is.
flow_speed = scalar_param("FlowSpeed", 0.35, -1900, 720, "Portal|Flow")       # How fast it flows.
flow_contrast = scalar_param("FlowContrast", 6.0, -1900, 800, "Portal|Flow")  # Higher values give thinner filaments.
pattern_int = scalar_param("PatternIntensity", 0.9, -1900, 880, "Portal|Flow")
normal_str = scalar_param("NormalStrength", 0.35, -1900, 960, "Portal|Flow")  # The relief the sheen rides on.
sweep_speed = scalar_param("SweepSpeed", 0.25, -1900, 1200, "Portal|Sweep")   # Sweeps per second.
sweep_width = scalar_param("SweepWidth", 0.10, -1900, 1280, "Portal|Sweep")   # Band width, as a fraction of V.
sweep_int = scalar_param("SweepIntensity", 2.2, -1900, 1360, "Portal|Sweep")

edge_dist = scalar_param("EdgeGlowDistance", 28.0, -1900, 1560, "Portal|Edge")  # cm
edge_int = scalar_param("EdgeGlowIntensity", 2.0, -1900, 1640, "Portal|Edge")

rough_p = scalar_param("Roughness", 0.08, -1900, 1800, "Portal|Surface")

# ---- UV ------------------------------------------------------------------------
uv = node(unreal.MaterialExpressionTextureCoordinate, -2140, 620)
v_mask = node(unreal.MaterialExpressionComponentMask, -1960, 1120)   # V, the vertical axis, used by the sweep.
v_mask.set_editor_property("r", False); v_mask.set_editor_property("g", True)
v_mask.set_editor_property("b", False); v_mask.set_editor_property("a", False)
wire(uv, v_mask, "")

time_n = node(unreal.MaterialExpressionTime, -2140, 760)

# ---- (1) The flowing energy grain: custom HLSL returning the normal and the ridge pattern -------------
# Two octaves of value noise are scrolled in opposite directions so no static pattern forms, and a ridge
# transform raised to the contrast power extracts thin filaments. The analytic derivative also produces
# the tangent normal of the same grain, which the sheen uses.
flow = node(unreal.MaterialExpressionCustom, -1560, 700)
flow.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT4)
flow.set_editor_property("description", "GatePortalFlow")
flow.set_editor_property("inputs", [custom_input("UV"), custom_input("T"), custom_input("Scale"),
                                    custom_input("Speed"), custom_input("Contrast"),
                                    custom_input("Strength")])
flow.set_editor_property("code", (
    "float h = 0.0, dx = 0.0, dy = 0.0, amp = 1.0, norm = 0.0;\n"
    "float2 flow = float2(0.0, -T * Speed);\n"
    "[unroll]\n"
    "for (int oct = 0; oct < 2; ++oct)\n"
    "{\n"
    "    float freq = (oct == 0) ? 1.0 : 2.3;\n"
    "    float dir  = (oct == 0) ? 1.0 : -0.55;\n"
    "    float2 P = UV * Scale * freq + flow * dir;\n"
    "    float2 I = floor(P);\n"
    "    float2 F = frac(P);\n"
    "    float2 W = F*F*(3.0-2.0*F);\n"
    "    float h00 = frac(sin(dot(I+float2(0.0,0.0), float2(127.1,311.7)))*43758.5453);\n"
    "    float h10 = frac(sin(dot(I+float2(1.0,0.0), float2(127.1,311.7)))*43758.5453);\n"
    "    float h01 = frac(sin(dot(I+float2(0.0,1.0), float2(127.1,311.7)))*43758.5453);\n"
    "    float h11 = frac(sin(dot(I+float2(1.0,1.0), float2(127.1,311.7)))*43758.5453);\n"
    "    float b = lerp(h00, h10, W.x);\n"
    "    float t = lerp(h01, h11, W.x);\n"
    "    float2 dW = 6.0*F*(1.0-F);\n"
    "    h  += amp * lerp(b, t, W.y);\n"
    "    dx += amp * lerp(h10-h00, h11-h01, W.y) * dW.x;\n"
    "    dy += amp * (t - b) * dW.y;\n"
    "    norm += amp;\n"
    "    amp *= 0.5;\n"
    "}\n"
    "h /= norm; dx /= norm; dy /= norm;\n"
    "float ridge = 1.0 - abs(2.0*h - 1.0);\n"
    "ridge = pow(saturate(ridge), max(Contrast, 1.0));\n"
    "float3 N = normalize(float3(-dx*Strength, -dy*Strength, 1.0));\n"
    "return float4(N, ridge);\n"
))
wire(uv, flow, "UV")
wire(time_n, flow, "T")
wire(flow_scale, flow, "Scale")
wire(flow_speed, flow, "Speed")
wire(flow_contrast, flow, "Contrast")
wire(normal_str, flow, "Strength")

flow_n = node(unreal.MaterialExpressionComponentMask, -1300, 660)   # The normal.
flow_n.set_editor_property("r", True); flow_n.set_editor_property("g", True)
flow_n.set_editor_property("b", True); flow_n.set_editor_property("a", False)
wire(flow, flow_n, "")
flow_p = node(unreal.MaterialExpressionComponentMask, -1300, 780)   # The ridge pattern.
flow_p.set_editor_property("r", False); flow_p.set_editor_property("g", False)
flow_p.set_editor_property("b", False); flow_p.set_editor_property("a", True)
wire(flow, flow_p, "")
mel.connect_material_property(flow_n, "", unreal.MaterialProperty.MP_NORMAL)

pat_term = node(unreal.MaterialExpressionMultiply, -1080, 780)
wire(flow_p, pat_term, "A"); wire(pattern_int, pat_term, "B")

# ---- (3) Fresnel: one minus the magnitude of the camera-to-normal dot product, raised to a power ------
# It is built by hand rather than with the Fresnel node because of that magnitude: the material is
# two-sided, so the sign of the dot product flips when viewed from behind, and folding it with the
# magnitude keeps the rim identical from either side.
cam_v = node(unreal.MaterialExpressionCameraVectorWS, -1560, 60)
vtx_n = node(unreal.MaterialExpressionVertexNormalWS, -1560, 160)
ndv = node(unreal.MaterialExpressionDotProduct, -1360, 100)
wire(cam_v, ndv, "A"); wire(vtx_n, ndv, "B")
ndv_abs = node(unreal.MaterialExpressionAbs, -1200, 100)
wire(ndv, ndv_abs, "")
fres_raw = node(unreal.MaterialExpressionOneMinus, -1060, 100)
wire(ndv_abs, fres_raw, "")
fres_pow_n = node(unreal.MaterialExpressionPower, -900, 100)
wire(fres_raw, fres_pow_n, "Base"); wire(fres_pow, fres_pow_n, "Exponent")
fres_term = node(unreal.MaterialExpressionMultiply, -740, 100)
wire(fres_pow_n, fres_term, "A"); wire(fres_int, fres_term, "B")

# ---- (4) The pass-through sweep: a Gaussian band cycling along V --------------------------
sweep = node(unreal.MaterialExpressionCustom, -1560, 1200)
sweep.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT1)
sweep.set_editor_property("description", "GatePortalSweep")
sweep.set_editor_property("inputs", [custom_input("V"), custom_input("T"),
                                     custom_input("Speed"), custom_input("Width")])
sweep.set_editor_property("code", (
    "float phase = frac(T * Speed);\n"
    "float d = V - phase;\n"
    "d = d - floor(d + 0.5);\n"          # The cyclic nearest distance, which joins the top and bottom.
    "float w = max(Width, 1e-4);\n"
    "return exp(-(d*d)/(w*w));\n"
))
wire(v_mask, sweep, "V")
wire(time_n, sweep, "T")
wire(sweep_speed, sweep, "Speed")
wire(sweep_width, sweep, "Width")
sweep_term = node(unreal.MaterialExpressionMultiply, -1300, 1240)
wire(sweep, sweep_term, "A"); wire(sweep_int, sweep_term, "B")

# ---- (5) The intersection glow: a depth fade brightens where it meets the frame and the floor ---------
# The depth fade is zero at the intersection, so it is inverted to make "brightest where they meet".
depth_fade = node(unreal.MaterialExpressionDepthFade, -1560, 1560)
depth_fade.set_editor_property("opacity_default", 1.0)
wire(edge_dist, depth_fade, "FadeDistance")
edge_raw = node(unreal.MaterialExpressionOneMinus, -1340, 1560)
wire(depth_fade, edge_raw, "")
edge_term = node(unreal.MaterialExpressionMultiply, -1180, 1600)
wire(edge_raw, edge_term, "A"); wire(edge_int, edge_term, "B")

# ---- Summing the effects into the emissive and the opacity ------------------------------------
sum1 = node(unreal.MaterialExpressionAdd, -560, 400)
wire(fres_term, sum1, "A"); wire(pat_term, sum1, "B")
sum2 = node(unreal.MaterialExpressionAdd, -420, 500)
wire(sum1, sum2, "A"); wire(sweep_term, sum2, "B")
sum3 = node(unreal.MaterialExpressionAdd, -280, 600)   # The total effect amount.
wire(sum2, sum3, "A"); wire(edge_term, sum3, "B")

# Emissive = PortalColor * GlowIntensity * (the effect amount plus a floor glow)
floor_glow = constant(-280, 720, 0.12)
glow_sum = node(unreal.MaterialExpressionAdd, -140, 640)
wire(sum3, glow_sum, "A"); wire(floor_glow, glow_sum, "B")
col_x_int = node(unreal.MaterialExpressionMultiply, -140, -360)
wire(portal_color, col_x_int, "A"); wire(glow_int, col_x_int, "B")
emissive = node(unreal.MaterialExpressionMultiply, 60, -300)
wire(col_x_int, emissive, "A"); wire(glow_sum, emissive, "B")
mel.connect_material_property(emissive, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)

# Opacity = the base opacity plus the effect amount scaled by the gain, clamped, so brighter is more opaque.
op_scaled = node(unreal.MaterialExpressionMultiply, -140, 900)
wire(sum3, op_scaled, "A"); wire(op_gain, op_scaled, "B")
op_sum = node(unreal.MaterialExpressionAdd, 20, 860)
wire(base_op, op_sum, "A"); wire(op_scaled, op_sum, "B")
op_clamp = node(unreal.MaterialExpressionClamp, 180, 860)   # Defaults to a range of 0 to 1.
wire(op_sum, op_clamp, "Input")
mel.connect_material_property(op_clamp, "", unreal.MaterialProperty.MP_OPACITY)

# ---- The surface sheen: a nearly black base colour, full specular and a low roughness ----------------
# An energy field should read as a highlight sitting on the surface rather than as diffuse reflection. The
# base colour is pushed down so lighting does not muddy the colour, and the sheen is produced by the
# specular, roughness and normal alone.
bc_scale = constant(-140, -180, 0.04)
basecol = node(unreal.MaterialExpressionMultiply, 60, -180)
wire(portal_color, basecol, "A"); wire(bc_scale, basecol, "B")
mel.connect_material_property(basecol, "", unreal.MaterialProperty.MP_BASE_COLOR)

spec = constant(60, -80, 1.0)
mel.connect_material_property(spec, "", unreal.MaterialProperty.MP_SPECULAR)
mel.connect_material_property(rough_p, "", unreal.MaterialProperty.MP_ROUGHNESS)

mel.recompile_material(mat)
eal.save_asset(FULL)
unreal.log("[DynamicRope] Rebuilt {}".format(FULL))


# ---- Per-preset colour instances -------------------------------------------------
# Instances differing only in colour, to pair with the demo's preset gates.
# name -> (PortalColor RGB, GlowIntensity, FlowContrast)
INSTANCES = {
    "MI_GatePortal_FreeSim":  ((0.10, 0.62, 1.00), 6.0, 6.0),    # Cyan, for free simulation.
    "MI_GatePortal_Capture":  ((1.00, 0.48, 0.08), 6.5, 5.0),    # Amber, for capture.
    "MI_GatePortal_Grapple":  ((0.18, 1.00, 0.45), 6.0, 7.0),    # Green, for the grappling hook.
    "MI_GatePortal_Danger":   ((1.00, 0.12, 0.18), 7.0, 4.0),    # Red, for warnings and reverting.
}


def make_instance(name, rgb, glow, contrast):
    path = MAT_DIR + "/" + name
    if eal.does_asset_exist(path):
        eal.delete_asset(path)

    mic = tools.create_asset(name, MAT_DIR, unreal.MaterialInstanceConstant,
                             unreal.MaterialInstanceConstantFactoryNew())
    mic.set_editor_property("parent", mat)

    r, g, b = rgb
    mel.set_material_instance_vector_parameter_value(mic, "PortalColor", unreal.LinearColor(r, g, b, 1.0))
    mel.set_material_instance_scalar_parameter_value(mic, "GlowIntensity", glow)
    mel.set_material_instance_scalar_parameter_value(mic, "FlowContrast", contrast)

    mel.update_material_instance(mic)
    eal.save_asset(path)
    unreal.log("[DynamicRope] Created {}".format(path))


for inst_name, inst_args in INSTANCES.items():
    make_instance(inst_name, *inst_args)

unreal.log("[DynamicRope] {} gate portal instance(s) created.".format(len(INSTANCES)))
