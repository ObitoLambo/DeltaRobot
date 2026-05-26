"""
make_transform_diagram.py
Generates delta_transform_diagram.pdf — visual axis & transform reference.
"""

from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    HRFlowable, KeepTogether
)
from reportlab.graphics.shapes import (
    Drawing, Rect, String, Line, Polygon, Circle, Group
)
from reportlab.graphics import renderPDF
import math, os

OUT = "delta_transform_diagram.pdf"
W, H = A4

# ── palette ──────────────────────────────────────────────────────────────────
BLUE   = colors.HexColor("#1a3a5c")
LBLUE  = colors.HexColor("#2e6da4")
GREEN  = colors.HexColor("#2e7d32")
LGREEN = colors.HexColor("#66bb6a")
ORANGE = colors.HexColor("#e65100")
RED    = colors.HexColor("#c62828")
AMBER  = colors.HexColor("#f57c00")
GREY   = colors.HexColor("#eeeeee")
DGREY  = colors.HexColor("#9e9e9e")
WHITE  = colors.white
PINK   = colors.HexColor("#fce4ec")
CYAN   = colors.HexColor("#0097a7")

# ── styles ───────────────────────────────────────────────────────────────────
SS = getSampleStyleSheet()
def style(name, **kw):
    base = SS[name] if name in SS else SS["Normal"]
    return ParagraphStyle(name + "_c", parent=base, **kw)

T1   = style("Heading1", fontSize=16, spaceAfter=4, textColor=BLUE)
T2   = style("Heading2", fontSize=11, spaceBefore=8, spaceAfter=3, textColor=LBLUE)
BODY = style("Normal",   fontSize=9,  leading=13, spaceAfter=3)
CODE = style("Code",     fontSize=8,  leading=11, fontName="Courier",
             backColor=colors.HexColor("#f5f5f5"), borderPad=3,
             leftIndent=6, rightIndent=6, spaceAfter=5)
CAP  = style("Normal",   fontSize=7.5, alignment=TA_CENTER,
             textColor=DGREY, spaceAfter=6)

def h1(t): return Paragraph(t, T1)
def h2(t): return Paragraph(t, T2)
def p(t):  return Paragraph(t, BODY)
def code(t): return Paragraph(t.replace("\n","<br/>").replace(" ","&nbsp;"), CODE)
def cap(t):  return Paragraph(t, CAP)
def sp(n=6): return Spacer(1, n)
def hr():    return HRFlowable(width="100%", thickness=0.5,
                               color=colors.HexColor("#cccccc"), spaceAfter=5)

def arrow(d, x1, y1, x2, y2, color=BLUE, width=1.2, label="", lx=0, ly=0, dash=None):
    kw = dict(strokeColor=color, strokeWidth=width)
    if dash:
        kw["strokeDashArray"] = dash
    d.add(Line(x1, y1, x2, y2, **kw))
    ang = math.atan2(y2 - y1, x2 - x1)
    al, aw = 3.5*mm, 1.5*mm
    pts = [
        x2, y2,
        x2 - al*math.cos(ang) + aw*math.sin(ang),
        y2 - al*math.sin(ang) - aw*math.cos(ang),
        x2 - al*math.cos(ang) - aw*math.sin(ang),
        y2 - al*math.sin(ang) + aw*math.cos(ang),
    ]
    d.add(Polygon(pts, fillColor=color, strokeColor=None))
    if label:
        d.add(String((x1+x2)/2 + lx, (y1+y2)/2 + ly, label,
                     fontSize=7, textAnchor="middle",
                     fillColor=color, fontName="Helvetica-Bold"))

def param_table(rows, widths=None):
    if widths is None:
        widths = [50*mm, 35*mm, 80*mm]
    t = Table(rows, colWidths=widths)
    t.setStyle(TableStyle([
        ("BACKGROUND",    (0,0), (-1,0),  BLUE),
        ("TEXTCOLOR",     (0,0), (-1,0),  WHITE),
        ("FONTNAME",      (0,0), (-1,0),  "Helvetica-Bold"),
        ("FONTSIZE",      (0,0), (-1,-1), 8.5),
        ("ROWBACKGROUNDS",(0,1), (-1,-1), [WHITE, GREY]),
        ("GRID",          (0,0), (-1,-1), 0.4, DGREY),
        ("VALIGN",        (0,0), (-1,-1), "MIDDLE"),
        ("LEFTPADDING",   (0,0), (-1,-1), 5),
        ("TOPPADDING",    (0,0), (-1,-1), 3),
        ("BOTTOMPADDING", (0,0), (-1,-1), 3),
    ]))
    return t

# ═══════════════════════════════════════════════════════════════════════════
# Figure 1 — Robot base frame + belt layout (top view)
# ═══════════════════════════════════════════════════════════════════════════
def fig_base_frame():
    DW, DH = 165*mm, 110*mm
    d = Drawing(DW, DH)

    cx, cy = 82*mm, 58*mm   # robot base origin

    # ── belt (rectangle, runs along Y axis) ──────────────────────────────
    bw = 40*mm   # belt width (X direction)
    d.add(Rect(cx - bw/2, 8*mm, bw, 90*mm,
               fillColor=colors.HexColor("#d7ccc8"),
               strokeColor=DGREY, strokeWidth=0.6))
    d.add(String(cx + bw/2 + 2*mm, 30*mm, "conveyor belt",
                 fontSize=7, fillColor=DGREY, textAnchor="start"))

    # ── zone bands ───────────────────────────────────────────────────────
    y_lim = 20*mm   # ±Y_LIMIT scaled
    # approach
    d.add(Rect(cx - bw/2, cy + y_lim, bw, 90*mm - (cy + y_lim) + 8*mm,
               fillColor=colors.HexColor("#fff8e1"),
               strokeColor=None))
    # workspace
    d.add(Rect(cx - bw/2, cy - y_lim, bw, 2*y_lim,
               fillColor=colors.HexColor("#e8f5e9"),
               strokeColor=GREEN, strokeWidth=0.8))
    # exit
    d.add(Rect(cx - bw/2, 8*mm, bw, cy - y_lim - 8*mm,
               fillColor=colors.HexColor("#ffebee"),
               strokeColor=None))

    # zone labels
    d.add(String(cx - bw/2 - 18*mm, cy + y_lim + 8*mm, "APPROACH",
                 fontSize=7, fillColor=AMBER, fontName="Helvetica-Bold", textAnchor="middle"))
    d.add(String(cx - bw/2 - 18*mm, cy, "WORKSPACE",
                 fontSize=7, fillColor=GREEN, fontName="Helvetica-Bold", textAnchor="middle"))
    d.add(String(cx - bw/2 - 18*mm, cy - y_lim - 8*mm, "EXIT",
                 fontSize=7, fillColor=RED, fontName="Helvetica-Bold", textAnchor="middle"))

    # bracket for workspace
    d.add(Line(cx - bw/2 - 22*mm, cy + y_lim, cx - bw/2 - 20*mm, cy + y_lim,
               strokeColor=DGREY, strokeWidth=0.6))
    d.add(Line(cx - bw/2 - 22*mm, cy - y_lim, cx - bw/2 - 20*mm, cy - y_lim,
               strokeColor=DGREY, strokeWidth=0.6))
    d.add(Line(cx - bw/2 - 22*mm, cy - y_lim, cx - bw/2 - 22*mm, cy + y_lim,
               strokeColor=DGREY, strokeWidth=0.6))
    d.add(String(cx - bw/2 - 26*mm, cy - 1*mm, "±150mm",
                 fontSize=6, fillColor=DGREY, textAnchor="middle"))

    # belt motion arrow
    arrow(d, cx + bw/2 + 3*mm, cy + 15*mm, cx + bw/2 + 3*mm, cy - 15*mm,
          color=AMBER, width=1.5, label="belt direction", lx=10*mm, ly=0)

    # ── robot base origin ────────────────────────────────────────────────
    d.add(Circle(cx, cy, 3*mm, fillColor=BLUE, strokeColor=None))
    d.add(String(cx + 3.5*mm, cy + 2*mm, "Base origin (0,0,0)",
                 fontSize=7, fillColor=BLUE, fontName="Helvetica-Bold"))

    # ── axes ─────────────────────────────────────────────────────────────
    ax_len = 22*mm
    # +X
    arrow(d, cx, cy, cx + ax_len, cy, color=RED, width=1.8)
    d.add(String(cx + ax_len + 1*mm, cy - 1*mm, "+X",
                 fontSize=8.5, fillColor=RED, fontName="Helvetica-Bold"))
    # +Y
    arrow(d, cx, cy, cx, cy + ax_len, color=GREEN, width=1.8)
    d.add(String(cx - 1*mm, cy + ax_len + 1*mm, "+Y",
                 fontSize=8.5, fillColor=GREEN, fontName="Helvetica-Bold",
                 textAnchor="middle"))
    # +Z (out of page — dot)
    d.add(Circle(cx - ax_len - 2*mm, cy, 4*mm,
                 fillColor=WHITE, strokeColor=BLUE, strokeWidth=1))
    d.add(Circle(cx - ax_len - 2*mm, cy, 1.5*mm, fillColor=BLUE, strokeColor=None))
    d.add(String(cx - ax_len - 2*mm, cy + 5.5*mm, "+Z (up)",
                 fontSize=6.5, fillColor=BLUE, textAnchor="middle"))

    # ── pick Z annotation ────────────────────────────────────────────────
    d.add(String(cx + bw/2 + 2*mm, cy - 3*mm,
                 "Z = -410 mm\n(belt surface)",
                 fontSize=6.5, fillColor=DGREY))

    # Y_LIMIT dashed lines
    d.add(Line(cx - bw/2, cy + y_lim, cx + bw/2, cy + y_lim,
               strokeColor=GREEN, strokeWidth=0.6, strokeDashArray=[3,2]))
    d.add(Line(cx - bw/2, cy - y_lim, cx + bw/2, cy - y_lim,
               strokeColor=RED, strokeWidth=0.6, strokeDashArray=[3,2]))

    d.add(String(cx, 2*mm, "Fig 1 — Top view: robot base frame & belt zones  (Y_LIMIT = ±150 mm, X_LIMIT = ±150 mm)",
                 fontSize=6.5, textAnchor="middle", fillColor=DGREY))

    return d


# ═══════════════════════════════════════════════════════════════════════════
# Figure 2 — Camera frame + image frame
# ═══════════════════════════════════════════════════════════════════════════
def fig_camera_frame():
    DW, DH = 165*mm, 90*mm
    d = Drawing(DW, DH)

    # ── camera frame (left side) ─────────────────────────────────────────
    cx, cy = 40*mm, 52*mm
    ax = 18*mm
    d.add(String(cx, DH - 4*mm, "CAMERA FRAME  (p_cam)",
                 fontSize=8, fontName="Helvetica-Bold",
                 fillColor=CYAN, textAnchor="middle"))

    # camera body
    d.add(Rect(cx - 7*mm, cy - 5*mm, 14*mm, 10*mm,
               fillColor=colors.HexColor("#e0f7fa"),
               strokeColor=CYAN, strokeWidth=0.8))
    d.add(String(cx, cy, "CAM", fontSize=6.5, textAnchor="middle",
                 fillColor=CYAN, fontName="Helvetica-Bold"))
    # lens
    d.add(Circle(cx, cy - 6*mm, 2.5*mm,
                 fillColor=colors.HexColor("#b2ebf2"),
                 strokeColor=CYAN, strokeWidth=0.6))

    # x_cam (+right)
    arrow(d, cx, cy, cx + ax, cy, color=RED, width=1.5)
    d.add(String(cx + ax + 1*mm, cy - 1*mm, "+x_cam",
                 fontSize=7.5, fillColor=RED, fontName="Helvetica-Bold"))
    # y_cam (+down in image)
    arrow(d, cx, cy, cx, cy - ax, color=GREEN, width=1.5)
    d.add(String(cx, cy - ax - 4*mm, "+y_cam\n(↓ in image)",
                 fontSize=7, fillColor=GREEN, textAnchor="middle"))
    # z_cam (into scene, downward physically)
    arrow(d, cx, cy, cx - ax*0.7, cy - ax*0.7, color=BLUE, width=1.5)
    d.add(String(cx - ax*0.7 - 1*mm, cy - ax*0.7 - 3*mm, "+z_cam\n(into scene)",
                 fontSize=7, fillColor=BLUE, textAnchor="middle"))

    # ── rotation matrix ──────────────────────────────────────────────────
    mx = 85*mm
    d.add(String(mx, DH - 4*mm, "ROTATION  (CAMERA_DIRECT_MATRIX)",
                 fontSize=8, fontName="Helvetica-Bold",
                 fillColor=BLUE, textAnchor="middle"))

    lines = [
        "x_base = -1 · x_cam",
        "y_base = +1 · y_cam",
        "z_base = -1 · z_cam",
    ]
    colors_r = [RED, GREEN, BLUE]
    for i, (ln, col) in enumerate(zip(lines, colors_r)):
        d.add(String(mx, cy + 10*mm - i*8*mm, ln,
                     fontSize=8.5, fillColor=col,
                     fontName="Courier", textAnchor="middle"))

    # matrix bracket
    bx, by = mx - 28*mm, cy - 10*mm
    bh = 28*mm
    for sx in (-1, 1):
        d.add(Line(bx + sx*28*mm, by, bx + sx*28*mm + sx*2*mm, by,
                   strokeColor=DGREY, strokeWidth=0.8))
        d.add(Line(bx + sx*28*mm, by, bx + sx*28*mm, by + bh,
                   strokeColor=DGREY, strokeWidth=0.8))
        d.add(Line(bx + sx*28*mm, by + bh, bx + sx*28*mm + sx*2*mm, by + bh,
                   strokeColor=DGREY, strokeWidth=0.8))

    mat = [
        "[-1,  0,  0]",
        "[ 0, +1,  0]",
        "[ 0,  0, -1]",
    ]
    for i, row in enumerate(mat):
        d.add(String(mx, by + bh - 8*mm - i*9*mm, row,
                     fontSize=7.5, fillColor=DGREY,
                     fontName="Courier", textAnchor="middle"))

    # ── translation ──────────────────────────────────────────────────────
    tx = 145*mm
    d.add(String(tx, DH - 4*mm, "TRANSLATION  t",
                 fontSize=8, fontName="Helvetica-Bold",
                 fillColor=ORANGE, textAnchor="middle"))

    trans = [
        ("CAM_TX_MM =    0.0", "mm", RED),
        ("CAM_TY_MM = -300.0", "mm", GREEN),
        ("CAM_TZ_MM =  352.0", "mm", BLUE),
    ]
    for i, (val, unit, col) in enumerate(trans):
        d.add(String(tx, cy + 10*mm - i*8*mm, val,
                     fontSize=7.5, fillColor=col,
                     fontName="Courier", textAnchor="middle"))

    d.add(String(DW/2, 2*mm,
                 "Fig 2 — Camera frame axes, rotation matrix, and translation vector",
                 fontSize=6.5, textAnchor="middle", fillColor=DGREY))
    return d


# ═══════════════════════════════════════════════════════════════════════════
# Figure 3 — Image frame (pixel coords)
# ═══════════════════════════════════════════════════════════════════════════
def fig_image_frame():
    DW, DH = 165*mm, 95*mm
    d = Drawing(DW, DH)

    # camera image rectangle
    iw, ih = 80*mm, 60*mm
    ix, iy = 15*mm, 18*mm

    # zone fills
    zone_h = ih / 3
    # exit (top)
    d.add(Rect(ix, iy + 2*zone_h, iw, zone_h,
               fillColor=colors.HexColor("#ffebee"), strokeColor=None))
    # workspace (middle)
    d.add(Rect(ix, iy + zone_h, iw, zone_h,
               fillColor=colors.HexColor("#e8f5e9"), strokeColor=None))
    # approach (bottom)
    d.add(Rect(ix, iy, iw, zone_h,
               fillColor=colors.HexColor("#fff8e1"), strokeColor=None))

    # image border
    d.add(Rect(ix, iy, iw, ih,
               fillColor=None, strokeColor=BLUE, strokeWidth=1.2))

    # zone labels
    d.add(String(ix + iw/2, iy + 2*zone_h + zone_h/2 - 2*mm,
                 "EXIT  (y_base < -150mm)", fontSize=7,
                 fillColor=RED, fontName="Helvetica-Bold", textAnchor="middle"))
    d.add(String(ix + iw/2, iy + zone_h + zone_h/2 - 2*mm,
                 "ROBOT WORKSPACE", fontSize=7,
                 fillColor=GREEN, fontName="Helvetica-Bold", textAnchor="middle"))
    d.add(String(ix + iw/2, iy + zone_h/2 - 2*mm,
                 "APPROACH  (y_base > +150mm)", fontSize=7,
                 fillColor=AMBER, fontName="Helvetica-Bold", textAnchor="middle"))

    # dashed zone dividers
    d.add(Line(ix, iy + zone_h, ix + iw, iy + zone_h,
               strokeColor=AMBER, strokeWidth=0.6, strokeDashArray=[3,2]))
    d.add(Line(ix, iy + 2*zone_h, ix + iw, iy + 2*zone_h,
               strokeColor=RED, strokeWidth=0.6, strokeDashArray=[3,2]))

    # image corner (0,0)
    d.add(String(ix - 1*mm, iy + ih + 1*mm, "(0,0)",
                 fontSize=6.5, fillColor=BLUE, textAnchor="end"))
    # u axis
    arrow(d, ix, iy + ih + 6*mm, ix + 28*mm, iy + ih + 6*mm,
          color=RED, width=1.5, label="+u  (→ pixel col)", lx=8*mm, ly=3*mm)
    # v axis
    arrow(d, ix - 6*mm, iy + ih, ix - 6*mm, iy + ih - 22*mm,
          color=GREEN, width=1.5, label="+v  (↓ pixel row)", lx=-14*mm, ly=0)

    d.add(String(ix + iw + 2*mm, iy + ih - 2*mm, "640×480 px",
                 fontSize=6.5, fillColor=DGREY))

    # projection formula on right
    fx = ix + iw + 6*mm
    fy_start = iy + ih - 4*mm
    d.add(String(fx, fy_start, "Pinhole projection:", fontSize=7.5,
                 fillColor=BLUE, fontName="Helvetica-Bold"))
    formulas = [
        ("u = fx · x_cam/z_cam + cx", RED),
        ("v = fy · y_cam/z_cam + cy", GREEN),
        ("z_cam = -z_base + CAM_TZ_MM", BLUE),
        ("     = -(-410) + 352 = 762 mm", DGREY),
    ]
    for i, (f, col) in enumerate(formulas):
        d.add(String(fx, fy_start - 9*mm - i*8*mm, f,
                     fontSize=7.2, fillColor=col, fontName="Courier"))

    d.add(String(DW/2, 2*mm,
                 "Fig 3 — Image pixel frame and zone mapping  (exit=top, approach=bottom)",
                 fontSize=6.5, textAnchor="middle", fillColor=DGREY))
    return d


# ═══════════════════════════════════════════════════════════════════════════
# Figure 4 — Full transform chain (flowchart)
# ═══════════════════════════════════════════════════════════════════════════
def fig_transform_chain():
    DW, DH = 165*mm, 105*mm
    d = Drawing(DW, DH)

    cx = DW / 2
    bw, bh = 50*mm, 11*mm
    gap = 16*mm
    steps = [
        ("pixel (u, v)",                    WHITE,  BLUE,   "Camera image coordinate"),
        ("p_cam = [x_cam, y_cam, z_cam]",   colors.HexColor("#e0f7fa"), CYAN,   "Pinhole inverse + depth"),
        ("p_base = [x_base, y_base, z_base]",colors.HexColor("#e8f5e9"),GREEN, "× T_cam_to_base (R + t)"),
        ("pick_xyz (corrected)",             colors.HexColor("#fff8e1"),AMBER,  "+ EE_OFFSET_[X,Y,Z]_MM"),
        ("θ₁, θ₂, θ₃  (motor angles)",      colors.HexColor("#fce4ec"),RED,   "Inverse kinematics"),
        ("Robot moves to object",            colors.HexColor("#ede7f6"),colors.HexColor("#6a1b9a"), "CAN → motors"),
    ]

    top = DH - 12*mm
    for i, (label, fill, stroke, note) in enumerate(steps):
        y = top - i * gap
        # box
        d.add(Rect(cx - bw/2, y - bh/2, bw, bh, rx=2*mm, ry=2*mm,
                   fillColor=fill, strokeColor=stroke, strokeWidth=1.2))
        d.add(String(cx, y - 0.5*mm, label,
                     fontSize=7, textAnchor="middle",
                     fillColor=stroke, fontName="Helvetica-Bold"))
        # annotation on right
        d.add(String(cx + bw/2 + 3*mm, y - 0.5*mm, note,
                     fontSize=6.5, fillColor=DGREY, textAnchor="start"))
        # arrow to next
        if i < len(steps) - 1:
            arrow(d, cx, y - bh/2, cx, y - gap + bh/2,
                  color=DGREY, width=0.8)

    d.add(String(DW/2, 2*mm,
                 "Fig 4 — Full transform chain: pixel → 3D base frame → corrected pick → IK → motor move",
                 fontSize=6.5, textAnchor="middle", fillColor=DGREY))
    return d


# ═══════════════════════════════════════════════════════════════════════════
# Build PDF
# ═══════════════════════════════════════════════════════════════════════════
doc = SimpleDocTemplate(
    OUT, pagesize=A4,
    leftMargin=20*mm, rightMargin=20*mm,
    topMargin=18*mm, bottomMargin=18*mm,
    title="Delta Robot — Axis & Transform Reference",
    author="Delta WS"
)

story = []

# title
story.append(Paragraph(
    '<font color="#1a3a5c"><b>Delta Robot — Axis &amp; Transform Reference</b></font>',
    ParagraphStyle("title", fontSize=20, alignment=TA_CENTER, spaceAfter=2,
                   fontName="Helvetica-Bold", textColor=BLUE)
))
story.append(Paragraph(
    "Coordinate frames · Camera transform · Image zones · Tuning parameters",
    ParagraphStyle("sub", fontSize=10, alignment=TA_CENTER, spaceAfter=4, textColor=DGREY)
))
story.append(hr())
story.append(sp(4))

# ── Section 1: Base frame ─────────────────────────────────────────────────
story.append(h1("1  Robot Base Frame & Belt Zones"))
story.append(p(
    "All robot coordinates use the <b>base frame</b>: origin at robot arm mount, "
    "<b>+Y toward belt entry (approach)</b>, <b>+X right</b>, <b>+Z up</b>. "
    "Belt surface is at <b>Z = −410 mm</b>. Workspace bounds: |X| ≤ 150 mm, |Y| ≤ 150 mm."
))
story.append(sp(3))
f1 = fig_base_frame()
story.append(Table([[f1]], colWidths=[165*mm]))
story.append(sp(4))

# ── Section 2: Camera frame ───────────────────────────────────────────────
story.append(h1("2  Camera Frame & Transform"))
story.append(p(
    "The camera is mounted above the belt looking down. "
    "<b>CAMERA_DIRECT_MATRIX</b> maps camera axes to base axes. "
    "<b>CAM_TY_MM = −300</b> means the camera is 300 mm toward the EXIT side from the base origin."
))
story.append(sp(3))
f2 = fig_camera_frame()
story.append(Table([[f2]], colWidths=[165*mm]))
story.append(sp(4))

story.append(h2("Transform equation  (T_cam_to_base)"))
story.append(code(
    "p_base = R @ p_cam + t\n\n"
    "x_base = -x_cam + CAM_TX_MM   (0.0)\n"
    "y_base = +y_cam + CAM_TY_MM   (-300.0 mm)\n"
    "z_base = -z_cam + CAM_TZ_MM   (352.0 mm)"
))

# ── Section 3: Image frame ────────────────────────────────────────────────
story.append(h1("3  Image Frame & Zone Mapping"))
story.append(p(
    "Pixel <b>v increases downward</b>. Approach zone (high +y_base) appears at the "
    "<b>bottom</b> of the image; exit zone (negative y_base) at the <b>top</b>. "
    "The workspace overlay is projected from base frame corners (±X_LIMIT, ±Y_LIMIT) at WORKSPACE_PICK_Z_MM."
))
story.append(sp(3))
f3 = fig_image_frame()
story.append(Table([[f3]], colWidths=[165*mm]))
story.append(sp(4))

# ── Section 4: Transform chain ────────────────────────────────────────────
story.append(h1("4  Full Transform Chain"))
story.append(sp(3))
f4 = fig_transform_chain()
story.append(Table([[f4]], colWidths=[165*mm]))
story.append(sp(4))

# ── Section 5: Tuning table ───────────────────────────────────────────────
story.append(h1("5  Tuning Parameters"))
rows = [
    ["Parameter", "Current Value", "What it affects", "How to tune"],
    ["CAM_TY_MM",       "-300.0 mm",
     "3D y_base coords\n+ overlay Y position",
     "Place obj at Y=0, read printed Y,\nset = -printed_Y"],
    ["CAM_TZ_MM",       "352.0 mm",
     "3D z_base coords\n(depth from camera)",
     "Measure camera height above\nbelt surface: h_cam + |PLACE_Z|"],
    ["EE_OFFSET_X_MM",  "+50.0 mm",
     "Shifts pick X BEFORE IK",
     "EE overshoots +Xmm → set to -X"],
    ["EE_OFFSET_Y_MM",  "+15.0 mm",
     "Shifts pick Y BEFORE IK",
     "EE overshoots +Ymm → set to -Y"],
    ["WORKSPACE_OVERLAY\n_Y_OFFSET_MM", "0.0 mm",
     "Visual overlay ONLY\n(no robot effect)",
     "Adjust until green box aligns\nwith physical workspace"],
    ["ROBOT_SPEED_MM_S", "500.0 mm/s",
     "Y prediction for\nmoving belt pick",
     "EE before obj → increase\nEE after obj → decrease"],
    ["CONVEYOR_APPROACH\n_WAIT_SEC", "0.3 s",
     "Hold at approach Z\nbefore descending",
     "Increase if EE still arrives\nbefore object"],
]
story.append(param_table(rows, [38*mm, 25*mm, 50*mm, 52*mm]))
story.append(sp(6))

story.append(hr())
story.append(Paragraph(
    "config.py  ·  camera_system.py  ·  main_app.py  ·  fk_ik.py",
    ParagraphStyle("foot", fontSize=7, alignment=TA_CENTER,
                   textColor=DGREY, fontName="Courier")
))

doc.build(story)
print(f"Saved: {os.path.abspath(OUT)}")
