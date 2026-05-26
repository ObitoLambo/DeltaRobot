"""
make_pdf.py — Generate delta_blind_pick_place_doc.pdf
Explains the full Blind Pick-and-Place system:
  • ROS 2 node (blind_pick_place.py)
  • Simulink / Stateflow model (delta_blind_pick_place_sim.m / .slx)
"""

from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_JUSTIFY
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    HRFlowable, KeepTogether, PageBreak
)
from reportlab.graphics.shapes import (
    Drawing, Rect, String, Line, Polygon, Circle,
    Path, Group
)
from reportlab.graphics.charts.lineplots import LinePlot
from reportlab.graphics import renderPDF
import math, os

OUT = "delta_blind_pick_place_doc.pdf"
W, H = A4

# ─── styles ──────────────────────────────────────────────────────────────────
SS = getSampleStyleSheet()

def style(name, **kw):
    base = SS[name] if name in SS else SS["Normal"]
    return ParagraphStyle(name + "_custom", parent=base, **kw)

T1  = style("Heading1", fontSize=18, spaceAfter=6, textColor=colors.HexColor("#1a3a5c"))
T2  = style("Heading2", fontSize=13, spaceBefore=10, spaceAfter=4,
            textColor=colors.HexColor("#1a3a5c"), borderPad=2)
T3  = style("Heading3", fontSize=11, spaceBefore=8, spaceAfter=2,
            textColor=colors.HexColor("#2e6da4"))
BODY = style("Normal", fontSize=9.5, leading=14, alignment=TA_JUSTIFY, spaceAfter=4)
CODE = style("Code",   fontSize=8.2,  leading=12, fontName="Courier",
             backColor=colors.HexColor("#f5f5f5"), borderPad=4,
             leftIndent=8, rightIndent=8, spaceAfter=6)
NOTE = style("Normal", fontSize=8.5, leading=12, textColor=colors.HexColor("#555555"),
             leftIndent=12, spaceAfter=4)
CAP  = style("Normal", fontSize=8, leading=10, alignment=TA_CENTER,
             textColor=colors.grey, spaceAfter=8)

def h1(t): return Paragraph(t, T1)
def h2(t): return Paragraph(t, T2)
def h3(t): return Paragraph(t, T3)
def p(t):  return Paragraph(t, BODY)
def code(t): return Paragraph(t.replace("\n","<br/>").replace(" ", "&nbsp;"), CODE)
def note(t): return Paragraph(f"<i>{t}</i>", NOTE)
def cap(t):  return Paragraph(t, CAP)
def sp(n=6): return Spacer(1, n)
def hr():    return HRFlowable(width="100%", thickness=0.5,
                               color=colors.HexColor("#cccccc"), spaceAfter=6)

# ─── colour palette ───────────────────────────────────────────────────────────
BLUE   = colors.HexColor("#1a3a5c")
LBLUE  = colors.HexColor("#2e6da4")
CYAN   = colors.HexColor("#0097a7")
GREEN  = colors.HexColor("#2e7d32")
ORANGE = colors.HexColor("#e65100")
RED    = colors.HexColor("#c62828")
GREY   = colors.HexColor("#eeeeee")
DGREY  = colors.HexColor("#9e9e9e")
WHITE  = colors.white

STATE_COLORS = {
    "IDLE":             colors.HexColor("#e3f2fd"),
    "MOVING_TO_PICK":  colors.HexColor("#fff9c4"),
    "GRASPING":         colors.HexColor("#fce4ec"),
    "MOVING_TO_PLACE": colors.HexColor("#e8f5e9"),
    "DROPPING":         colors.HexColor("#fff3e0"),
    "RESETTING":        colors.HexColor("#f3e5f5"),
}
STATE_BORDER = {
    "IDLE":             colors.HexColor("#1565c0"),
    "MOVING_TO_PICK":  colors.HexColor("#f9a825"),
    "GRASPING":         colors.HexColor("#ad1457"),
    "MOVING_TO_PLACE": colors.HexColor("#2e7d32"),
    "DROPPING":         colors.HexColor("#e65100"),
    "RESETTING":        colors.HexColor("#6a1b9a"),
}

# ─── FSM diagram ──────────────────────────────────────────────────────────────
def fsm_diagram():
    """Draw the 6-state FSM on a 160 × 120 mm canvas."""
    DW, DH = 160*mm, 120*mm
    d = Drawing(DW, DH)

    # state boxes: (cx, cy, label, key)
    SW, SH = 34*mm, 14*mm
    states = [
        (25*mm,  90*mm, "IDLE",             "IDLE"),
        (80*mm,  90*mm, "MOVING\nTO PICK",  "MOVING_TO_PICK"),
        (135*mm, 90*mm, "GRASPING",         "GRASPING"),
        (135*mm, 30*mm, "MOVING\nTO PLACE", "MOVING_TO_PLACE"),
        (80*mm,  30*mm, "DROPPING",         "DROPPING"),
        (25*mm,  30*mm, "RESETTING",        "RESETTING"),
    ]

    for cx, cy, lbl, key in states:
        x, y = cx - SW/2, cy - SH/2
        # shadow
        d.add(Rect(x+1.2*mm, y-1.2*mm, SW, SH,
                   fillColor=colors.HexColor("#cccccc"), strokeColor=None))
        # box
        d.add(Rect(x, y, SW, SH, rx=3*mm, ry=3*mm,
                   fillColor=STATE_COLORS[key],
                   strokeColor=STATE_BORDER[key], strokeWidth=1.2))
        lines = lbl.split("\n")
        for i, ln in enumerate(lines):
            yoff = (len(lines)-1)*3.5*mm/2 - i*3.5*mm
            d.add(String(cx, cy + yoff - 1.5*mm, ln,
                         fontSize=7.5, textAnchor="middle",
                         fillColor=STATE_BORDER[key], fontName="Helvetica-Bold"))

    def arrow(x1,y1,x2,y2, label="", col=DGREY, lbl_dx=0, lbl_dy=0):
        d.add(Line(x1,y1,x2,y2, strokeColor=col, strokeWidth=0.8))
        # arrowhead
        ang = math.atan2(y2-y1, x2-x1)
        al = 4*mm
        aw = 2*mm
        pts = [
            x2, y2,
            x2 - al*math.cos(ang) + aw*math.sin(ang),
            y2 - al*math.sin(ang) - aw*math.cos(ang),
            x2 - al*math.cos(ang) - aw*math.sin(ang),
            y2 - al*math.sin(ang) + aw*math.cos(ang),
        ]
        d.add(Polygon(pts, fillColor=col, strokeColor=None))
        if label:
            mx, my = (x1+x2)/2 + lbl_dx, (y1+y2)/2 + lbl_dy
            d.add(String(mx, my, label, fontSize=6, textAnchor="middle",
                         fillColor=BLUE, fontName="Helvetica-Oblique"))

    # IDLE → MOVING_TO_PICK
    arrow(25*mm+SW/2, 90*mm, 80*mm-SW/2, 90*mm,
          "[trigger ↑ & !busy]", LBLUE, lbl_dy=2.5*mm)

    # MOVING_TO_PICK → GRASPING
    arrow(80*mm+SW/2, 90*mm, 135*mm-SW/2, 90*mm,
          "[traj_done]", GREEN, lbl_dy=2.5*mm)

    # GRASPING → MOVING_TO_PLACE (down)
    arrow(135*mm, 90*mm-SH/2, 135*mm, 30*mm+SH/2,
          "[settle ✓]", CYAN, lbl_dx=9*mm)

    # MOVING_TO_PLACE → DROPPING
    arrow(135*mm-SW/2, 30*mm, 80*mm+SW/2, 30*mm,
          "[traj_done]", GREEN, lbl_dy=2.5*mm)

    # DROPPING → RESETTING
    arrow(80*mm-SW/2, 30*mm, 25*mm+SW/2, 30*mm,
          "[settle ✓]", CYAN, lbl_dy=2.5*mm)

    # RESETTING → IDLE (up)
    arrow(25*mm, 30*mm+SH/2, 25*mm, 90*mm-SH/2,
          "[traj_done]", ORANGE, lbl_dx=-10*mm)

    # error: MOVING_TO_PICK → RESETTING (diagonal)
    arrow(80*mm-SW/2*0.5, 90*mm-SH/2,
          25*mm+SW/2*0.5, 30*mm+SH/2,
          "[traj_failed]", RED, lbl_dx=-8*mm, lbl_dy=2*mm)

    # error: MOVING_TO_PLACE → RESETTING (left)
    arrow(135*mm-SW/2*0.5, 30*mm-SH/2*0.5,
          25*mm+SW/2*0.5, 30*mm-SH/2*0.5,
          "[traj_failed]", RED, lbl_dy=-3*mm)

    # default transition dot
    d.add(Circle(25*mm-SW/2-5*mm, 90*mm, 2.5*mm,
                 fillColor=BLUE, strokeColor=None))
    arrow(25*mm-SW/2-5*mm+2.5*mm, 90*mm, 25*mm-SW/2, 90*mm, col=BLUE)

    return d


# ─── trapezoidal profile diagram ─────────────────────────────────────────────
def trapz_diagram():
    """Draw a velocity vs time trapezoidal profile (70 × 45 mm)."""
    DW, DH = 100*mm, 55*mm
    d = Drawing(DW, DH)

    ox, oy = 12*mm, 8*mm
    pw, ph = 75*mm, 35*mm

    # axes
    d.add(Line(ox, oy, ox, oy+ph+4*mm,
               strokeColor=BLUE, strokeWidth=0.8))
    d.add(Line(ox, oy, ox+pw+4*mm, oy,
               strokeColor=BLUE, strokeWidth=0.8))
    d.add(String(ox-2*mm, oy+ph+4*mm, "v", fontSize=8,
                 textAnchor="end", fillColor=BLUE, fontName="Helvetica-Bold"))
    d.add(String(ox+pw+5*mm, oy-1*mm, "t", fontSize=8,
                 textAnchor="start", fillColor=BLUE, fontName="Helvetica-Bold"))

    # profile
    ta = 0.22; tc = 0.56; T = 1.0
    pts = [
        ox, oy,
        ox + ta*pw, oy + ph,
        ox + (ta+tc)*pw, oy + ph,
        ox + T*pw, oy,
    ]
    d.add(Polygon(pts + [ox+T*pw, oy, ox, oy],
                  fillColor=colors.HexColor("#dce8f5"), strokeColor=None))
    d.add(Line(pts[0], pts[1], pts[2], pts[3],
               strokeColor=LBLUE, strokeWidth=1.5))
    d.add(Line(pts[2], pts[3], pts[4], pts[5],
               strokeColor=LBLUE, strokeWidth=1.5))
    d.add(Line(pts[4], pts[5], pts[6], pts[7],
               strokeColor=LBLUE, strokeWidth=1.5))

    # annotations
    def vline(x, label, col=DGREY):
        d.add(Line(ox+x*pw, oy, ox+x*pw, oy+ph,
                   strokeColor=col, strokeWidth=0.5,
                   strokeDashArray=[2,2]))
        d.add(String(ox+x*pw, oy-3.5*mm, label, fontSize=7,
                     textAnchor="middle", fillColor=col))

    vline(ta,       "t_a",   ORANGE)
    vline(ta+tc,    "T-t_a", ORANGE)
    vline(T,        "T",     BLUE)

    d.add(String(ox+pw*0.5, oy+ph+2*mm, "v_max",
                 fontSize=7, textAnchor="middle",
                 fillColor=GREEN, fontName="Helvetica-Bold"))
    d.add(Line(ox, oy+ph, ox+pw*(ta+tc), oy+ph,
               strokeColor=GREEN, strokeWidth=0.5, strokeDashArray=[2,2]))

    # phase labels
    d.add(String(ox+pw*(ta/2),          oy+ph*0.5, "accel", fontSize=6.5,
                 textAnchor="middle", fillColor=ORANGE))
    d.add(String(ox+pw*(ta+tc/2),       oy+ph*0.5, "cruise", fontSize=6.5,
                 textAnchor="middle", fillColor=GREEN))
    d.add(String(ox+pw*(ta+tc+(1-ta-tc)/2), oy+ph*0.5, "decel", fontSize=6.5,
                 textAnchor="middle", fillColor=ORANGE))

    return d


# ─── transit-Z path diagram ───────────────────────────────────────────────────
def transit_diagram():
    """Top-view schematic of the transit-Z routing strategy (100 × 70 mm)."""
    DW, DH = 110*mm, 75*mm
    d = Drawing(DW, DH)

    cx, cy = 55*mm, 37*mm   # centre (HOME)
    R = 28*mm               # workspace radius circle

    d.add(Circle(cx, cy, R, fillColor=colors.HexColor("#f0f4f8"),
                 strokeColor=DGREY, strokeWidth=0.6, strokeDashArray=[3,3]))
    d.add(Circle(cx, cy, 2*mm, fillColor=BLUE, strokeColor=None))
    d.add(String(cx+2.5*mm, cy-1*mm, "HOME (0,0)", fontSize=7,
                 fillColor=BLUE, fontName="Helvetica-Bold"))

    # pick point (top-left quadrant)
    px, py = cx - 20*mm, cy + 18*mm
    d.add(Circle(px, py, 2*mm, fillColor=ORANGE, strokeColor=None))
    d.add(String(px+3*mm, py, "pick", fontSize=7, fillColor=ORANGE,
                 fontName="Helvetica-Bold"))

    # place point (bottom-right quadrant)
    qx, qy = cx + 22*mm, cy - 16*mm
    d.add(Circle(qx, qy, 2*mm, fillColor=GREEN, strokeColor=None))
    d.add(String(qx+3*mm, qy, "place", fontSize=7, fillColor=GREEN,
                 fontName="Helvetica-Bold"))

    def seg(x1,y1,x2,y2,col,dash=None):
        kw = dict(strokeColor=col, strokeWidth=1.2)
        if dash: kw["strokeDashArray"] = dash
        d.add(Line(x1,y1,x2,y2,**kw))
        ang = math.atan2(y2-y1,x2-x1)
        al,aw = 4*mm, 1.5*mm
        pts = [x2,y2,
               x2-al*math.cos(ang)+aw*math.sin(ang),
               y2-al*math.sin(ang)-aw*math.cos(ang),
               x2-al*math.cos(ang)-aw*math.sin(ang),
               y2-al*math.sin(ang)+aw*math.cos(ang)]
        d.add(Polygon(pts, fillColor=col, strokeColor=None))

    # MOVING_TO_PICK path:  HOME → home_t (Z only, vertical) → pick_t (lateral) → pick
    # We show X-Y projection (Z move is "depth" — just a label)
    # Step 1: HOME → pick_t (lateral at transit Z)
    seg(cx, cy, px, cy, LBLUE)   # HOME_xy → pick_xy (at transit Z)
    # Step 2: pick_t → pick (descend)
    seg(px, cy, px, py, ORANGE)

    # MOVING_TO_PLACE path: pick → pick_t → home_t → place_t → place
    seg(px, py, px, cy, ORANGE, [3,2])         # rise
    seg(px, cy, cx, cy, LBLUE,  [3,2])         # → home col
    seg(cx, cy, qx, cy, GREEN)                 # home_t → place_t
    seg(qx, cy, qx, qy, GREEN)                 # → place (descend)

    # transit Z line (dashed horizontal guide)
    d.add(Line(10*mm, cy, 100*mm, cy, strokeColor=DGREY,
               strokeWidth=0.5, strokeDashArray=[4,3]))
    d.add(String(3*mm, cy-1*mm, "transit Z\n(≤-400)", fontSize=6,
                 fillColor=DGREY))

    # legend
    items = [
        (LBLUE,  "HOME→pick lateral"),
        (ORANGE, "pick descent / ascent"),
        (GREEN,  "→ place lateral + descent"),
    ]
    for i,(col,lbl) in enumerate(items):
        lx, ly = 4*mm, 18*mm - i*6*mm
        d.add(Line(lx, ly+1*mm, lx+8*mm, ly+1*mm,
                   strokeColor=col, strokeWidth=1.5))
        d.add(String(lx+10*mm, ly, lbl, fontSize=6.5, fillColor=col))

    return d


# ─── parameter tables ─────────────────────────────────────────────────────────
def param_table(rows, col_widths=None):
    if col_widths is None:
        col_widths = [45*mm, 30*mm, 85*mm]
    tbl = Table(rows, colWidths=col_widths)
    tbl.setStyle(TableStyle([
        ("BACKGROUND",  (0,0), (-1,0),  BLUE),
        ("TEXTCOLOR",   (0,0), (-1,0),  WHITE),
        ("FONTNAME",    (0,0), (-1,0),  "Helvetica-Bold"),
        ("FONTSIZE",    (0,0), (-1,-1), 8.5),
        ("ROWBACKGROUNDS", (0,1), (-1,-1), [WHITE, GREY]),
        ("GRID",        (0,0), (-1,-1), 0.4, DGREY),
        ("VALIGN",      (0,0), (-1,-1), "MIDDLE"),
        ("LEFTPADDING", (0,0), (-1,-1), 5),
        ("TOPPADDING",  (0,0), (-1,-1), 3),
        ("BOTTOMPADDING",(0,0),(-1,-1), 3),
    ]))
    return tbl


# ═══════════════════════════════════════════════════════════════════════════════
# Build document
# ═══════════════════════════════════════════════════════════════════════════════
doc = SimpleDocTemplate(
    OUT, pagesize=A4,
    leftMargin=22*mm, rightMargin=22*mm,
    topMargin=22*mm, bottomMargin=22*mm,
    title="Delta Robot Blind Pick-and-Place",
    author="Delta WS"
)

story = []

# ── title ──────────────────────────────────────────────────────────────────────
story.append(sp(8))
story.append(Paragraph(
    '<font color="#1a3a5c"><b>Delta Robot — Blind Pick-and-Place</b></font>',
    ParagraphStyle("title", fontSize=22, alignment=TA_CENTER, spaceAfter=4,
                   fontName="Helvetica-Bold", textColor=BLUE)
))
story.append(Paragraph(
    "How It Works: ROS 2 Node &amp; Simulink / Stateflow Model",
    ParagraphStyle("sub", fontSize=12, alignment=TA_CENTER, spaceAfter=2,
                   textColor=DGREY)
))
story.append(Paragraph(
    "blind_pick_place.py  ·  delta_blind_pick_place_sim.m",
    ParagraphStyle("files", fontSize=9, alignment=TA_CENTER, spaceAfter=6,
                   textColor=DGREY, fontName="Courier")
))
story.append(hr())
story.append(sp(4))

# ── 1. Overview ────────────────────────────────────────────────────────────────
story.append(h1("1  Overview"))
story.append(p(
    "Blind pick-and-place moves a delta robot from a known <b>pick position</b> to a known "
    "<b>place position</b> without any camera feedback.  The end-effector path is planned "
    "entirely in software using a <b>trapezoidal velocity profile</b>, and the gripper is a "
    "pneumatic solenoid valve commanded over CAN bus."
))
story.append(p(
    "The same logic exists in two forms.  The <b>ROS&nbsp;2 node</b> "
    "(<font name='Courier'>blind_pick_place.py</font>) runs on the physical robot. "
    "The <b>Simulink / Stateflow model</b> "
    "(<font name='Courier'>delta_blind_pick_place.slx</font>) simulates the identical "
    "sequence entirely in MATLAB — same IK, same trajectory planner, same FSM states — "
    "so you can verify timing, tune speed presets, and inspect motor angles before touching "
    "hardware."
))
story.append(sp(4))

# ── 2. System architecture ─────────────────────────────────────────────────────
story.append(h1("2  System Architecture"))
story.append(h2("2.1  ROS 2 node"))
story.append(p(
    "The node <font name='Courier'>BlindPickAndPlace</font> owns three collaborating objects:"
))
arch_rows = [
    ["Component", "Class", "Role"],
    ["Motor controller", "DeltaMotorController", "3 × RS-00 motors via CAN (can0, IDs 1–3)"],
    ["Pneumatic gripper", "PneumaticGripper",     "Solenoid valve via CAN (ID 0x04)"],
    ["Pick sequence",     "Thread (FSM)",          "Non-blocking pick-and-place state machine"],
]
story.append(param_table(arch_rows, [45*mm, 45*mm, 70*mm]))
story.append(sp(6))

story.append(h2("2.2  Simulink model"))
story.append(p(
    "The <font name='Courier'>.slx</font> model generated by "
    "<font name='Courier'>delta_blind_pick_place_sim.m</font> contains:"
))
sim_rows = [
    ["Block",               "Type",                "Purpose"],
    ["BlindPickPlace_FSM",  "Stateflow Chart",     "Full FSM — all 6 states, IK calls, trajectory stepping"],
    ["Trigger",             "Pulse Generator",     "Simulates /delta/trigger_pick service call"],
    ["pick_x/y/z",          "Constant ×3",         "Pick position in mm (default: 0, 80, −380)"],
    ["place_x/y/z",         "Constant ×3",         "Place position in mm (default: 80, 0, −380)"],
    ["SimTime",             "Clock",               "Simulation time → gripper settle timer"],
    ["AngleMux + Scopes",   "Mux + Scope ×3",      "Display state_id, θ₁/θ₂/θ₃ (rad), gripper"],
]
story.append(param_table(sim_rows, [38*mm, 38*mm, 84*mm]))
story.append(sp(4))

# ── 3. FSM ─────────────────────────────────────────────────────────────────────
story.append(PageBreak())
story.append(h1("3  Finite State Machine"))
story.append(p(
    "The pick-and-place sequence is a six-state Moore machine. "
    "One cycle executes per trigger event.  While a cycle is running the <b>busy</b> flag "
    "blocks re-triggering.  An IK failure in any motion state causes an immediate jump to "
    "RESETTING and opens the gripper as a safety action."
))
story.append(sp(4))

fsm_d = fsm_diagram()
fsm_tbl = Table([[fsm_d]], colWidths=[160*mm])
fsm_tbl.setStyle(TableStyle([
    ("ALIGN",   (0,0),(0,0),"CENTER"),
    ("VALIGN",  (0,0),(0,0),"MIDDLE"),
    ("BOX",     (0,0),(0,0), 0.5, DGREY),
    ("BACKGROUND",(0,0),(0,0), colors.HexColor("#fafafa")),
]))
story.append(fsm_tbl)
story.append(cap("Figure 1 — Blind pick-and-place FSM.  Blue = normal flow, orange/red = error paths."))
story.append(sp(6))

state_rows = [
    ["State",           "state_id", "Entry action",                         "Exit condition"],
    ["IDLE",            "0",        "busy=0, gripper open, reset flags",     "trigger rising edge & !busy"],
    ["MOVING_TO_PICK",  "1",        "Build 3-seg traj HOME→pick, wp_idx=1",  "traj_done (all waypoints sent)"],
    ["GRASPING",        "2",        "gripper_out=1, start 0.5 s timer",      "sim_t ≥ settle_timer"],
    ["MOVING_TO_PLACE", "3",        "Build 4-seg traj pick→place, wp_idx=1", "traj_done"],
    ["DROPPING",        "4",        "gripper_out=0, start 0.5 s timer",      "sim_t ≥ settle_timer"],
    ["RESETTING",       "5",        "gripper_out=0, build 3-seg traj→HOME",  "traj_done"],
]
story.append(param_table(state_rows, [36*mm, 17*mm, 68*mm, 49*mm]))
story.append(sp(4))

# ── 4. Transit-Z routing ───────────────────────────────────────────────────────
story.append(h1("4  Transit-Z Waypoint Routing"))
story.append(p(
    "The delta robot workspace is <b>non-convex near the ceiling</b>: a straight diagonal "
    "from pick to place can exit the reachable sphere.  The solution — used identically in "
    "the ROS 2 node and the Simulink model — is to route every motion through the "
    "<b>centre column (X=0, Y=0)</b> at a <b>safe transit depth</b>."
))

trz_rows = [
    ["Variable",        "Value",          "Meaning"],
    ["SAFE_TRANSIT_Z",  "−400 mm",        "Z where workspace radius ≥ 200 mm in all directions"],
    ["transit_z",       "min(pick_z, place_z, −400)", "Chosen per cycle so both endpoints are above it"],
    ["home_t",          "(0, 0, transit_z)",  "Centre column at transit depth"],
    ["pick_t",          "(pick_x, pick_y, transit_z)", "Directly above pick point"],
    ["place_t",         "(place_x, place_y, transit_z)", "Directly above place point"],
]
story.append(param_table(trz_rows, [38*mm, 52*mm, 70*mm]))
story.append(sp(6))

trd = transit_diagram()
trd_tbl = Table([[trd]], colWidths=[110*mm])
trd_tbl.setStyle(TableStyle([
    ("ALIGN",(0,0),(0,0),"CENTER"),
    ("BOX",  (0,0),(0,0), 0.5, DGREY),
    ("BACKGROUND",(0,0),(0,0), colors.HexColor("#fafafa")),
]))

path_tbl = Table([
    [Paragraph("<b>MOVING_TO_PICK</b>  (3 segments)", BODY),
     Paragraph("HOME → <i>home_t</i> → <i>pick_t</i> → pick_xyz", BODY)],
    [Paragraph("<b>MOVING_TO_PLACE</b>  (4 segments)", BODY),
     Paragraph("pick_xyz → <i>pick_t</i> → <i>home_t</i> → <i>place_t</i> → place_xyz", BODY)],
    [Paragraph("<b>RESETTING</b>  (3 segments)", BODY),
     Paragraph("last_pos → <i>cur_t</i> → <i>home_t</i> → HOME(0,0,−350)", BODY)],
], colWidths=[52*mm, 108*mm])
path_tbl.setStyle(TableStyle([
    ("ROWBACKGROUNDS",(0,0),(-1,-1),[WHITE, GREY]),
    ("GRID",(0,0),(-1,-1),0.4,DGREY),
    ("LEFTPADDING",(0,0),(-1,-1),5),
    ("TOPPADDING",(0,0),(-1,-1),3),
    ("BOTTOMPADDING",(0,0),(-1,-1),3),
    ("FONTSIZE",(0,0),(-1,-1),9),
]))

story.append(Table([[trd_tbl, path_tbl]], colWidths=[115*mm, 45*mm]))
story.append(cap("Figure 2 — XY projection of the transit-Z path (left). Path breakdown per motion state (right)."))
story.append(sp(4))

# ── 5. Trajectory planning ─────────────────────────────────────────────────────
story.append(PageBreak())
story.append(h1("5  Trapezoidal Trajectory Planning"))
story.append(p(
    "Each straight-line segment is sampled with a <b>symmetric trapezoidal velocity profile</b> "
    "at a fixed time step (<i>dt</i> = 0.05 s, 20 Hz).  "
    "If the segment is too short to reach <i>v_max</i>, the profile degrades gracefully to a "
    "triangle (no cruise phase)."
))
story.append(sp(4))

tzd = trapz_diagram()
tz_tbl = Table([[tzd]], colWidths=[100*mm])
tz_tbl.setStyle(TableStyle([
    ("ALIGN",(0,0),(0,0),"CENTER"),
    ("BOX",  (0,0),(0,0), 0.5, DGREY),
    ("BACKGROUND",(0,0),(0,0), colors.HexColor("#fafafa")),
]))
story.append(tz_tbl)
story.append(cap("Figure 3 — Trapezoidal velocity profile.  Triangle profile (no cruise) used for short segments."))
story.append(sp(6))

story.append(h2("5.1  Profile equations"))
story.append(p("Acceleration phase duration and distance:"))
story.append(code(
    "t_a = v_max / a_max\n"
    "d_a = 0.5 * a_max * t_a²"
))
story.append(p("If 2·d_a ≥ distance  →  triangle profile (peak speed limited):"))
story.append(code(
    "t_a    = sqrt(distance / a_max)\n"
    "v_peak = a_max * t_a\n"
    "T      = 2 · t_a"
))
story.append(p("Otherwise  →  trapezoidal profile:"))
story.append(code(
    "v_peak   = v_max\n"
    "t_cruise = (distance - 2·d_a) / v_max\n"
    "T        = 2·t_a + t_cruise"
))
story.append(p("Cumulative arc-length at time <i>t</i>:"))
story.append(code(
    "if t ≤ t_a              :  s = 0.5 · a_max · t²\n"
    "if t_a < t ≤ T - t_a   :  s = d_a + v_peak · (t - t_a)\n"
    "if t > T - t_a          :  s = distance - 0.5 · a_max · (T - t)²"
))

story.append(h2("5.2  Speed presets  (from blind_pick_place.py)"))
preset_rows = [
    ["Preset",   "v_max (mm/s)", "a_max (mm/s²)", "Motor ω (rad/s)", "Motor α (rad/s²)"],
    ["low",      "80",           "200",            "1.0",             "2.0"],
    ["medium",   "2000",         "5000",            "25.0",            "50.0"],
    ["max",      "4000",         "10000",           "50.0",            "100.0"],
]
story.append(param_table(preset_rows, [28*mm, 30*mm, 32*mm, 35*mm, 35*mm]))
story.append(note(
    "The Simulink model defaults to the 'low' preset (v=80 mm/s, a=200 mm/s²).  "
    "Change the constants in the entry actions of any motion state to switch presets."
))
story.append(sp(4))

# ── 6. Delta IK/FK ─────────────────────────────────────────────────────────────
story.append(PageBreak())
story.append(h1("6  Delta Robot Kinematics"))
story.append(h2("6.1  Geometry constants"))
geom_rows = [
    ["Symbol", "Value (mm)", "Meaning"],
    ["e",      "35",         "End-effector equilateral triangle side"],
    ["f",      "157",        "Base equilateral triangle side"],
    ["re",     "400",        "Forearm (distal link) length"],
    ["rf",     "200",        "Upper arm (proximal link) length"],
]
story.append(param_table(geom_rows, [28*mm, 28*mm, 104*mm]))
story.append(sp(6))

story.append(h2("6.2  Inverse kinematics  (xyz → θ₁, θ₂, θ₃)"))
story.append(p(
    "Each arm angle is solved in its own YZ-plane after rotating the XY plane by 0°, −120°, "
    "+120° for arms 1, 2, 3.  The sub-problem for arm 1 (others identical after rotation):"
))
story.append(code(
    "y1 = -0.5 · tan(30°) · f\n"
    "y0 ← y0 - 0.5 · tan(30°) · e\n\n"
    "a  = (x0² + y0² + z0² + rf² - re² - y1²) / (2·z0)\n"
    "b  = (y1 - y0) / z0\n"
    "d  = -(a + b·y1)² + rf·(b²·rf + rf)     ← discriminant\n\n"
    "yj = (y1 - a·b - sqrt(d)) / (b² + 1)\n"
    "zj = a + b·yj\n"
    "θ  = atan2d(-zj, y1 - yj)               ← degrees"
))
story.append(p(
    "If <i>d</i> &lt; 0 the point is unreachable for that arm → status = −1.  "
    "All three arms must return status = 0 and θ > 0 for the solution to be valid."
))
story.append(sp(4))

story.append(h2("6.3  Workspace limits  (from config.py)"))
ws_rows = [
    ["Limit",   "Value",       "Note"],
    ["X_LIMIT", "±151.563 mm", "Bounding box half-width in X"],
    ["Y_LIMIT", "±151.563 mm", "Bounding box half-width in Y"],
    ["Z_MIN",   "−500 mm",     "Maximum depth"],
    ["Z_MAX",   "−323 mm",     "Ceiling (IK fails above this)"],
]
story.append(param_table(ws_rows, [28*mm, 38*mm, 94*mm]))
story.append(note(
    "The bounding box is a rectangular over-approximation.  "
    "check_ws() also runs IK to reject corners of the box that are outside the reachable sphere."
))
story.append(sp(4))

# ── 7. Simulink / Stateflow implementation ─────────────────────────────────────
story.append(PageBreak())
story.append(h1("7  Simulink / Stateflow Implementation"))
story.append(h2("7.1  Chart configuration"))
cfg_rows = [
    ["Setting",              "Value",             "Reason"],
    ["ActionLanguage",       "MATLAB",            "Allows full MATLAB syntax in state actions"],
    ["StateMachineType",     "Classic",           "Moore machine — outputs set on entry"],
    ["ChartUpdate",          "DISCRETE",          "Runs at fixed sample time, no continuous solver"],
    ["SampleTime",           "0.05 s  (20 Hz)",   "Matches trajectory dt — one waypoint per step"],
]
story.append(param_table(cfg_rows, [45*mm, 38*mm, 77*mm]))
story.append(sp(6))

story.append(h2("7.2  Chart data"))
data_rows = [
    ["Name",         "Type",   "Scope",  "Description"],
    ["trigger",      "double", "Input",  "1 = trigger active (rising edge detected internally)"],
    ["pick_x/y/z",   "double", "Input",  "Pick position in mm"],
    ["place_x/y/z",  "double", "Input",  "Place position in mm"],
    ["sim_t",        "double", "Input",  "Simulation time from Clock block"],
    ["state_id",     "double", "Output", "Current FSM state (0–5)"],
    ["theta1/2/3_out","double","Output", "Motor angle commands in radians"],
    ["gripper_out",  "double", "Output", "1 = gripper closed, 0 = open"],
    ["traj [3000×3]","double", "Local",  "Pre-built waypoint buffer (x, y, z columns)"],
    ["wp_idx",       "double", "Local",  "Current waypoint index into traj"],
    ["wp_count",     "double", "Local",  "Total valid waypoints in traj"],
    ["settle_timer", "double", "Local",  "Absolute time when gripper dwell ends"],
    ["last_x/y/z",   "double", "Local",  "Last commanded Cartesian position (for RESETTING)"],
    ["busy",         "double", "Local",  "1 while a cycle is running (blocks re-trigger)"],
    ["traj_done",    "double", "Local",  "1 when all waypoints in traj have been sent"],
    ["traj_failed",  "double", "Local",  "1 if any IK call returns status ≠ 0"],
]
story.append(param_table(data_rows, [40*mm, 20*mm, 20*mm, 80*mm]))
story.append(sp(6))

story.append(h2("7.3  Embedded MATLAB functions  (Stateflow.EMFunction)"))
fn_rows = [
    ["Function",        "Signature",                               "Maps to (Python)"],
    ["angle_yz",        "[status,θ] = angle_yz(x,y,z,e,f,re,rf)", "_delta_calcAngleYZ"],
    ["delta_ik",        "[status,t1,t2,t3] = delta_ik(x,y,z)",    "delta_calcInverse"],
    ["check_ws",        "ok = check_ws(x,y,z)",                    "check_workspace"],
    ["gen_traj_seg",    "[wps,n] = gen_traj_seg(p0,p1,v,a,dt)",   "linear_waypoints"],
]
story.append(param_table(fn_rows, [32*mm, 70*mm, 58*mm]))
story.append(note(
    "All four functions live at chart scope.  delta_ik calls angle_yz — "
    "Stateflow resolves intra-chart function calls during simulation."
))
story.append(sp(4))

story.append(h2("7.4  How a motion state executes a trajectory"))
story.append(p(
    "Every motion state (MOVING_TO_PICK, MOVING_TO_PLACE, RESETTING) uses the same "
    "two-phase pattern:"
))
story.append(code(
    "% ── entry: build the full multi-segment path once ─────────────────\n"
    "wp_count = 0;  wp_idx = 1;\n"
    "[sg, n] = gen_traj_seg(x0,y0,z0, x1,y1,z1, 80, 200, 0.05);\n"
    "traj(1:n, :) = sg(1:n, :);  wp_count = n;\n"
    "% ... repeat for each subsequent segment ...\n\n"
    "% ── do: step one waypoint per 20 Hz tick ──────────────────────────\n"
    "if wp_idx <= wp_count\n"
    "    [st, t1, t2, t3] = delta_ik(traj(wp_idx,1), traj(wp_idx,2), traj(wp_idx,3));\n"
    "    if st == 0\n"
    "        theta1_out = t1 * pi/180;\n"
    "        theta2_out = t2 * pi/180;\n"
    "        theta3_out = t3 * pi/180;\n"
    "    else\n"
    "        traj_failed = 1;   % → error transition fires next tick\n"
    "    end\n"
    "    last_x = traj(wp_idx,1);  last_y = traj(wp_idx,2);  last_z = traj(wp_idx,3);\n"
    "    wp_idx = wp_idx + 1;\n"
    "else\n"
    "    traj_done = 1;   % → normal transition fires next tick\n"
    "end"
))
story.append(sp(4))

# ── 8. How to use ──────────────────────────────────────────────────────────────
story.append(h1("8  How to Run"))
story.append(h2("8.1  Generate the .slx model"))
story.append(code(
    ">> cd /path/to/delta_ws\n"
    ">> run('delta_blind_pick_place_sim.m')\n"
    "% Output: delta_blind_pick_place.slx"
))
story.append(h2("8.2  Simulate"))
story.append(code(
    ">> sim('delta_blind_pick_place')\n"
    "% or press the Run button in Simulink"
))
story.append(h2("8.3  Change pick / place positions"))
story.append(p(
    "Double-click any of the six <font name='Courier'>Constant</font> source blocks "
    "(<font name='Courier'>pick_x</font> … <font name='Courier'>place_z</font>) and "
    "enter the desired value in mm.  "
    "All positions are validated against the workspace limits on every IK call — "
    "an out-of-reach point sets <font name='Courier'>traj_failed</font> and triggers "
    "the error recovery path."
))
story.append(h2("8.4  Change speed"))
story.append(p(
    "In each motion state's <b>entry</b> action, edit the three constants:"
))
story.append(code(
    "V = 80;    % mm/s   — max Cartesian speed\n"
    "A = 200;   % mm/s²  — max Cartesian acceleration\n"
    "DT = 0.05; % s      — waypoint interval (keep = chart SampleTime)"
))
story.append(p(
    "Use the presets from Section 5.2.  Increasing speed reduces cycle time but requires "
    "matching motor velocity/acceleration limits in <font name='Courier'>DeltaMotorController</font>."
))
story.append(sp(4))

# ── 9. ROS 2 vs Simulink mapping ──────────────────────────────────────────────
story.append(PageBreak())
story.append(h1("9  ROS 2 ↔ Simulink Mapping"))
map_rows = [
    ["ROS 2 concept",                "Simulink equivalent"],
    ["BlindPickAndPlace ROS 2 node", "BlindPickPlace_FSM Stateflow chart"],
    ["/delta/trigger_pick service",  "Trigger Pulse Generator (rising edge)"],
    ["/delta/blind_target topic",    "pick_x/y/z Constant blocks"],
    ["/delta/robot_state publisher", "state_id output (0–5 integer)"],
    ["DeltaMotorController.execute_trajectory()", "do: action — IK per tick → theta_out"],
    ["linear_waypoints() in trajectory.py",       "gen_traj_seg() EMFunction in chart"],
    ["delta_calcInverse() in fk_ik.py",            "delta_ik() EMFunction in chart"],
    ["check_workspace() in fk_ik.py",              "check_ws() EMFunction in chart"],
    ["PneumaticGripper.close() / open()",          "gripper_out = 1 / 0 in entry action"],
    ["close_settle_s / open_settle_s (0.5 s)",     "settle_timer = sim_t + 0.5"],
    ["SAFE_TRANSIT_Z = −400 mm",                   "TZ = min([pick_z, place_z, −400]) in entry"],
    ["ENABLE_MOTORS flag",                         "Not needed — simulation always 'runs'"],
    ["threading.Thread (daemon)",                  "Stateflow do: action (runs each tick)"],
]
story.append(param_table(map_rows, [88*mm, 72*mm]))
story.append(sp(4))

# ── footer note ───────────────────────────────────────────────────────────────
story.append(sp(8))
story.append(hr())
story.append(Paragraph(
    "Generated from blind_pick_place.py  ·  fk_ik.py  ·  trajectory.py  ·  config.py  "
    "·  delta_blind_pick_place_sim.m",
    ParagraphStyle("foot", fontSize=7.5, alignment=TA_CENTER,
                   textColor=DGREY, fontName="Courier")
))

# ── build ──────────────────────────────────────────────────────────────────────
doc.build(story)
print(f"\nSaved: {os.path.abspath(OUT)}\n")
