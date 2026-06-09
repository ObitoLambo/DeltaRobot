# CHANGES: [2.2] added AUTO_MOVE, ENABLE_MOTORS, MOVE_COOLDOWN_SEC, MOVE_THRESHOLD_MM
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

DETECTION_MODE = "orange_blob"
YOLO_MODEL = "yolo11n.pt"
YOLO_IMGSZ = 1280
YOLO_IOU = 0.45
YOLO_MAX_DET = 20
YOLO_DEVICE = "cpu"
CONF_THRES = 0.25
TARGET_CLASS = None
VIEW_IMAGE = True
DRAW_CAMERA_AXIS_LEGEND = False
DRAW_BASE_AXIS_OVERLAY = False
DRAW_DETECTION_CONTOUR = False
DRAW_DETECTION_CORNERS = False
DRAW_OBJECT_SIZE_LABEL = False
REQUIRE_FULL_BBOX_IN_FRAME = True
FRAME_MARGIN_PX = 12
DETECT_ONLY_IN_WORKSPACE = True
DRAW_REJECTED_DETECTIONS = False
BBOX_LABEL = "box"
BBOX_FRAME_MIN_AREA = 2500.0
BBOX_FRAME_MAX_AREA_RATIO = 0.70
BBOX_FRAME_MIN_WIDTH_PX = 60
BBOX_FRAME_MIN_HEIGHT_PX = 40
BBOX_NMS_IOU = 0.35
CANNY_LOW = 60
CANNY_HIGH = 180

BLUE_RECT_HUE_LOW  = 100       # OpenCV H (0-180): start of blue band
BLUE_RECT_HUE_HIGH = 130       # end of blue band
BLUE_RECT_SAT_MIN  = 80        # minimum saturation — rejects washed-out / gray regions
BLUE_RECT_VAL_MIN  = 50        # minimum value — rejects near-black shadows
BLUE_RECT_MIN_SAT_MEAN   = 100.0   # mean saturation inside contour (replaces contrast check)
BLUE_RECT_MIN_AREA_PX    = 600.0
BLUE_RECT_MAX_AREA_RATIO = 0.12
BLUE_RECT_MIN_WIDTH_PX   = 24
BLUE_RECT_MIN_HEIGHT_PX  = 24
BLUE_RECT_BORDER_REJECT_PX     = 10
BLUE_RECT_MASK_OPEN_PX         = 3
BLUE_RECT_MASK_CLOSE_PX        = 7
BLUE_RECT_POLY_EPSILON_SCALE   = 0.05
BLUE_RECT_MIN_RECTANGULARITY   = 0.76
BLUE_RECT_MIN_ASPECT_RATIO     = 1.05
BLUE_RECT_MAX_ASPECT_RATIO     = 4.50

ORANGE_SQ_HUE_LOW  = 5        # OpenCV H (0-180): start of orange band
ORANGE_SQ_HUE_HIGH = 30       # end of orange band
ORANGE_SQ_SAT_MIN  = 120      # reject washed-out / gray
ORANGE_SQ_VAL_MIN  = 80       # reject near-black shadows
ORANGE_SQ_MIN_SAT_MEAN        = 100.0

# Orange blob detection (simpler — centroid only, no shape check)
ORANGE_BLOB_HUE_LOW   = 5     # OpenCV H (0-180)
ORANGE_BLOB_HUE_HIGH  = 30
ORANGE_BLOB_SAT_MIN   = 100
ORANGE_BLOB_VAL_MIN   = 80
ORANGE_BLOB_MIN_AREA_PX      = 80.0    # 30mm @ Z=-410 → ~230 px², well below
ORANGE_BLOB_MAX_AREA_RATIO   = 0.05
ORANGE_BLOB_MASK_OPEN_PX     = 2
ORANGE_BLOB_MASK_CLOSE_PX    = 5
ORANGE_BLOB_BORDER_REJECT_PX = 4
ORANGE_SQ_MIN_AREA_PX         = 100.0   # 30mm @ Z=-410 → ~230 px², set well below
ORANGE_SQ_MAX_AREA_RATIO      = 0.05
ORANGE_SQ_MIN_WIDTH_PX        = 8       # 30mm → ~15 px wide
ORANGE_SQ_MIN_HEIGHT_PX       = 8
ORANGE_SQ_BORDER_REJECT_PX    = 5       # reduced — small object near edge should not be rejected too aggressively
ORANGE_SQ_MASK_OPEN_PX        = 3
ORANGE_SQ_MASK_CLOSE_PX       = 7
ORANGE_SQ_POLY_EPSILON_SCALE  = 0.05
ORANGE_SQ_MIN_RECTANGULARITY  = 0.76
ORANGE_SQ_MIN_ASPECT_RATIO    = 0.75  # square: allow slight perspective
ORANGE_SQ_MAX_ASPECT_RATIO    = 1.33

WHITE_RECT_MIN_AREA_PX = 600.0
WHITE_RECT_MAX_AREA_RATIO = 0.12
WHITE_RECT_MIN_WIDTH_PX = 24
WHITE_RECT_MIN_HEIGHT_PX = 24
WHITE_RECT_MIN_BRIGHTNESS = 180
WHITE_RECT_MAX_SATURATION = 80
WHITE_RECT_MIN_CONTRAST = 28.0
WHITE_RECT_MASK_OPEN_PX = 3
WHITE_RECT_MASK_CLOSE_PX = 7
WHITE_RECT_BORDER_REJECT_PX = 10
WHITE_RECT_POLY_EPSILON_SCALE = 0.05
WHITE_RECT_MIN_RECTANGULARITY = 0.76
WHITE_RECT_MIN_ASPECT_RATIO = 1.05
WHITE_RECT_MAX_ASPECT_RATIO = 4.50
WHITE_RECT_SUBPIX_WINDOW = 5

PLANE_HOMOGRAPHY_ENABLE = False
PLANE_HOMOGRAPHY_MATRIX = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)

RECT_POSE_ENABLE = False
RECT_REAL_WIDTH_MM = 0.0
RECT_REAL_HEIGHT_MM = 0.0

WORKSPACE_ROI_ENABLE = False
WORKSPACE_MARGIN_X_PX = 80
WORKSPACE_MARGIN_Y_PX = 80
ROBOT_EXCLUDE_ENABLE = False
ROBOT_EXCLUDE_POLYGONS_NORM = (
    ((0.02, 0.00), (0.12, 0.00), (0.56, 0.28), (0.50, 0.40), (0.40, 0.34)),
    ((0.88, 0.00), (0.98, 0.00), (0.60, 0.34), (0.50, 0.40), (0.44, 0.28)),
    ((0.42, 0.30), (0.58, 0.30), (0.63, 0.58), (0.58, 0.82), (0.42, 0.82), (0.37, 0.58)),
)
ROBOT_EXCLUDE_MAX_BBOX_OVERLAP = 0.12
ROBOT_EXCLUDE_MAX_CONTOUR_OVERLAP = 0.08
DRAW_ROBOT_EXCLUDE = False
FOREGROUND_FLOOR_DEPTH_M = 0.870
FOREGROUND_FLOOR_PERCENTILE = 80.0
FOREGROUND_BG_CLOSE_PX = 41
FOREGROUND_MASK_OPEN_PX = 5
FOREGROUND_MASK_CLOSE_PX = 11
FOREGROUND_MIN_HEIGHT_M = 0.008
FOREGROUND_MAX_HEIGHT_M = 0.200
FOREGROUND_MIN_AREA_PX = 3000.0
FOREGROUND_MAX_AREA_RATIO = 0.50
FOREGROUND_MIN_WIDTH_PX = 60
FOREGROUND_MIN_HEIGHT_PX = 40
FOREGROUND_REQUIRE_BOX_LIKE = True
FOREGROUND_BORDER_REJECT_PX = 30
FOREGROUND_MAX_ASPECT_RATIO = 4.0


BOX_ONLY_ENABLE = True
BOX_ROI_SHRINK = 0.20
BOX_MIN_CONTOUR_AREA = 80.0
BOX_POLY_EPSILON_SCALE = 0.04
BOX_MIN_RECTANGULARITY = 0.72
BOX_MAX_ASPECT_RATIO = 3.0

AVG_FRAME_COUNT = 5              # 5 frames: less lag on conveyor, still smooth
CENTER_WINDOW = 5
TRACK_GRID_PX = 80
# Larger grid for conveyor mode: object moves ~2-4 px/frame so 160 px gives
# ~40-80 stable frames per cell before the track ID resets at a boundary.
CONVEYOR_TRACK_GRID_PX = 160
DETECTION_CONFIRM_FRAMES = 2     # 2 frames: faster trigger, still avoids false positives
DETECTION_LOST_RESET_FRAMES = 8  # hold on longer before resetting — avoids flicker

STABLE_THRESH_X_MM = 1.5
STABLE_THRESH_Y_MM = 1.5
STABLE_THRESH_Z_MM = 2.0

DEPTH_MIN_M = 0.001
DEPTH_MAX_M = 1.00
DEPTH_BOX_SCALE = 0.35
DEPTH_MIN_VALID_PIXELS = 20
DEPTH_USE_CONTOUR_MASK = True
DEPTH_CONTOUR_ERODE_PX = 5
DEPTH_TRIM_LOW_PERCENT = 25.0
DEPTH_TRIM_HIGH_PERCENT = 75.0
DEPTH_MAD_SCALE = 2.0
DEPTH_HISTORY_SIZE = 7
DEPTH_MAX_JUMP_M = 0.015

# Home position — workspace centre, mid-height
HOME_X =   0.0
HOME_Y =   0.0
HOME_Z = -350.0

# Drop at belt exit edge — Y = -150mm
PLACE_X =  150.0
PLACE_Y = -150.0

PLACE_Z = -460.0   # wrist Z for place  (EE tip = PLACE_Z - 150 = -610mm, 40mm above belt)
PICK_Z  = -470.0   # wrist Z for pick   (EE tip = PICK_Z  - 150 = -620mm, 30mm above belt)

# Acceleration used for triangular travel-time prediction (medium preset).
# Within ±150mm workspace all moves are triangular: t = 2*sqrt(dist / TRAJ_A_MAX_MM_S2)
TRAJ_A_MAX_MM_S2 = 5000.0

# Extra wait at approach Z (mm above object) after robot pre-positions.
# Gives the object time to arrive if Y prediction is still slightly off.
# Increase in small steps (0.1 s) if robot still arrives before object.
CONVEYOR_APPROACH_WAIT_SEC = 0.3   # short motor-settle; arrival now handled dynamically
CONVEYOR_ARRIVAL_Y_THRESH_MM = 15.0  # pick when |err_y| < this (object within 15mm in Y)
CONVEYOR_ARRIVAL_TIMEOUT_S   = 5.0   # give up and pick anyway after 5s

# Static EE correction offsets (mm).  Measure the real error at your target
# position and set each offset to negate it: if the EE lands 10 mm too far in
# +X, set EE_OFFSET_X_MM = -10.0.
EE_OFFSET_X_MM = 0.0
EE_OFFSET_Y_MM = 0.0
EE_OFFSET_Z_MM   = -60.0  # camera now reports platform Z directly via FAKE_DEPTH_M=0.650

FAKE_DEPTH_ENABLE = True
FAKE_DEPTH_M = 0.650      # fixed camera-to-belt distance (m); tune if belt Z drifts

ERROR_MAP_ENABLE = False

MOTOR_VEL_MAX    = 3.0   # PP mode max velocity — increase for faster moves (was 1.0)
MOTOR_ACC_SET    = 5.0   # PP mode acceleration — increase for snappier starts (was 2.0)
FK_VERIFY_TOL_MM = 3.0
PRINT_COOLDOWN_SEC = 1.0
FAKE_MOVE_COOLDOWN_SEC = 1.0
AUTO_MOVE = True
ENABLE_MOTORS = True
MOVE_COOLDOWN_SEC = 1.5
MOVE_THRESHOLD_MM = 4.0

# z_base = -530 + CAM_TZ_MM(-20) = -550mm = belt surface
# z_platform = -550 - EE_OFFSET_Z_MM(-60) = -490mm

# Fixed fake object position for testing EE correction
# Set FAKE_OBJ_ENABLE = True to use fixed position
# instead of real camera detection


VISION_ONLY_ENABLE = False
SIMPLE_RESULT_PRINT = False
PURE_CAMERA_TEST_ENABLE = False

# Conveyor belt mode: skip stability check (object is always moving).
# Camera publishes a target once track is confirmed + averaged over AVG_FRAME_COUNT frames.
CONVEYOR_MODE = True

# ── Conveyor belt hardware parameters ────────────────────────────────────────
# Stepper motor: MIN_DELAY=70µs, STEPS_PER_REV=1600, GEAR_RATIO=36, PULLEY_DIA=49mm
#   Motor RPM  = 60_000_000 / (2 × 70 × 1600) = 267.86 RPM
#   Output RPM = 267.86 / 36                   = 7.44  RPM
#   Belt mm/s  = (7.44 × π × 49) / 60         = 19.09 mm/s  ← design maximum
#   Measured physical speed: 12.85 mm/s (stepper running below design maximum)
CONVEYOR_BELT_SPEED_MM_S = 12.85   # mm/s — physically measured

# Velocity sanity bounds: camera estimate is rejected if it falls outside this range.
# Lower bound covers belt not yet at speed; upper bound catches outlier regression.
CONVEYOR_VY_MIN_MM_S =  1.0    # below this → belt probably stopped, use 0
CONVEYOR_VY_MAX_MM_S = 25.0    # above this → regression outlier, clamp to design speed

# Minimum seconds between consecutive target publishes to avoid flooding the robot.
TARGET_PUBLISH_COOLDOWN_SEC = 3.0

# Camera-frame workspace zone overlay.
# Projects the robot reachable square (±X_LIMIT, ±Y_LIMIT) at WORKSPACE_PICK_Z_MM
# and draws three coloured zones: approach (amber), workspace (green), exit (red).
DRAW_WORKSPACE_ZONES = True
WORKSPACE_PICK_Z_MM  = -490.0   # platform Z at pick height (EE tip 150mm below = belt)

# EE marker detection (white laser dot on end-effector tip)
EE_CORRECTION_ENABLE    = True
EE_CORRECTION_MAX_MM    = 30.0
EE_CORRECTION_ALPHA     = 0.5
EE_CORRECTION_GAIN      = 0.5    # halves X error each iter: 13→6.5→3.25→1.6mm
EE_CORRECTION_THRESH_MM = 3.0    # descend only when X ≤ 3mm from centroid
EE_CORRECTION_MIN_MM    = 2.0
EE_CORRECTION_MAX_ITERS = 6      # up to 6 iters; ~3 needed for 13mm→<2mm
EE_CORRECTION_TIMEOUT_S = 2.5
EE_CORRECTION_WAIT_S    = 0.05
EE_LASER_HUE_LOW1  = 0      # red lower range
EE_LASER_HUE_HIGH1 = 10
EE_LASER_HUE_LOW2  = 130    # magenta/pink: 650nm laser on D455 appears H≈150-165
EE_LASER_HUE_HIGH2 = 180
EE_LASER_SAT_MIN        = 10     # laser pixels have S=14-48 (near-white pink core)
EE_LASER_VAL_MIN        = 230    # only near-saturated pixels: cuts ambient surfaces
EE_LASER_CORE_VAL_MIN   = 180    # overexposed white core: any hue, very bright
EE_LASER_CORE_SAT_MAX   = 80     # white core has near-zero saturation
EE_LASER_MIN_AREA  = 1
EE_LASER_MAX_AREA  = 100
EE_LASER_MAX_JUMP_PX   = 999   # disabled — ROI search handles rejection instead
EE_LASER_SMOOTH_FRAMES = 5     # 5-frame median
EE_LASER_ROI_PX        = 35    # search radius around last position (px)
DRAW_EE_MARKER         = True
# Shift the overlay box in the camera stream without affecting 3D detection.
# Positive X_OFFSET_MM moves box right in robot base frame (→ left in image).
# Positive Y_OFFSET_MM moves box toward the approach side (belt entry direction).
WORKSPACE_OVERLAY_X_OFFSET_MM = 0.0
WORKSPACE_OVERLAY_Y_OFFSET_MM = 0.0

CAMERA_TRANSFORM_MODE = "A"
CAMERA_USE_DIRECT_MATRIX = True

# Rotation: x_base = -x_cam,  y_base = y_cam,  z_base = -z_cam
# Measured mount: Y = 300 mm in front of base, Z = 80 mm above base origin
# Working distance to home (z=-350): 80 + 350 = 430 mm
CAMERA_DIRECT_MATRIX = (
    (-1.0,  0.0,  0.0),
    ( 0.0,  1.0,  0.0),
    ( 0.0,  0.0, -1.0),
)
CAM_FINE_ROLL_DEG  = 0.0
CAM_FINE_PITCH_DEG = 0.0
CAM_FINE_YAW_DEG   = 0.0
CAM_TX_MM =  -10.0   # tuning — adjust in ±10mm steps until green box centres under arm
CAM_TY_MM = 235.0    # calculated: EE dot at y=38px (exit Y=-150mm) → crosshair at 161px → T_Y=96≈100mm
CAM_TZ_MM =  -20.0   # measured: camera is 20 mm below base frame origin

# Full 4×4 homogeneous T_cam_to_base  (p_base = T @ [p_cam; 1])
# Built from the R and t above — use camera_system._build_T_cam_to_base() at runtime
CAMERA_T_BASE = (
    (-1.0,  0.0,  0.0,  CAM_TX_MM),
    ( 0.0,  1.0,  0.0, CAM_TY_MM),
    ( 0.0,  0.0, -1.0,  CAM_TZ_MM),
    ( 0.0,  0.0,  0.0,    1.0),
)

X_LIMIT = 150.0   # rectangular pre-filter; IK in check_workspace rejects unreachable corners
Y_LIMIT = 150.0
Z_MIN = -560.0
# Actual IK ceiling at centre: -sqrt(re^2 - (rf + (f-e)*tan30/2)^2) ≈ -323.5 mm.
# The old value (-196.875) was geometrically wrong; any Z above -323 fails IK.
Z_MAX = -323.0

THETA1_MIN = -5
THETA1_MAX = 90   # physical hard stop observed at ~56° — use 54° with 2° margin
THETA2_MIN = -5
THETA2_MAX = 90.0
THETA3_MIN = -5
THETA3_MAX = 90.0
