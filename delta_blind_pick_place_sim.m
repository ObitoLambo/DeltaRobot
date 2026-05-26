%% delta_blind_pick_place_sim.m
%%
%% Generates  delta_blind_pick_place.slx
%% Full Simulink / Stateflow model of the blind pick-and-place sequence.
%%
%% Mirrors: src/delta_main_app/delta_main_app/blind_pick_place.py
%%          (BlindPickAndPlace._run_one)
%%
%% FSM states
%%   IDLE → MOVING_TO_PICK → GRASPING → MOVING_TO_PLACE → DROPPING → RESETTING → IDLE
%%
%% Transit-Z routing (identical to ROS 2 node)
%%   Every move passes through the centre column (0,0) at a safe Z ≤ -400 mm.
%%   MOVING_TO_PICK  : HOME → home_t → pick_t → pick_xyz        (3 segments)
%%   MOVING_TO_PLACE : pick_xyz → pick_t → home_t → place_t → place_xyz  (4 segments)
%%   RESETTING       : last_pos → cur_t  → home_t → HOME         (3 segments)
%%
%% Robot geometry (mm)
%%   e=35  f=157  re=400  rf=200
%%
%% Requirements: MATLAB R2021b+, Simulink, Stateflow
%% Usage:  >> run('delta_blind_pick_place_sim.m')

MDL = 'delta_blind_pick_place';

%% ═══════════════════════════════════════════════════════════════════
%% 1. Create / overwrite model
%% ═══════════════════════════════════════════════════════════════════
if bdIsLoaded(MDL), close_system(MDL, 0); end
if exist([MDL '.slx'], 'file'), delete([MDL '.slx']); end

new_system(MDL);
open_system(MDL);
set_param(MDL, ...
    'StopTime',   '30', ...
    'SolverType', 'Fixed-step', ...
    'FixedStep',  '0.05', ...          % 20 Hz — matches trajectory dt
    'Solver',     'FixedStepDiscrete');

%% ═══════════════════════════════════════════════════════════════════
%% 2. Add Simulink blocks
%% ═══════════════════════════════════════════════════════════════════

% Trigger: 1-step pulse at t=2 s (simulates /delta/trigger_pick service call)
add_block('simulink/Sources/Pulse Generator', [MDL '/Trigger'], ...
    'Position',   [30  26 80  54], ...
    'Period',     '25',   ...
    'PulseWidth', '4',    ...   % 4% of 25 s ≈ 1 sample wide
    'Amplitude',  '1',   ...
    'PhaseDelay', '2');

% Pick position (mm)  — default: (0, 80, -380)
add_block('simulink/Sources/Constant',[MDL '/pick_x'],'Value','0',   'Position',[30  90  90 110]);
add_block('simulink/Sources/Constant',[MDL '/pick_y'],'Value','80',  'Position',[30 130  90 150]);
add_block('simulink/Sources/Constant',[MDL '/pick_z'],'Value','-380','Position',[30 170  90 190]);

% Place position (mm) — default: (80, 0, -380)
add_block('simulink/Sources/Constant',[MDL '/place_x'],'Value','80', 'Position',[30 220  90 240]);
add_block('simulink/Sources/Constant',[MDL '/place_y'],'Value','0',  'Position',[30 260  90 280]);
add_block('simulink/Sources/Constant',[MDL '/place_z'],'Value','-380','Position',[30 300  90 320]);

% Simulation time (used for gripper settle timer)
add_block('simulink/Sources/Clock',[MDL '/SimTime'],'Position',[30 360 70 390]);

% Stateflow chart
add_block('sflib/Chart',[MDL '/BlindPickPlace_FSM'],'Position',[160 20 980 620]);

% Mux for the three motor angles
add_block('simulink/Signal Routing/Mux',[MDL '/AngleMux'], ...
    'Inputs','3','Position',[1020 115 1025 205]);

% Scopes
add_block('simulink/Sinks/Scope',[MDL '/StateScope'],  'Position',[1060  40 1110  70]);
add_block('simulink/Sinks/Scope',[MDL '/AnglesScope'], 'Position',[1060 145 1110 175]);
add_block('simulink/Sinks/Scope',[MDL '/GripperScope'],'Position',[1060 230 1110 260]);

%% ═══════════════════════════════════════════════════════════════════
%% 3. Wire blocks
%% ═══════════════════════════════════════════════════════════════════
add_line(MDL,'Trigger/1',  'BlindPickPlace_FSM/1');
add_line(MDL,'pick_x/1',   'BlindPickPlace_FSM/2');
add_line(MDL,'pick_y/1',   'BlindPickPlace_FSM/3');
add_line(MDL,'pick_z/1',   'BlindPickPlace_FSM/4');
add_line(MDL,'place_x/1',  'BlindPickPlace_FSM/5');
add_line(MDL,'place_y/1',  'BlindPickPlace_FSM/6');
add_line(MDL,'place_z/1',  'BlindPickPlace_FSM/7');
add_line(MDL,'SimTime/1',  'BlindPickPlace_FSM/8');

add_line(MDL,'BlindPickPlace_FSM/1','StateScope/1');
add_line(MDL,'BlindPickPlace_FSM/2','AngleMux/1');
add_line(MDL,'BlindPickPlace_FSM/3','AngleMux/2');
add_line(MDL,'BlindPickPlace_FSM/4','AngleMux/3');
add_line(MDL,'AngleMux/1','AnglesScope/1');
add_line(MDL,'BlindPickPlace_FSM/5','GripperScope/1');

%% ═══════════════════════════════════════════════════════════════════
%% 4. Configure Stateflow chart
%% ═══════════════════════════════════════════════════════════════════
rt    = sfroot();
mdlSF = rt.find('-isa','Simulink.BlockDiagram','-and','Name',MDL);
ch    = mdlSF.find('-isa','Stateflow.Chart');

ch.StateMachineType = 'Classic';
ch.ActionLanguage   = 'MATLAB';
ch.ChartUpdate      = 'DISCRETE';
ch.SampleTime       = '0.05';

%% ═══════════════════════════════════════════════════════════════════
%% 5. Chart data
%% ═══════════════════════════════════════════════════════════════════

% ── inputs ──────────────────────────────────────────────────────────
d=Stateflow.Data(ch); d.Name='trigger';  d.Scope='Input'; d.DataType='double';  d.Port=1;
d=Stateflow.Data(ch); d.Name='pick_x';  d.Scope='Input'; d.DataType='double';  d.Port=2;
d=Stateflow.Data(ch); d.Name='pick_y';  d.Scope='Input'; d.DataType='double';  d.Port=3;
d=Stateflow.Data(ch); d.Name='pick_z';  d.Scope='Input'; d.DataType='double';  d.Port=4;
d=Stateflow.Data(ch); d.Name='place_x'; d.Scope='Input'; d.DataType='double';  d.Port=5;
d=Stateflow.Data(ch); d.Name='place_y'; d.Scope='Input'; d.DataType='double';  d.Port=6;
d=Stateflow.Data(ch); d.Name='place_z'; d.Scope='Input'; d.DataType='double';  d.Port=7;
d=Stateflow.Data(ch); d.Name='sim_t';   d.Scope='Input'; d.DataType='double';  d.Port=8;

% ── outputs ─────────────────────────────────────────────────────────
d=Stateflow.Data(ch); d.Name='state_id';   d.Scope='Output'; d.DataType='double'; d.Port=1; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='theta1_out'; d.Scope='Output'; d.DataType='double'; d.Port=2; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='theta2_out'; d.Scope='Output'; d.DataType='double'; d.Port=3; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='theta3_out'; d.Scope='Output'; d.DataType='double'; d.Port=4; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='gripper_out';d.Scope='Output'; d.DataType='double'; d.Port=5; d.Props.InitialValue='0';

% ── locals ──────────────────────────────────────────────────────────
d=Stateflow.Data(ch); d.Name='busy';         d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='traj_done';    d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='traj_failed';  d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='wp_idx';       d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='1';
d=Stateflow.Data(ch); d.Name='wp_count';     d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='settle_timer'; d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='last_x';       d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='last_y';       d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';
d=Stateflow.Data(ch); d.Name='last_z';       d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='-350';
d=Stateflow.Data(ch); d.Name='prev_trigger'; d.Scope='Local'; d.DataType='double'; d.Props.InitialValue='0';

% trajectory buffer  [3000 × 3]  — generous enough for all segments at 20 Hz
d=Stateflow.Data(ch);
d.Name      = 'traj';
d.Scope     = 'Local';
d.DataType  = 'double';
d.Dimensions = '3000, 3';
d.Props.InitialValue = 'zeros(3000,3)';

%% ═══════════════════════════════════════════════════════════════════
%% 6. MATLAB helper functions (Stateflow.EMFunction)
%% ═══════════════════════════════════════════════════════════════════

% ── 6a. angle_yz  (IK sub-function for one arm) ─────────────────────
f_ayz = Stateflow.EMFunction(ch);
f_ayz.Name     = 'angle_yz';
f_ayz.Position = [1050  30 300 120];
f_ayz.LabelString = '[status, theta] = angle_yz(x0, y0, z0, e_, f_, re_, rf_)';
f_ayz.Script = strjoin({
'function [status, theta] = angle_yz(x0, y0, z0, e_, f_, re_, rf_)'
'% IK helper: solve one arm angle in the YZ plane.'
'% Matches Python _delta_calcAngleYZ.'
'sqrt3 = sqrt(3.0);'
'tan30 = 1.0 / sqrt3;'
'y1 = -0.5 * tan30 * f_;'
'y0 = y0 - 0.5 * tan30 * e_;'
'a = (x0^2 + y0^2 + z0^2 + rf_^2 - re_^2 - y1^2) / (2.0 * z0);'
'b = (y1 - y0) / z0;'
'd = -(a + b*y1)^2 + rf_ * (b^2 * rf_ + rf_);'
'if d < 0'
'    status = -1;'
'    theta  = 0.0;'
'    return;'
'end'
'yj    = (y1 - a*b - sqrt(d)) / (b^2 + 1.0);'
'zj    = a + b * yj;'
'theta = atan2d(-zj, (y1 - yj));'
'if yj > y1'
'    theta = theta + 180.0;'
'end'
'status = 0;'
'end'
}, newline);

% ── 6b. delta_ik  (full inverse kinematics, returns degrees) ─────────
f_ik = Stateflow.EMFunction(ch);
f_ik.Name     = 'delta_ik';
f_ik.Position = [1050 180 300 130];
f_ik.LabelString = '[status, t1, t2, t3] = delta_ik(x0, y0, z0)';
f_ik.Script = strjoin({
'function [status, t1, t2, t3] = delta_ik(x0, y0, z0)'
'% Delta robot inverse kinematics.'
'% Geometry: e=35 f=157 re=400 rf=200 mm  (matches fk_ik.py).'
'% Returns joint angles in DEGREES; status 0=OK -1=unreachable.'
'E_=35.0; F_=157.0; RE_=400.0; RF_=200.0;'
'status=-1; t1=0; t2=0; t3=0;'
'sqrt3=sqrt(3.0); sin120=sqrt3/2; cos120=-0.5;'
'[s1,a1]=angle_yz(x0, y0, z0, E_,F_,RE_,RF_);'
'if s1~=0, return; end'
'[s2,a2]=angle_yz(x0*cos120+y0*sin120, y0*cos120-x0*sin120, z0, E_,F_,RE_,RF_);'
'if s2~=0, return; end'
'[s3,a3]=angle_yz(x0*cos120-y0*sin120, y0*cos120+x0*sin120, z0, E_,F_,RE_,RF_);'
'if s3~=0, return; end'
'if a1<0 || a2<0 || a3<0, return; end'
'status=0; t1=a1; t2=a2; t3=a3;'
'end'
}, newline);

% ── 6c. check_ws  (workspace bounding-box + IK feasibility) ──────────
f_ws = Stateflow.EMFunction(ch);
f_ws.Name     = 'check_ws';
f_ws.Position = [1050 340 300 100];
f_ws.LabelString = 'ok = check_ws(x, y, z)';
f_ws.Script = strjoin({
'function ok = check_ws(x, y, z)'
'% Workspace check: bounding box then IK feasibility.'
'% Limits from delta_common/config.py'
'X_LIM=151.563; Y_LIM=151.563; Z_MIN=-500.0; Z_MAX=-323.0;'
'ok = 0;'
'if abs(x)>X_LIM || abs(y)>Y_LIM || z<Z_MIN || z>Z_MAX, return; end'
'[st,~,~,~] = delta_ik(x,y,z);'
'ok = double(st==0);'
'end'
}, newline);

% ── 6d. gen_traj_seg  (trapezoidal waypoints for one straight segment) ─
f_seg = Stateflow.EMFunction(ch);
f_seg.Name     = 'gen_traj_seg';
f_seg.Position = [1050 470 300 180];
f_seg.LabelString = '[wps, n] = gen_traj_seg(p0x,p0y,p0z, p1x,p1y,p1z, vmax,amax,dt_s)';
f_seg.Script = strjoin({
'function [wps, n] = gen_traj_seg(p0x,p0y,p0z, p1x,p1y,p1z, vmax,amax,dt_s)'
'% Trapezoidal velocity profile waypoints along p0→p1.'
'% Matches trajectory.py linear_waypoints().'
'% Returns fixed 600×3 buffer; only first n rows are valid.'
'MAX_PTS = 600;'
'wps = zeros(MAX_PTS, 3);'
'n = 0;'
'dx=p1x-p0x; dy=p1y-p0y; dz=p1z-p0z;'
'dist = sqrt(dx*dx + dy*dy + dz*dz);'
'if dist < 0.5'
'    n=1; wps(1,:)=[p1x p1y p1z]; return;'
'end'
'ux=dx/dist; uy=dy/dist; uz=dz/dist;'
'ta = vmax/amax;'
'da = 0.5*amax*ta^2;'
'if 2*da >= dist'
'    ta   = sqrt(dist/amax);'
'    vp   = amax*ta;'
'    da   = 0.5*amax*ta^2;'
'    T    = 2*ta;'
'else'
'    vp = vmax;'
'    T  = 2*ta + (dist-2*da)/vmax;'
'end'
't = 0.0;'
'while t <= T+1e-9 && n < MAX_PTS'
'    if t <= ta'
'        s = 0.5*amax*t^2;'
'    elseif t <= T-ta'
'        s = da + vp*(t-ta);'
'    else'
'        dr = T-t; s = dist - 0.5*amax*dr^2;'
'    end'
'    s = min(max(s,0),dist);'
'    n = n+1;'
'    wps(n,1)=p0x+s*ux; wps(n,2)=p0y+s*uy; wps(n,3)=p0z+s*uz;'
'    t = t+dt_s;'
'end'
'if n > 0'
'    wps(n,:) = [p1x p1y p1z];'
'end'
'end'
}, newline);

%% ═══════════════════════════════════════════════════════════════════
%% 7. States
%% ═══════════════════════════════════════════════════════════════════

% Layout (pixels inside chart canvas)
% Row 1 (top):   IDLE → MOVING_TO_PICK → GRASPING
%                              ↓(err)          ↓
% Row 2 (bot):   RESETTING ← DROPPING ← MOVING_TO_PLACE

RW=170; MW=210; SH=110; HG=50; VG=70;
c1=35;          c2=c1+RW+HG;    c3=c2+MW+HG;
r1=55;          r2=r1+SH+VG;

% ── IDLE ─────────────────────────────────────────────────────────────
s_idle = Stateflow.State(ch);
s_idle.Name     = 'IDLE';
s_idle.Position = [c1, r1, RW, SH];
s_idle.LabelString = strjoin({
'IDLE'
'entry:'
'  state_id    = 0;'
'  busy        = 0;'
'  traj_done   = 0;'
'  traj_failed = 0;'
'  gripper_out = 0;'
'do:'
'  prev_trigger = trigger;'
}, newline);

% ── MOVING_TO_PICK ────────────────────────────────────────────────────
s_mtp = Stateflow.State(ch);
s_mtp.Name     = 'MOVING_TO_PICK';
s_mtp.Position = [c2, r1, MW, SH];
s_mtp.LabelString = strjoin({
'MOVING_TO_PICK'
'entry:'
'  state_id  = 1;'
'  busy      = 1;'
'  traj_done = 0; traj_failed = 0;'
'  wp_idx = 1; wp_count = 0;'
'  V=80; A=200; DT=0.05;'
'  TZ = min([pick_z, place_z, -400]);'
'  [sg,n]=gen_traj_seg(0,0,-350, 0,0,TZ,          V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'  [sg,n]=gen_traj_seg(0,0,TZ,   pick_x,pick_y,TZ, V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'  [sg,n]=gen_traj_seg(pick_x,pick_y,TZ, pick_x,pick_y,pick_z, V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'do:'
'  if wp_idx <= wp_count'
'    [st,t1,t2,t3]=delta_ik(traj(wp_idx,1),traj(wp_idx,2),traj(wp_idx,3));'
'    if st==0'
'      theta1_out=t1*pi/180; theta2_out=t2*pi/180; theta3_out=t3*pi/180;'
'    else'
'      traj_failed=1;'
'    end'
'    last_x=traj(wp_idx,1); last_y=traj(wp_idx,2); last_z=traj(wp_idx,3);'
'    wp_idx=wp_idx+1;'
'  else'
'    traj_done=1;'
'  end'
}, newline);

% ── GRASPING ──────────────────────────────────────────────────────────
s_grasp = Stateflow.State(ch);
s_grasp.Name     = 'GRASPING';
s_grasp.Position = [c3, r1, RW, SH];
s_grasp.LabelString = strjoin({
'GRASPING'
'entry:'
'  state_id    = 2;'
'  gripper_out = 1;'
'  settle_timer = sim_t + 0.5;'
}, newline);

% ── MOVING_TO_PLACE ───────────────────────────────────────────────────
s_mpl = Stateflow.State(ch);
s_mpl.Name     = 'MOVING_TO_PLACE';
s_mpl.Position = [c3, r2, MW, SH];
s_mpl.LabelString = strjoin({
'MOVING_TO_PLACE'
'entry:'
'  state_id  = 3;'
'  traj_done = 0; traj_failed = 0;'
'  wp_idx = 1; wp_count = 0;'
'  V=80; A=200; DT=0.05;'
'  TZ = min([pick_z, place_z, -400]);'
'  [sg,n]=gen_traj_seg(pick_x,pick_y,pick_z,     pick_x,pick_y,TZ,    V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'  [sg,n]=gen_traj_seg(pick_x,pick_y,TZ,          0,0,TZ,              V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'  [sg,n]=gen_traj_seg(0,0,TZ,                    place_x,place_y,TZ,  V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'  [sg,n]=gen_traj_seg(place_x,place_y,TZ, place_x,place_y,place_z,   V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'do:'
'  if wp_idx <= wp_count'
'    [st,t1,t2,t3]=delta_ik(traj(wp_idx,1),traj(wp_idx,2),traj(wp_idx,3));'
'    if st==0'
'      theta1_out=t1*pi/180; theta2_out=t2*pi/180; theta3_out=t3*pi/180;'
'    else'
'      traj_failed=1;'
'    end'
'    last_x=traj(wp_idx,1); last_y=traj(wp_idx,2); last_z=traj(wp_idx,3);'
'    wp_idx=wp_idx+1;'
'  else'
'    traj_done=1;'
'  end'
}, newline);

% ── DROPPING ──────────────────────────────────────────────────────────
s_drop = Stateflow.State(ch);
s_drop.Name     = 'DROPPING';
s_drop.Position = [c2, r2, MW, SH];
s_drop.LabelString = strjoin({
'DROPPING'
'entry:'
'  state_id    = 4;'
'  gripper_out = 0;'
'  settle_timer = sim_t + 0.5;'
}, newline);

% ── RESETTING ─────────────────────────────────────────────────────────
s_rst = Stateflow.State(ch);
s_rst.Name     = 'RESETTING';
s_rst.Position = [c1, r2, RW, SH];
s_rst.LabelString = strjoin({
'RESETTING'
'entry:'
'  state_id  = 5;'
'  traj_done = 0; traj_failed = 0;'
'  wp_idx = 1; wp_count = 0;'
'  V=80; A=200; DT=0.05;'
'  TZ = -400;'
'  gripper_out = 0;'
'  [sg,n]=gen_traj_seg(last_x,last_y,last_z,   last_x,last_y,TZ, V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'  [sg,n]=gen_traj_seg(last_x,last_y,TZ,        0,0,TZ,           V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'  [sg,n]=gen_traj_seg(0,0,TZ,                  0,0,-350,          V,A,DT); traj(wp_count+1:wp_count+n,:)=sg(1:n,:); wp_count=wp_count+n;'
'do:'
'  if wp_idx <= wp_count'
'    [st,t1,t2,t3]=delta_ik(traj(wp_idx,1),traj(wp_idx,2),traj(wp_idx,3));'
'    if st==0'
'      theta1_out=t1*pi/180; theta2_out=t2*pi/180; theta3_out=t3*pi/180;'
'    else'
'      traj_failed=1;'
'    end'
'    last_x=traj(wp_idx,1); last_y=traj(wp_idx,2); last_z=traj(wp_idx,3);'
'    wp_idx=wp_idx+1;'
'  else'
'    traj_done=1;'
'  end'
}, newline);

%% ═══════════════════════════════════════════════════════════════════
%% 8. Default transition (chart entry → IDLE)
%% ═══════════════════════════════════════════════════════════════════
t0 = Stateflow.Transition(ch);
t0.Destination       = s_idle;
t0.DestinationOClock = 9;

%% ═══════════════════════════════════════════════════════════════════
%% 9. Normal-flow transitions
%% ═══════════════════════════════════════════════════════════════════

% IDLE ──► MOVING_TO_PICK  (rising edge on trigger, not busy)
t1 = Stateflow.Transition(ch);
t1.Source            = s_idle;
t1.Destination       = s_mtp;
t1.LabelString       = '[trigger>0.5 && prev_trigger<0.5 && ~busy]';
t1.SourceOClock      = 3;
t1.DestinationOClock = 9;

% MOVING_TO_PICK ──► GRASPING
t2 = Stateflow.Transition(ch);
t2.Source            = s_mtp;
t2.Destination       = s_grasp;
t2.LabelString       = '[traj_done && ~traj_failed]';
t2.SourceOClock      = 3;
t2.DestinationOClock = 9;

% GRASPING ──► MOVING_TO_PLACE  (gripper settle complete)
t3 = Stateflow.Transition(ch);
t3.Source            = s_grasp;
t3.Destination       = s_mpl;
t3.LabelString       = '[sim_t >= settle_timer]';
t3.SourceOClock      = 6;
t3.DestinationOClock = 12;

% MOVING_TO_PLACE ──► DROPPING
t4 = Stateflow.Transition(ch);
t4.Source            = s_mpl;
t4.Destination       = s_drop;
t4.LabelString       = '[traj_done && ~traj_failed]';
t4.SourceOClock      = 9;
t4.DestinationOClock = 3;

% DROPPING ──► RESETTING  (gripper settle complete)
t5 = Stateflow.Transition(ch);
t5.Source            = s_drop;
t5.Destination       = s_rst;
t5.LabelString       = '[sim_t >= settle_timer]';
t5.SourceOClock      = 9;
t5.DestinationOClock = 3;

% RESETTING ──► IDLE
t6 = Stateflow.Transition(ch);
t6.Source            = s_rst;
t6.Destination       = s_idle;
t6.LabelString       = '[traj_done]';
t6.SourceOClock      = 12;
t6.DestinationOClock = 6;

%% ═══════════════════════════════════════════════════════════════════
%% 10. Error / abort transitions (mirrors _run_one finally-block)
%%     Safety: open gripper and go straight to RESETTING.
%% ═══════════════════════════════════════════════════════════════════

% MOVING_TO_PICK ──► RESETTING  (IK failure mid-pick)
t7 = Stateflow.Transition(ch);
t7.Source            = s_mtp;
t7.Destination       = s_rst;
t7.LabelString       = '[traj_failed] {gripper_out=0;}';
t7.SourceOClock      = 6;
t7.DestinationOClock = 12;

% MOVING_TO_PLACE ──► RESETTING  (IK failure mid-place)
t8 = Stateflow.Transition(ch);
t8.Source            = s_mpl;
t8.Destination       = s_rst;
t8.LabelString       = '[traj_failed] {gripper_out=0;}';
t8.SourceOClock      = 12;
t8.DestinationOClock = 6;

%% ═══════════════════════════════════════════════════════════════════
%% 11. Save
%% ═══════════════════════════════════════════════════════════════════
save_system(MDL, [MDL '.slx']);

fprintf('\n========================================\n');
fprintf('  Saved: %s/%s.slx\n', pwd, MDL);
fprintf('========================================\n');
fprintf('\nSimulation setup\n');
fprintf('  Sample time : 0.05 s  (20 Hz, matches trajectory dt)\n');
fprintf('  Stop time   : 30 s\n');
fprintf('  Trigger     : Pulse at t=2 s\n');
fprintf('\nChart inputs  (ports 1–8)\n');
fprintf('  1: trigger       — rising edge starts a pick cycle\n');
fprintf('  2–4: pick_x/y/z  — pick position in mm\n');
fprintf('  5–7: place_x/y/z — place position in mm\n');
fprintf('  8: sim_t         — simulation time (for gripper settle)\n');
fprintf('\nChart outputs (ports 1–5)\n');
fprintf('  1: state_id      — 0=IDLE 1=MV_PICK 2=GRASP 3=MV_PLACE 4=DROP 5=RESET\n');
fprintf('  2: theta1_out    — motor 1 angle cmd (rad)\n');
fprintf('  3: theta2_out    — motor 2 angle cmd (rad)\n');
fprintf('  4: theta3_out    — motor 3 angle cmd (rad)\n');
fprintf('  5: gripper_out   — 1=closed (pick), 0=open (release)\n');
fprintf('\nFSM states\n');
fprintf('  IDLE            — waits for trigger rising-edge\n');
fprintf('  MOVING_TO_PICK  — traj: HOME→home_t→pick_t→pick_xyz  (3 segs, trapz)\n');
fprintf('  GRASPING        — gripper CLOSE, 0.5 s settle\n');
fprintf('  MOVING_TO_PLACE — traj: pick_xyz→pick_t→home_t→place_t→place_xyz  (4 segs)\n');
fprintf('  DROPPING        — gripper OPEN, 0.5 s settle\n');
fprintf('  RESETTING       — traj: last_pos→cur_t→home_t→HOME  (3 segs)\n');
fprintf('\nTo run:  sim(''%s'')\n', MDL);
fprintf('To open: open_system(''%s'')\n\n', MDL);
