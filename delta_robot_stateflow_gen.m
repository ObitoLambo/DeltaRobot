% delta_robot_stateflow_gen.m
%
% Generates  delta_robot_fsm.slx  — a Stateflow chart of the delta robot
% blind pick-and-place finite state machine.
%
% Requirements: MATLAB R2020b or later, Simulink, Stateflow toolbox.
%
% Usage (MATLAB command window):
%   >> run('delta_robot_stateflow_gen.m')
%
% Source reference:
%   src/delta_main_app/delta_main_app/blind_pick_place.py  (BlindPickAndPlace._run_one)
%
% ─── State flow ───────────────────────────────────────────────────────────────
%
%          [trigger_received]          [traj_ok]         [gripper_closed]
%  ●──► IDLE ──────────────► MOVING_TO_PICK ──────► GRASPING ─────────────┐
%        ▲                         │ [traj_failed]                          │
%        │                         └──────────────────────────►┐           ▼
%        │  [home_reached]          [traj_failed]              RESETTING  MOVING_TO_PLACE
%        └────────────────── RESETTING ◄────────────────────────┘  ◄──────┘
%                                   ▲           [gripper_open]       [traj_ok]
%                                   └──────────────────── DROPPING ──┘
%
% ─── Transit-Z waypoint strategy ─────────────────────────────────────────────
%
%   Every motion state routes through a safe transit depth (Z ≤ –400 mm)
%   via the centre column (HOME_X=0, HOME_Y=0) before moving laterally.
%
%   MOVING_TO_PICK   : HOME → home_t → pick_t → pick_xyz
%   MOVING_TO_PLACE  : pick_xyz → pick_t → home_t → place_t → place_xyz
%   RESETTING        : current_pos → cur_t → home_t → HOME(0,0,–350)
%
% ─────────────────────────────────────────────────────────────────────────────

MDL = 'delta_robot_fsm';

%% 1. Create / overwrite model ─────────────────────────────────────────────────
if bdIsLoaded(MDL)
    close_system(MDL, 0);
end
if exist([MDL '.slx'], 'file')
    delete([MDL '.slx']);
end

new_system(MDL);
open_system(MDL);
set_param(MDL, 'StopTime', 'inf', 'SolverType', 'Fixed-step');

%% 2. Add Stateflow Chart block ────────────────────────────────────────────────
chartBlk = [MDL '/Delta_Robot_FSM'];
add_block('sflib/Chart', chartBlk, 'Position', [30 30 870 560]);

%% 3. Get Stateflow chart handle ───────────────────────────────────────────────
rt    = sfroot();
mdlSF = rt.find('-isa', 'Simulink.BlockDiagram', '-and', 'Name', MDL);
ch    = mdlSF.find('-isa', 'Stateflow.Chart');

ch.StateMachineType = 'Classic';

%% 4. Layout constants (pixels inside the chart canvas) ───────────────────────
%
%   Row 1 (top):    IDLE  ──►  MOVING_TO_PICK  ──►  GRASPING
%                                   │(err)                │
%   Row 2 (bot):  RESETTING ◄── DROPPING  ◄──  MOVING_TO_PLACE
%
RW  = 165;   % regular-state width
MW  = 210;   % motion-state width
SH  = 95;    % state height
HG  = 45;    % horizontal gap between states
VG  = 65;    % vertical gap between rows

c1 = 35;              % col-1 x (IDLE / RESETTING)
c2 = c1 + RW + HG;   % col-2 x (MOVING_TO_PICK / DROPPING)
c3 = c2 + MW + HG;   % col-3 x (GRASPING / MOVING_TO_PLACE)

r1 = 55;              % row-1 y (top)
r2 = r1 + SH + VG;   % row-2 y (bottom)

%% 5. Create states ────────────────────────────────────────────────────────────

% ── IDLE ──────────────────────────────────────────────────────────────────────
s_idle = Stateflow.State(ch);
s_idle.Name = 'IDLE';
s_idle.Position = [c1, r1, RW, SH];
s_idle.LabelString = sprintf([ ...
    'IDLE\n' ...
    'entry: publish_state(''IDLE'');\n' ...
    '       busy = false;']);

% ── MOVING_TO_PICK ────────────────────────────────────────────────────────────
s_mtp = Stateflow.State(ch);
s_mtp.Name = 'MOVING_TO_PICK';
s_mtp.Position = [c2, r1, MW, SH];
s_mtp.LabelString = sprintf([ ...
    'MOVING_TO_PICK\n' ...
    'entry: publish_state(''MOVING_TO_PICK'');\n' ...
    '       busy = true;\n' ...
    'do: execute_traj(\n' ...
    '      HOME->home_t->pick_t->pick_xyz);']);

% ── GRASPING ──────────────────────────────────────────────────────────────────
s_grasp = Stateflow.State(ch);
s_grasp.Name = 'GRASPING';
s_grasp.Position = [c3, r1, RW, SH];
s_grasp.LabelString = sprintf([ ...
    'GRASPING\n' ...
    'entry: publish_state(''GRASPING'');\n' ...
    'do:    gripper_close();\n' ...
    '       wait(close_settle_s);']);

% ── MOVING_TO_PLACE ───────────────────────────────────────────────────────────
s_mplace = Stateflow.State(ch);
s_mplace.Name = 'MOVING_TO_PLACE';
s_mplace.Position = [c3, r2, MW, SH];
s_mplace.LabelString = sprintf([ ...
    'MOVING_TO_PLACE\n' ...
    'entry: publish_state(''MOVING_TO_PLACE'');\n' ...
    'do: execute_traj(\n' ...
    '      pick_xyz->pick_t->home_t\n' ...
    '             ->place_t->place_xyz);']);

% ── DROPPING ──────────────────────────────────────────────────────────────────
s_drop = Stateflow.State(ch);
s_drop.Name = 'DROPPING';
s_drop.Position = [c2, r2, MW, SH];
s_drop.LabelString = sprintf([ ...
    'DROPPING\n' ...
    'entry: publish_state(''DROPPING'');\n' ...
    'do:    gripper_open();\n' ...
    '       wait(open_settle_s);']);

% ── RESETTING ─────────────────────────────────────────────────────────────────
s_reset = Stateflow.State(ch);
s_reset.Name = 'RESETTING';
s_reset.Position = [c1, r2, RW, SH];
s_reset.LabelString = sprintf([ ...
    'RESETTING\n' ...
    'entry: publish_state(''RESETTING'');\n' ...
    'do: execute_traj(\n' ...
    '      cur_pos->cur_t->home_t->HOME);']);

%% 6. Default transition (chart entry → IDLE) ──────────────────────────────────
t0 = Stateflow.Transition(ch);
t0.Destination       = s_idle;
t0.DestinationOClock = 9;    % arrow arrives from the left

%% 7. Normal-flow transitions ─────────────────────────────────────────────────

% IDLE ──► MOVING_TO_PICK
t1 = Stateflow.Transition(ch);
t1.Source            = s_idle;
t1.Destination       = s_mtp;
t1.LabelString       = '[trigger_received]';
t1.SourceOClock      = 3;    % exit right side of IDLE
t1.DestinationOClock = 9;    % enter left side of MOVING_TO_PICK

% MOVING_TO_PICK ──► GRASPING
t2 = Stateflow.Transition(ch);
t2.Source            = s_mtp;
t2.Destination       = s_grasp;
t2.LabelString       = '[traj_ok]';
t2.SourceOClock      = 3;
t2.DestinationOClock = 9;

% GRASPING ──► MOVING_TO_PLACE  (right col, top to bottom)
t3 = Stateflow.Transition(ch);
t3.Source            = s_grasp;
t3.Destination       = s_mplace;
t3.LabelString       = '[gripper_closed]';
t3.SourceOClock      = 6;    % exit bottom of GRASPING
t3.DestinationOClock = 12;   % enter top of MOVING_TO_PLACE

% MOVING_TO_PLACE ──► DROPPING
t4 = Stateflow.Transition(ch);
t4.Source            = s_mplace;
t4.Destination       = s_drop;
t4.LabelString       = '[traj_ok]';
t4.SourceOClock      = 9;    % exit left of MOVING_TO_PLACE
t4.DestinationOClock = 3;    % enter right of DROPPING

% DROPPING ──► RESETTING
t5 = Stateflow.Transition(ch);
t5.Source            = s_drop;
t5.Destination       = s_reset;
t5.LabelString       = '[gripper_open]';
t5.SourceOClock      = 9;
t5.DestinationOClock = 3;

% RESETTING ──► IDLE  (left col, bottom to top)
t6 = Stateflow.Transition(ch);
t6.Source            = s_reset;
t6.Destination       = s_idle;
t6.LabelString       = '[home_reached]';
t6.SourceOClock      = 12;   % exit top of RESETTING
t6.DestinationOClock = 6;    % enter bottom of IDLE

%% 8. Error / abort transitions ────────────────────────────────────────────────
% If a trajectory raises an exception, the finally-block in _run_one()
% always executes RESETTING then returns to IDLE.
% The gripper is opened as a safety action on the transition.

% MOVING_TO_PICK ──► RESETTING  (diagonal, error path)
t7 = Stateflow.Transition(ch);
t7.Source      = s_mtp;
t7.Destination = s_reset;
t7.LabelString = '[traj_failed] / gripper_open();';
t7.SourceOClock      = 6;    % exit bottom of MOVING_TO_PICK
t7.DestinationOClock = 12;   % enter top of RESETTING

% MOVING_TO_PLACE ──► RESETTING  (same column, above it)
t8 = Stateflow.Transition(ch);
t8.Source      = s_mplace;
t8.Destination = s_reset;
t8.LabelString = '[traj_failed] / gripper_open();';
t8.SourceOClock      = 9;    % exit left of MOVING_TO_PLACE (overlaps t4, offset by MATLAB)
t8.DestinationOClock = 3;    % enter right of RESETTING

%% 9. Save ─────────────────────────────────────────────────────────────────────
save_system(MDL, [MDL '.slx']);
fprintf('\n========================================\n');
fprintf('  Saved: %s/%s.slx\n', pwd, MDL);
fprintf('========================================\n');
fprintf('\nState summary:\n');
fprintf('  IDLE            — waiting for /delta/trigger_pick or /delta/blind_target\n');
fprintf('  MOVING_TO_PICK  — trapezoidal traj: HOME->home_t->pick_t->pick_xyz\n');
fprintf('  GRASPING        — pneumatic solenoid CLOSE (CAN id=0x04), wait settle\n');
fprintf('  MOVING_TO_PLACE — trapezoidal traj: pick_xyz->...->place_xyz\n');
fprintf('  DROPPING        — pneumatic solenoid OPEN, wait settle\n');
fprintf('  RESETTING       — trapezoidal traj: current_pos->...->HOME(0,0,-350mm)\n');
fprintf('\nopen_system(''%s'') to view.\n', MDL);
