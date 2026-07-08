% delta_ik_node.m  —  MATLAB side of the delta robot IK bridge.
%
% This script subscribes to /delta/matlab/target_xyz (DeltaTarget),
% solves inverse kinematics using delta_ik.m, and publishes the joint
% angles to /delta/matlab/joint_thetas (DeltaJointAngles).
%
% PRE-REQUISITES (one-time):
%   1. run setup_msgs.m        % generate custom message classes
%   2. clear classes           % reload message definitions
%   3. Verify: ros2msg("custom_messages/DeltaTarget")
%
% RUN:
%   cd /home/s4mb4th/delta_ws/matlab
%   delta_ik_node
%
% STOP:  Ctrl+C

DOMAIN_ID    = 69;
RECV_TIMEOUT = 10;    % seconds to wait for each target

fprintf('=== Delta IK ROS2 node ===\n');
fprintf('Domain ID  : %d\n', DOMAIN_ID);
fprintf('Subscribe  : /delta/matlab/target_xyz\n');
fprintf('Publish    : /delta/matlab/joint_thetas\n');
fprintf('Press Ctrl+C to stop.\n\n');

% ── create node ───────────────────────────────────────────────────────────────
setenv('ROS_DOMAIN_ID', num2str(DOMAIN_ID));

try
    % R2023a+: pass DomainID directly
    node = ros2node('/matlab_ik', 'DomainID', DOMAIN_ID);
catch
    % R2022b fallback — domain ID comes from env var set above
    node = ros2node('/matlab_ik');
end

% ── publisher / subscriber ────────────────────────────────────────────────────
sub = ros2subscriber(node, '/delta/matlab/target_xyz', ...
    'custom_messages/DeltaTarget', ...
    'Reliability', 'reliable', 'History', 'keeplast', 'Depth', 10);

pub = ros2publisher(node, '/delta/matlab/joint_thetas', ...
    'custom_messages/DeltaJointAngles', ...
    'Reliability', 'reliable', 'History', 'keeplast', 'Depth', 10);

reply_tmpl = ros2message('custom_messages/DeltaJointAngles');

fprintf('[ready] Waiting for targets...\n\n');

% ── main loop ─────────────────────────────────────────────────────────────────
while true
    % --- receive target (blocks up to RECV_TIMEOUT s) -------------------------
    try
        tgt = receive(sub, RECV_TIMEOUT);
    catch ME
        if contains(ME.message, 'timeout', 'IgnoreCase', true)
            fprintf('[wait]  No target in %.0f s — still listening...\n', RECV_TIMEOUT);
            continue;
        end
        rethrow(ME);
    end

    x = tgt.x_mm;
    y = tgt.y_mm;
    z = tgt.z_mm;
    fprintf('[target] x=%7.2f  y=%7.2f  z=%7.2f mm\n', x, y, z);

    % --- solve IK -------------------------------------------------------------
    [t1, t2, t3, ik_ok] = delta_ik(x, y, z);

    % --- publish reply --------------------------------------------------------
    reply              = reply_tmpl;
    reply.theta1_deg   = t1;
    reply.theta2_deg   = t2;
    reply.theta3_deg   = t3;
    reply.ik_valid     = ik_ok;
    send(pub, reply);

    if ik_ok
        fprintf('[reply]  theta=(%.2f, %.2f, %.2f) deg  OK\n', t1, t2, t3);
    else
        fprintf('[reply]  IK FAILED for (%.1f, %.1f, %.1f) mm\n', x, y, z);
    end
    fprintf('\n');
end
