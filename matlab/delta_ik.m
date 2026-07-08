function [theta1, theta2, theta3, valid] = delta_ik(x0, y0, z0)
% DELTA_IK  Inverse kinematics for the delta robot.
%
%   [theta1, theta2, theta3, valid] = delta_ik(x0, y0, z0)
%
%   Inputs  — x0, y0, z0 in mm, robot base frame (z is negative below base)
%   Outputs — theta1/2/3 in degrees; valid=true when a solution exists
%
%   Parameters match Python fk_ik.py exactly:
%     e  = 35  mm   end-effector platform triangle side
%     f  = 157 mm   base frame triangle side
%     re = 400 mm   lower arm (forearm) length
%     rf = 200 mm   upper arm (bicep)  length

e  = 35.0;
f  = 157.0;
re = 400.0;
rf = 200.0;

sin120 = sqrt(3.0) / 2.0;
cos120 = -0.5;

[s1, theta1] = calc_angle_yz(x0,                    y0,                    z0, e, f, re, rf);
[s2, theta2] = calc_angle_yz(x0*cos120 + y0*sin120,  y0*cos120 - x0*sin120, z0, e, f, re, rf);
[s3, theta3] = calc_angle_yz(x0*cos120 - y0*sin120,  y0*cos120 + x0*sin120, z0, e, f, re, rf);

if s1 ~= 0 || s2 ~= 0 || s3 ~= 0 || theta1 < 0 || theta2 < 0 || theta3 < 0
    theta1 = 0.0;  theta2 = 0.0;  theta3 = 0.0;
    valid  = false;
else
    valid  = true;
end

end % delta_ik


% ── helpers ───────────────────────────────────────────────────────────────────

function [status, theta] = calc_angle_yz(x0, y0, z0, e, f, re, rf)
tan30 = 1.0 / sqrt(3.0);

y1 = -0.5 * tan30 * f;
y0 = y0 - 0.5 * tan30 * e;

a = (x0^2 + y0^2 + z0^2 + rf^2 - re^2 - y1^2) / (2.0 * z0);
b = (y1 - y0) / z0;

d = -(a + b*y1)^2 + rf * (b^2 * rf + rf);
if d < 0
    status = -1;
    theta  = 0.0;
    return;
end

yj    = (y1 - a*b - sqrt(d)) / (b^2 + 1.0);
zj    = a + b * yj;
theta = atan2d(-zj, y1 - yj);
if yj > y1
    theta = theta + 180.0;
end
status = 0;
end % calc_angle_yz
