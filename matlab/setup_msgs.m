% setup_msgs.m  —  Run this ONCE in MATLAB to generate custom message classes.
%
% After it finishes:
%   1. Run:  clear classes
%   2. Verify: ros2msg("custom_messages/DeltaTarget")
%
% You only need to re-run this if the .msg files change.

pkg_dir = '/home/s4mb4th/delta_ws/src/can_driver';
fprintf('Generating ROS2 custom messages from:\n  %s\n\n', pkg_dir);
ros2genmsg(pkg_dir);

out_dir = fullfile(pkg_dir, 'matlab_msg_gen', 'ros2');
if isfolder(out_dir)
    addpath(genpath(out_dir));
    savepath;
    fprintf('\nPath saved. Now run:  clear classes\n');
else
    fprintf('\nDone. Locate the generated folder and add it to your MATLAB path.\n');
end
