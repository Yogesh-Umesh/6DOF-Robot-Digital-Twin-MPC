%% robot_part2_mpc.m
% Part 2 — MPC Control on Puma560 6-DOF Robot
% MATLAB R2026a

clc; close all;

%% Load robot and trajectory
robot = loadrobot('puma560', 'DataFormat', 'row');

tf=5; dt=0.01; t=(0:dt:tf)';
N = length(t);

q_start = deg2rad([0,   0,   0,  0,  0,  0]);
q_end   = deg2rad([45, -30,  60, 30, 45, -30]);

q_traj  = zeros(N,6);
for j = 1:6
    q0=q_start(j); qf=q_end(j);
    a2=3*(qf-q0)/tf^2; a3=-2*(qf-q0)/tf^3;
    q_traj(:,j) = q0 + a2*t.^2 + a3*t.^3;
end

disp('Trajectory loaded.');

%% Linearize robot at nominal config
q_nom  = deg2rad([22.5, -15, 30, 15, 22.5, -15]);
qd_nom = zeros(1,6);

% Mass matrix at nominal
M_nom = massMatrix(robot, q_nom) + 0.001*eye(6);
Minv  = inv(M_nom);

% Damping
D = diag([0.8 0.8 0.6 0.4 0.4 0.3]);

% State space: x = [q1..q6, qd1..qd6] (12 states)
n = 12; m = 6;
A = [zeros(6)  eye(6);
     zeros(6) -Minv*D];
B = [zeros(6); Minv];
C = [eye(6) zeros(6)];   % observe joint positions
Ds = zeros(6, 6);

sys   = ss(A, B, C, Ds);
Ts    = 0.01;
sys_d = c2d(sys, Ts);

fprintf('System: %d states, %d inputs, %d outputs\n', n, m, size(C,1));

%% Design MPC
mpcObj = mpc(sys_d, Ts);
mpcObj.PredictionHorizon  = 20;
mpcObj.ControlHorizon     = 5;
mpcObj.Weights.OutputVariables          = ones(1,6) * 5;
mpcObj.Weights.ManipulatedVariablesRate = ones(1,6) * 0.05;
mpcObj.Weights.ManipulatedVariables     = zeros(1,6);

% Torque limits
for j = 1:6
    mpcObj.MV(j).Min = -150;
    mpcObj.MV(j).Max =  150;
end

disp('MPC object created.');

%% Simulate MPC
x = zeros(12,1);
q1_mpc = zeros(N,6);
mpcState = mpcstate(mpcObj);

disp('Running MPC simulation...');

for k = 1:N
    y = C * x;
    q1_mpc(k,:) = rad2deg(y');

    ref_k = q_traj(k,:)';
    u = mpcmove(mpcObj, mpcState, y, ref_k);

    x = sys_d.A * x + sys_d.B * u;
end

disp('MPC simulation complete.');

final_err_mpc = q_traj(end,:) - deg2rad(q1_mpc(end,:));
fprintf('Final MPC errors (deg): J1=%.3f J2=%.3f J3=%.3f J4=%.3f J5=%.3f J6=%.3f\n',...
    rad2deg(final_err_mpc(1)),rad2deg(final_err_mpc(2)),rad2deg(final_err_mpc(3)),...
    rad2deg(final_err_mpc(4)),rad2deg(final_err_mpc(5)),rad2deg(final_err_mpc(6)));

q_traj_deg = rad2deg(q_traj);
max_err_mpc = max(abs(q_traj_deg(:) - q1_mpc(:)));
fprintf('Max MPC tracking error: %.4f deg\n', max_err_mpc);

%% Plot MPC tracking
figure('Name','MPC Tracking 6DOF','Color','white','Position',[50 50 1400 700]);
jointNames = {'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'};
for j = 1:6
    subplot(2,3,j);
    plot(t, q_traj_deg(:,j),  'r--', 'LineWidth', 2); hold on;
    plot(t, q1_mpc(:,j),      'g-',  'LineWidth', 1.5);
    title(sprintf('%s MPC Tracking', jointNames{j}));
    xlabel('Time (s)'); ylabel('Angle (deg)');
    legend('Reference','MPC','Location','northwest');
    grid on;
end
sgtitle('Puma560 MPC Controller 6-DOF Trajectory Tracking','FontSize',13,'FontWeight','bold');

%% Plot MPC error
figure('Name','MPC Error 6DOF','Color','white','Position',[50 50 1400 700]);
colors = lines(6);
for j = 1:6
    subplot(2,3,j);
    plot(t, q_traj_deg(:,j) - q1_mpc(:,j), 'Color', colors(j,:), 'LineWidth', 1.5);
    title(sprintf('%s MPC Error', jointNames{j}));
    xlabel('Time (s)'); ylabel('Error (deg)');
    grid on; yline(0,'k--');
end
sgtitle('Puma560 MPC Tracking Error','FontSize',13,'FontWeight','bold');

%% PID vs MPC comparison table
pid_max_err = 2.0305;
fprintf('\n===== PART 2: PID vs MPC COMPARISON =====\n');
fprintf('%-25s %-15s %-15s\n','Metric','PID','MPC');
fprintf('%-25s %-15.4f %-15.4f\n','Max Error (deg)', pid_max_err, max_err_mpc);
fprintf('%-25s %-15.3f %-15.3f\n','Final J1 error', -0.006, rad2deg(final_err_mpc(1)));
fprintf('%-25s %-15.3f %-15.3f\n','Final J3 error', -1.369, rad2deg(final_err_mpc(3)));
fprintf('==========================================\n');

%% Save plots
figs = findall(0,'Type','figure');
for i = 1:length(figs)
    fname = sprintf('C:\\Users\\yoges\\Desktop\\Part2_MPC_Fig%d.png', i);
    saveas(figs(i), fname);
    fprintf('Saved: %s\n', fname);
end
disp('All Part 2 MPC plots saved.');