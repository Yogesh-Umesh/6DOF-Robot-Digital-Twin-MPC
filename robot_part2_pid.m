%% robot_part2_pid.m
% Part 2 — PID Control on Puma560 6-DOF Robot
% MATLAB R2026a

clc; close all;

%% Load robot
robot = loadrobot('puma560', 'DataFormat', 'row');

%% Trajectory
tf=5; dt=0.01; t=(0:dt:tf)';
N = length(t);

q_start = deg2rad([0,   0,   0,  0,  0,  0]);
q_end   = deg2rad([45, -30,  60, 30, 45, -30]);

q_traj  = zeros(N,6);
qd_traj = zeros(N,6);

for j = 1:6
    q0=q_start(j); qf=q_end(j);
    a2=3*(qf-q0)/tf^2; a3=-2*(qf-q0)/tf^3;
    q_traj(:,j)  = q0 + a2*t.^2 + a3*t.^3;
    qd_traj(:,j) = 2*a2*t + 3*a3*t.^2;
end

disp('Trajectory generated.');

%% PID Gains
Kp = [80  80  60  40  40  30];
Ki = [2   2   1.5 1   1   0.8];
Kd = [10  10  8   6   6   5];

%% Simulation
Ts = dt;
q_actual  = zeros(N,6);
qd_actual = zeros(N,6);
u_pid     = zeros(N,6);

q  = q_start(:)';
qd = zeros(1,6);
int_e  = zeros(1,6);
prev_e = zeros(1,6);
lims = deg2rad([-180 180; -90 90; -90 90; -90 90; -90 90; -90 90]);

disp('Running PID simulation...');

for k = 1:N
    q  = q(:)';
    qd = qd(:)';

    if any(isnan(q)) || any(isnan(qd))
        fprintf('NaN at step %d\n', k); break;
    end

    q_actual(k,:)  = q;
    qd_actual(k,:) = qd;

    e     = q_traj(k,:) - q;
    de    = (e - prev_e) / Ts;
    int_e = int_e + e * Ts;
    u     = Kp.*e + Ki.*int_e + Kd.*de;
    u     = max(min(u, 150), -150);

    try
        g_torque = gravityTorque(robot, q); g_torque = g_torque(:)';
        tau      = u + g_torque;
        M        = massMatrix(robot, q) + 0.001*eye(6);
        C        = velocityProduct(robot, q, qd); C = C(:)';
        qdd      = (M \ (tau - g_torque - C)')';
        qdd      = max(min(qdd, 50), -50);
        qd       = qd + qdd * Ts;
        q        = q  + qd  * Ts;
        for j = 1:6
            q(j) = max(min(q(j), lims(j,2)), lims(j,1));
        end
        u_pid(k,:) = tau;
    catch ME
        fprintf('Error at step %d: %s\n', k, ME.message); break;
    end
    prev_e = e;
end

disp('PID simulation complete.');
final_err = rad2deg(q_traj(end,:) - q_actual(end,:));
fprintf('Final errors (deg): J1=%.3f J2=%.3f J3=%.3f J4=%.3f J5=%.3f J6=%.3f\n',...
    final_err(1),final_err(2),final_err(3),final_err(4),final_err(5),final_err(6));
fprintf('Max tracking error: %.4f deg\n', max(abs(rad2deg(q_traj(:) - q_actual(:)))));

%% Plot PID tracking
figure('Name','PID Tracking 6DOF','Color','white','Position',[50 50 1400 700]);
jointNames = {'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'};
colors = lines(6);
for j = 1:6
    subplot(2,3,j);
    plot(t, rad2deg(q_traj(:,j)),   'r--', 'LineWidth', 2); hold on;
    plot(t, rad2deg(q_actual(:,j)), 'b-',  'LineWidth', 1.5);
    title(sprintf('%s PID Tracking', jointNames{j}));
    xlabel('Time (s)'); ylabel('Angle (deg)');
    legend('Reference','Actual','Location','northwest');
    grid on;
end
sgtitle('Puma560 PID Controller 6-DOF Trajectory Tracking','FontSize',13,'FontWeight','bold');

%% Plot error
figure('Name','PID Error 6DOF','Color','white','Position',[50 50 1400 700]);
for j = 1:6
    subplot(2,3,j);
    plot(t, rad2deg(q_traj(:,j) - q_actual(:,j)), 'Color', colors(j,:), 'LineWidth', 1.5);
    title(sprintf('%s Error', jointNames{j}));
    xlabel('Time (s)'); ylabel('Error (deg)');
    grid on; yline(0,'k--');
end
sgtitle('Puma560 PID Tracking Error','FontSize',13,'FontWeight','bold');

%% End-effector path
ee_ref = zeros(N,3); ee_act = zeros(N,3);
for k = 1:N
    T1 = getTransform(robot, q_traj(k,:),  'link7'); ee_ref(k,:) = T1(1:3,4)';
    T2 = getTransform(robot, q_actual(k,:), 'link7'); ee_act(k,:) = T2(1:3,4)';
end

figure('Name','EE Path PID','Color','white','Position',[100 100 700 600]);
plot3(ee_ref(:,1),ee_ref(:,2),ee_ref(:,3),'r--','LineWidth',2); hold on;
plot3(ee_act(:,1),ee_act(:,2),ee_act(:,3),'b-','LineWidth',2);
plot3(ee_ref(1,1),ee_ref(1,2),ee_ref(1,3),'go','MarkerSize',12,'MarkerFaceColor','g');
plot3(ee_ref(end,1),ee_ref(end,2),ee_ref(end,3),'r*','MarkerSize',12);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-Effector Path PID Control');
legend('Reference','Actual','Start','End');
grid on; axis equal; view(45,30);

ee_err = sqrt(sum((ee_ref - ee_act).^2, 2)) * 1000;
fprintf('Max end-effector error: %.4f mm\n', max(ee_err));

%% Robot animation at key frames
figure('Name','Robot Animation','Color','white','Position',[100 100 900 600]);
frames = [1, round(N/4), round(N/2), round(3*N/4), N];
for i = 1:length(frames)
    subplot(1,5,i);
    show(robot, q_actual(frames(i),:), 'visuals','on','frames','off');
    title(sprintf('t=%.1fs', t(frames(i))));
    axis equal; grid on;
end
sgtitle('Puma560 Robot Motion under PID Control','FontSize',12,'FontWeight','bold');

%% Save plots
figs = findall(0,'Type','figure');
for i = 1:length(figs)
    fname = sprintf('C:\\Users\\yoges\\Desktop\\Part2_PID_Fig%d.png', i);
    saveas(figs(i), fname);
    fprintf('Saved: %s\n', fname);
end
disp('All Part 2 PID plots saved.');