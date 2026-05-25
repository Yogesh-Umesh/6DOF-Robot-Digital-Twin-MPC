%% robot_part2_robust.m
% Part 2 — Robustness Testing on Puma560 6-DOF Robot
% MATLAB R2026a

clc; close all;

%% Setup
robot = loadrobot('puma560', 'DataFormat', 'row');

tf=5; dt=0.01; t=(0:dt:tf)';
N = length(t); Ts = dt;

q_start = deg2rad([0,   0,   0,  0,  0,  0]);
q_end   = deg2rad([45, -30,  60, 30, 45, -30]);

q_traj = zeros(N,6);
for j = 1:6
    q0=q_start(j); qf=q_end(j);
    a2=3*(qf-q0)/tf^2; a3=-2*(qf-q0)/tf^3;
    q_traj(:,j) = q0 + a2*t.^2 + a3*t.^3;
end
q_traj_deg = rad2deg(q_traj);

% PID gains
Kp = [80  80  60  40  40  30];
Ki = [2   2   1.5 1   1   0.8];
Kd = [10  10  8   6   6   5];

% MPC setup
q_nom = deg2rad([22.5, -15, 30, 15, 22.5, -15]);
M_nom = massMatrix(robot, q_nom) + 0.001*eye(6);
Minv  = inv(M_nom);
D     = diag([0.8 0.8 0.6 0.4 0.4 0.3]);
A = [zeros(6) eye(6); zeros(6) -Minv*D];
B = [zeros(6); Minv];
C = [eye(6) zeros(6)];
sys_d = c2d(ss(A,B,C,zeros(6,6)), Ts);

mpcObj = mpc(sys_d, Ts);
mpcObj.PredictionHorizon  = 20;
mpcObj.ControlHorizon     = 5;
mpcObj.Weights.OutputVariables          = ones(1,6)*5;
mpcObj.Weights.ManipulatedVariablesRate = ones(1,6)*0.05;
mpcObj.Weights.ManipulatedVariables     = zeros(1,6);
for j=1:6; mpcObj.MV(j).Min=-150; mpcObj.MV(j).Max=150; end

lims = deg2rad([-180 180;-90 90;-90 90;-90 90;-90 90;-90 90]);

disp('Setup complete. Running robustness tests...');

%% PID simulation function (inline)
function [qa] = sim_pid(robot, q_traj, Kp, Ki, Kd, Ts, lims, noise_std, dist_torque)
    N = size(q_traj,1);
    q_start = q_traj(1,:);
    q  = q_start(:)';
    qd = zeros(1,6);
    int_e=zeros(1,6); prev_e=zeros(1,6);
    qa = zeros(N,6);
    for k=1:N
        q=q(:)'; qd=qd(:)';
        if any(isnan(q))||any(isnan(qd)); qa(k:end,:)=qa(max(1,k-1),:); break; end
        qa(k,:)=q;
        y_noisy = q + noise_std*randn(1,6);
        e  = q_traj(k,:) - y_noisy;
        de = (e-prev_e)/Ts;
        int_e = int_e + e*Ts;
        u = Kp.*e + Ki.*int_e + Kd.*de;
        u = max(min(u,150),-150) + dist_torque;
        try
            g = gravityTorque(robot,q); g=g(:)';
            tau = u+g;
            M = massMatrix(robot,q)+0.001*eye(6);
            Cv = velocityProduct(robot,q,qd); Cv=Cv(:)';
            qdd = (M\(tau-g-Cv)')';
            qdd = max(min(qdd,50),-50);
            qd=qd+qdd*Ts; q=q+qd*Ts;
            for j=1:6; q(j)=max(min(q(j),lims(j,2)),lims(j,1)); end
        catch; break; end
        prev_e=e;
    end
end

%% MPC simulation function (inline)
function [qm] = sim_mpc(mpcObj, sys_d, C, q_traj, noise_std, dist_torque, N)
    x=zeros(12,1);
    qm=zeros(N,6);
    mpcState=mpcstate(mpcObj);
    for k=1:N
        y=C*x; yn=y+noise_std*randn(6,1);
        qm(k,:)=rad2deg(y');
        u=mpcmove(mpcObj,mpcState,yn,q_traj(k,:)');
        u=u+dist_torque;
        x=sys_d.A*x+sys_d.B*u;
    end
end

%% =====================================================================
%  TEST 1 — SENSOR NOISE
%% =====================================================================
disp('Test 1 — Sensor Noise...');
noise_std = deg2rad(0.5);

qa_pid_n = sim_pid(robot,q_traj,Kp,Ki,Kd,Ts,lims,noise_std,0);
qa_pid_n_deg = rad2deg(qa_pid_n);

x=zeros(12,1); qm_n=zeros(N,6); mpcState=mpcstate(mpcObj);
for k=1:N
    y=C*x; yn=y+noise_std*randn(6,1);
    qm_n(k,:)=rad2deg(y');
    u=mpcmove(mpcObj,mpcState,yn,q_traj(k,:)');
    x=sys_d.A*x+sys_d.B*u;
end

figure('Name','Test1 Noise 6DOF','Color','white','Position',[50 50 1400 600]);
for j=1:3
    subplot(2,3,j);
    plot(t,q_traj_deg(:,j),'r--','LineWidth',2); hold on;
    plot(t,qa_pid_n_deg(:,j),'b-','LineWidth',1.5);
    title(sprintf('J%d PID+Noise',j)); xlabel('Time(s)'); ylabel('deg');
    legend('Ref','PID'); grid on;

    subplot(2,3,j+3);
    plot(t,q_traj_deg(:,j),'r--','LineWidth',2); hold on;
    plot(t,qm_n(:,j),'g-','LineWidth',1.5);
    title(sprintf('J%d MPC+Noise',j)); xlabel('Time(s)'); ylabel('deg');
    legend('Ref','MPC'); grid on;
end
sgtitle('Robustness Test 1 Sensor Noise (Joints 1 to 3)','FontSize',12,'FontWeight','bold');

err_pid_n = max(abs(q_traj_deg(:) - qa_pid_n_deg(:)));
err_mpc_n = max(abs(q_traj_deg(:) - qm_n(:)));
fprintf('Noise — PID MaxErr: %.4f deg | MPC MaxErr: %.4f deg\n', err_pid_n, err_mpc_n);

%% =====================================================================
%  TEST 2 — EXTERNAL DISTURBANCE
%% =====================================================================
disp('Test 2 — External Disturbance...');
dist = 3;

qa_pid_d = sim_pid(robot,q_traj,Kp,Ki,Kd,Ts,lims,0,dist);
qa_pid_d_deg = rad2deg(qa_pid_d);

x=zeros(12,1); qm_d=zeros(N,6); mpcState=mpcstate(mpcObj);
for k=1:N
    y=C*x;
    qm_d(k,:)=rad2deg(y');
    u=mpcmove(mpcObj,mpcState,y,q_traj(k,:)');
    u=u+dist;
    x=sys_d.A*x+sys_d.B*u;
end

figure('Name','Test2 Disturbance 6DOF','Color','white','Position',[50 50 1400 600]);
for j=1:3
    subplot(2,3,j);
    plot(t,q_traj_deg(:,j),'r--','LineWidth',2); hold on;
    plot(t,qa_pid_d_deg(:,j),'b-','LineWidth',1.5);
    title(sprintf('J%d PID+Dist',j)); xlabel('Time(s)'); ylabel('deg');
    legend('Ref','PID'); grid on;

    subplot(2,3,j+3);
    plot(t,q_traj_deg(:,j),'r--','LineWidth',2); hold on;
    plot(t,qm_d(:,j),'g-','LineWidth',1.5);
    title(sprintf('J%d MPC+Dist',j)); xlabel('Time(s)'); ylabel('deg');
    legend('Ref','MPC'); grid on;
end
sgtitle('Robustness Test 2 External Disturbance (Joints 1 to 3)','FontSize',12,'FontWeight','bold');

err_pid_d = max(abs(q_traj_deg(:) - qa_pid_d_deg(:)));
err_mpc_d = max(abs(q_traj_deg(:) - qm_d(:)));
fprintf('Disturbance — PID MaxErr: %.4f deg | MPC MaxErr: %.4f deg\n', err_pid_d, err_mpc_d);

%% =====================================================================
%  TEST 3 — MASS UNCERTAINTY
%% =====================================================================
disp('Test 3 — Mass Uncertainty +20 percent...');

% Build uncertain robot
robot_u = loadrobot('puma560','DataFormat','row');
for i=1:robot_u.NumBodies
    robot_u.Bodies{i}.Mass = robot_u.Bodies{i}.Mass * 1.2;
end

qa_pid_u = sim_pid(robot_u,q_traj,Kp,Ki,Kd,Ts,lims,0,0);
qa_pid_u_deg = rad2deg(qa_pid_u);

% MPC uses nominal model — plant is uncertain
M_u   = massMatrix(robot_u,q_nom)+0.001*eye(6);
Minv_u = inv(M_u);
Au = [zeros(6) eye(6); zeros(6) -Minv_u*D];
Bu = [zeros(6); Minv_u];
sys_du = c2d(ss(Au,Bu,C,zeros(6,6)),Ts);

x=zeros(12,1); qm_u=zeros(N,6); mpcState=mpcstate(mpcObj);
for k=1:N
    y=C*x;
    qm_u(k,:)=rad2deg(y');
    u=mpcmove(mpcObj,mpcState,y,q_traj(k,:)');
    x=sys_du.A*x+sys_du.B*u;
end

figure('Name','Test3 Mass 6DOF','Color','white','Position',[50 50 1400 600]);
for j=1:3
    subplot(2,3,j);
    plot(t,q_traj_deg(:,j),'r--','LineWidth',2); hold on;
    plot(t,qa_pid_u_deg(:,j),'b-','LineWidth',1.5);
    title(sprintf('J%d PID+Mass',j)); xlabel('Time(s)'); ylabel('deg');
    legend('Ref','PID'); grid on;

    subplot(2,3,j+3);
    plot(t,q_traj_deg(:,j),'r--','LineWidth',2); hold on;
    plot(t,qm_u(:,j),'g-','LineWidth',1.5);
    title(sprintf('J%d MPC+Mass',j)); xlabel('Time(s)'); ylabel('deg');
    legend('Ref','MPC'); grid on;
end
sgtitle('Robustness Test 3 Mass Uncertainty +20 percent (Joints 1 to 3)','FontSize',12,'FontWeight','bold');

err_pid_u = max(abs(q_traj_deg(:) - qa_pid_u_deg(:)));
err_mpc_u = max(abs(q_traj_deg(:) - qm_u(:)));
fprintf('Mass +20 percent — PID MaxErr: %.4f deg | MPC MaxErr: %.4f deg\n', err_pid_u, err_mpc_u);

%% =====================================================================
%  SUMMARY
%% =====================================================================
fprintf('\n====== PART 2 ROBUSTNESS SUMMARY ======\n');
fprintf('%-25s %-12s %-12s\n','Scenario','PID MaxErr','MPC MaxErr');
fprintf('%-25s %-12.4f %-12.4f\n','Ideal',          2.0305, 0.2580);
fprintf('%-25s %-12.4f %-12.4f\n','Sensor Noise',   err_pid_n, err_mpc_n);
fprintf('%-25s %-12.4f %-12.4f\n','Disturbance',    err_pid_d, err_mpc_d);
fprintf('%-25s %-12.4f %-12.4f\n','Mass +20 pct',   err_pid_u, err_mpc_u);
fprintf('========================================\n');

%% Save
figs = findall(0,'Type','figure');
for i=1:length(figs)
    fname = sprintf('C:\\Users\\yoges\\Desktop\\Part2_Robust_Fig%d.png',i);
    saveas(figs(i),fname);
    fprintf('Saved: %s\n',fname);
end
disp('Part 2 robustness testing complete. All plots saved.');