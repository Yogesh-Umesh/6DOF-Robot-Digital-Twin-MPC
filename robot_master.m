%% robot_master.m
% Complete 2-DOF Robot Project — Phases 1 to 4
% MATLAB R2026a
% Yoges — Desktop

clc; close all; bdclose all;

%% =====================================================================
%  PARAMETERS
%% =====================================================================
L1=1.0; L2=0.8; m1=1.5; m2=1.0; w=0.05;
assignin('base','L1',L1); assignin('base','L2',L2);
assignin('base','m1',m1); assignin('base','m2',m2);
assignin('base','w',w);
disp('Parameters loaded.');

%% =====================================================================
%  PHASE 1 — FORWARD KINEMATICS PLOT
%% =====================================================================
configs=[0,0;45,-30;90,-90;135,45;-45,60];
figure('Name','FK','Color','white','Position',[100 100 1200 300]);
colors=lines(size(configs,1));
for i=1:size(configs,1)
    q1=deg2rad(configs(i,1)); q2=deg2rad(configs(i,2));
    x0=0; y0=0;
    x1=L1*cos(q1); y1=L1*sin(q1);
    x2=x1+L2*cos(q1+q2); y2=y1+L2*sin(q1+q2);
    subplot(1,size(configs,1),i);
    plot([x0 x1 x2],[y0 y1 y2],'-o','Color',colors(i,:),'LineWidth',3,'MarkerSize',8,'MarkerFaceColor',colors(i,:));
    hold on;
    plot(x0,y0,'ks','MarkerSize',12,'MarkerFaceColor','k');
    plot(x2,y2,'r*','MarkerSize',12);
    title(sprintf('q1=%d°\nq2=%d°',configs(i,1),configs(i,2)));
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal; grid on; xlim([-2 2]); ylim([-2 2]);
end
sgtitle('2-DOF Robot — Forward Kinematics','FontSize',14,'FontWeight','bold');
disp('Phase 1 — FK plot done.');

%% =====================================================================
%  PHASE 1 — BUILD SIMSCAPE MODEL
%% =====================================================================
modelName='robot_2dof';
if exist([modelName '.slx'],'file'); delete([modelName '.slx']); end
new_system(modelName); open_system(modelName);
set_param(modelName,'Solver','ode15s','StopTime','5');

mechConfigLib=sprintf('sm_lib/Utilities/Mechanism\nConfiguration');
rigidTransLib=sprintf('sm_lib/Frames and\nTransforms/Rigid\nTransform');
worldFrameLib=sprintf('sm_lib/Frames and\nTransforms/World Frame');

add_block('nesl_utility/Solver Configuration',[modelName '/Solver Config'],'Position',[50 600 250 660]);
add_block(mechConfigLib,[modelName '/Mechanism Config'],'Position',[300 600 530 660]);
add_block(worldFrameLib,[modelName '/World Frame'],'Position',[50 50 180 110]);
add_block(rigidTransLib,[modelName '/Base Transform'],'Position',[230 50 380 110]);

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Rev Joint 1'],'Position',[430 50 600 110]);
set_param([modelName '/Rev Joint 1'],'TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Link 1'],'Position',[650 50 800 110]);
set_param([modelName '/Link 1'],'InertiaType','CalculateFromGeometry','BasedOnType','Mass','Mass','m1','CylinderRadius','0.025','CylinderLength','L1');
add_block(rigidTransLib,[modelName '/Link1 Offset'],'Position',[830 50 980 110]);

add_block('sm_lib/Joints/Revolute Joint',[modelName '/Rev Joint 2'],'Position',[1000 50 1170 110]);
set_param([modelName '/Rev Joint 2'],'TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on');
add_block('sm_lib/Body Elements/Cylindrical Solid',[modelName '/Link 2'],'Position',[1200 50 1350 110]);
set_param([modelName '/Link 2'],'InertiaType','CalculateFromGeometry','BasedOnType','Mass','Mass','m2','CylinderRadius','0.025','CylinderLength','L2');
add_block(rigidTransLib,[modelName '/Link2 Offset'],'Position',[1380 50 1530 110]);

add_block('simulink/Sources/From Workspace',[modelName '/Ref Q1'],'VariableName','[t q1_traj]','Position',[50 200 200 240]);
add_block('simulink/Sources/From Workspace',[modelName '/Ref Q2'],'VariableName','[t q2_traj]','Position',[50 300 200 340]);

add_block('nesl_utility/PS-Simulink Converter',[modelName '/PS-SL1'],'Position',[650 200 800 240]);
add_block('nesl_utility/PS-Simulink Converter',[modelName '/PS-SL2'],'Position',[1000 200 1150 240]);

add_block('simulink/Math Operations/Gain',[modelName '/Rad2Deg1'],'Gain','180/pi','Position',[830 200 960 240]);
add_block('simulink/Math Operations/Gain',[modelName '/Rad2Deg2'],'Gain','180/pi','Position',[1180 200 1310 240]);

add_block('simulink/Math Operations/Sum',[modelName '/Error1'],'Inputs','+-','Position',[250 200 290 240]);
add_block('simulink/Math Operations/Sum',[modelName '/Error2'],'Inputs','+-','Position',[250 300 290 340]);

add_block('simulink/Continuous/PID Controller',[modelName '/PID1'],'P','Kp1','I','Ki1','D','Kd1','Position',[330 190 480 250]);
add_block('simulink/Continuous/PID Controller',[modelName '/PID2'],'P','Kp2','I','Ki2','D','Kd2','Position',[330 290 480 350]);

add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL-PS1'],'Position',[520 200 620 240]);
add_block('nesl_utility/Simulink-PS Converter',[modelName '/SL-PS2'],'Position',[520 300 620 340]);

% PID Gains
Kp1=150; Ki1=5; Kd1=20; Kp2=100; Ki2=3; Kd2=15;
assignin('base','Kp1',Kp1); assignin('base','Ki1',Ki1); assignin('base','Kd1',Kd1);
assignin('base','Kp2',Kp2); assignin('base','Ki2',Ki2); assignin('base','Kd2',Kd2);

% Connect physical chain
add_line(modelName,'World Frame/RConn1','Base Transform/LConn1','autorouting','on');
add_line(modelName,'Base Transform/RConn1','Rev Joint 1/LConn1','autorouting','on');
add_line(modelName,'Rev Joint 1/RConn1','Link 1/RConn1','autorouting','on');
add_line(modelName,'Link 1/RConn1','Link1 Offset/LConn1','autorouting','on');
add_line(modelName,'Link1 Offset/RConn1','Rev Joint 2/LConn1','autorouting','on');
add_line(modelName,'Rev Joint 2/RConn1','Link 2/RConn1','autorouting','on');
add_line(modelName,'Link 2/RConn1','Link2 Offset/LConn1','autorouting','on');
add_line(modelName,'World Frame/RConn1','Mechanism Config/RConn1','autorouting','on');
add_line(modelName,'Base Transform/RConn1','Solver Config/RConn1','autorouting','on');

% Connect sensing
add_line(modelName,'Rev Joint 1/RConn2','PS-SL1/LConn1','autorouting','on');
add_line(modelName,'Rev Joint 2/RConn2','PS-SL2/LConn1','autorouting','on');
add_line(modelName,'PS-SL1/1','Rad2Deg1/1','autorouting','on');
add_line(modelName,'PS-SL2/1','Rad2Deg2/1','autorouting','on');

% Connect control loop
add_line(modelName,'Ref Q1/1','Error1/1','autorouting','on');
add_line(modelName,'Rad2Deg1/1','Error1/2','autorouting','on');
add_line(modelName,'Ref Q2/1','Error2/1','autorouting','on');
add_line(modelName,'Rad2Deg2/1','Error2/2','autorouting','on');
add_line(modelName,'Error1/1','PID1/1','autorouting','on');
add_line(modelName,'Error2/1','PID2/1','autorouting','on');
add_line(modelName,'PID1/1','SL-PS1/1','autorouting','on');
add_line(modelName,'PID2/1','SL-PS2/1','autorouting','on');
add_line(modelName,'SL-PS1/RConn1','Rev Joint 1/LConn2','autorouting','on');
add_line(modelName,'SL-PS2/RConn1','Rev Joint 2/LConn2','autorouting','on');

save_system(modelName);
disp('Phase 1 — Simscape model built and saved.');

%% =====================================================================
%  PHASE 2 — TRAJECTORY PLANNING
%% =====================================================================
tf=5; dt=0.01; t=(0:dt:tf)';
q1_0=0; q1_f=90; q2_0=0; q2_f=-60;
a2_1=(3*(q1_f-q1_0)/tf^2); a3_1=(-2*(q1_f-q1_0)/tf^3);
a2_2=(3*(q2_f-q2_0)/tf^2); a3_2=(-2*(q2_f-q2_0)/tf^3);
q1_traj = q1_0 + a2_1*t.^2 + a3_1*t.^3;
q2_traj = q2_0 + a2_2*t.^2 + a3_2*t.^3;
qd1_traj = 2*a2_1*t + 3*a3_1*t.^2;
qd2_traj = 2*a2_2*t + 3*a3_2*t.^2;
assignin('base','t',t);
assignin('base','q1_traj',q1_traj);
assignin('base','q2_traj',q2_traj);

figure('Name','Trajectories','Color','white','Position',[100 100 900 500]);
subplot(2,2,1); plot(t,q1_traj,'b-','LineWidth',2); title('Joint 1 Position'); xlabel('Time (s)'); ylabel('deg'); grid on;
subplot(2,2,2); plot(t,q2_traj,'r-','LineWidth',2); title('Joint 2 Position'); xlabel('Time (s)'); ylabel('deg'); grid on;
subplot(2,2,3); plot(t,qd1_traj,'b-','LineWidth',2); title('Joint 1 Velocity'); xlabel('Time (s)'); ylabel('deg/s'); grid on;
subplot(2,2,4); plot(t,qd2_traj,'r-','LineWidth',2); title('Joint 2 Velocity'); xlabel('Time (s)'); ylabel('deg/s'); grid on;
sgtitle('Cubic Polynomial Trajectories','FontSize',13,'FontWeight','bold');

q1_rad=deg2rad(q1_traj); q2_rad=deg2rad(q2_traj);
x_ee=L1*cos(q1_rad)+L2*cos(q1_rad+q2_rad);
y_ee=L1*sin(q1_rad)+L2*sin(q1_rad+q2_rad);
figure('Name','End Effector','Color','white');
plot(x_ee,y_ee,'b-','LineWidth',2.5); hold on;
plot(x_ee(1),y_ee(1),'go','MarkerSize',12,'MarkerFaceColor','g');
plot(x_ee(end),y_ee(end),'r*','MarkerSize',12);
xlabel('X (m)'); ylabel('Y (m)');
title('End-Effector Path'); legend('Path','Start','End'); grid on; axis equal;
disp('Phase 2 — Trajectory planning done.');

%% =====================================================================
%  PHASE 3 — PID SIMULATION
%% =====================================================================
disp('Phase 3 — Running PID simulation...');
simOut = sim(modelName);
simlog = simOut.simlog;
q1_pid = simlog.Rev_Joint_1.Rz.q.series.values('deg');
t_pid1 = simlog.Rev_Joint_1.Rz.q.series.time;
q2_pid = simlog.Rev_Joint_2.Rz.q.series.values('deg');
t_pid2 = simlog.Rev_Joint_2.Rz.q.series.time;

figure('Name','PID Tracking','Color','white','Position',[100 100 900 500]);
subplot(2,1,1);
plot(t,q1_traj,'r--','LineWidth',2); hold on;
plot(t_pid1,q1_pid,'b-','LineWidth',2);
title('Joint 1 — PID Tracking'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Reference','Actual'); grid on;
subplot(2,1,2);
plot(t,q2_traj,'r--','LineWidth',2); hold on;
plot(t_pid2,q2_pid,'b-','LineWidth',2);
title('Joint 2 — PID Tracking'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Reference','Actual'); grid on;
sgtitle('PID Controller — Trajectory Tracking','FontSize',13,'FontWeight','bold');

q1_ref_i = interp1(t,q1_traj,t_pid1);
q2_ref_i = interp1(t,q2_traj,t_pid2);
err1_pid = q1_ref_i - q1_pid;
err2_pid = q2_ref_i - q2_pid;

figure('Name','PID Error','Color','white','Position',[100 100 900 400]);
subplot(2,1,1); plot(t_pid1,err1_pid,'b-','LineWidth',2); title('Joint 1 — PID Error'); xlabel('Time (s)'); ylabel('Error (deg)'); grid on; yline(0,'r--');
subplot(2,1,2); plot(t_pid2,err2_pid,'r-','LineWidth',2); title('Joint 2 — PID Error'); xlabel('Time (s)'); ylabel('Error (deg)'); grid on; yline(0,'r--');
sgtitle('PID Tracking Error','FontSize',13,'FontWeight','bold');
fprintf('Phase 3 done. Max PID error J1: %.4f deg, J2: %.4f deg\n', max(abs(err1_pid)), max(abs(err2_pid)));

%% =====================================================================
%  PHASE 4 — MPC
%% =====================================================================
disp('Phase 4 — Setting up MPC...');
q1n=deg2rad(45); q2n=deg2rad(-30);
I1=(1/3)*m1*L1^2; I2=(1/3)*m2*L2^2;
M11=I1+I2+m2*L1^2+2*m2*L1*L2*cos(q2n)*(1/2);
M12=I2+m2*L1*L2*cos(q2n)*(1/2); M22=I2;
M=[M11 M12;M12 M22]; Minv=inv(M);
A=[0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0];
A(3:4,3:4)=-Minv*[0.5 0;0 0.3];
B=[0 0;0 0;Minv(1,:);Minv(2,:)];
C=[1 0 0 0;0 1 0 0]; D=zeros(2,2);
sys=ss(A,B,C,D); Ts=0.01; sys_d=c2d(sys,Ts);
mpcObj=mpc(sys_d,Ts);
mpcObj.PredictionHorizon=40; mpcObj.ControlHorizon=10;
mpcObj.Weights.OutputVariables=[10 10];
mpcObj.Weights.ManipulatedVariablesRate=[0.01 0.01];
mpcObj.Weights.ManipulatedVariables=[0 0];
mpcObj.MV(1).Min=-100; mpcObj.MV(1).Max=100;
mpcObj.MV(2).Min=-100; mpcObj.MV(2).Max=100;

q1_f_rad=deg2rad(90); q2_f_rad=deg2rad(-60);
a2_1r=3*q1_f_rad/tf^2; a3_1r=-2*q1_f_rad/tf^3;
a2_2r=3*q2_f_rad/tf^2; a3_2r=-2*q2_f_rad/tf^3;
q1_ref_rad=a2_1r*t.^2+a3_1r*t.^3;
q2_ref_rad=a2_2r*t.^2+a3_2r*t.^3;

N=length(t); x=zeros(4,1);
q1_mpc=zeros(N,1); q2_mpc=zeros(N,1);
mpcState=mpcstate(mpcObj);

for k=1:N
    y=C*x;
    q1_mpc(k)=rad2deg(y(1));
    q2_mpc(k)=rad2deg(y(2));
    u=mpcmove(mpcObj,mpcState,y,[q1_ref_rad(k);q2_ref_rad(k)]);
    x=sys_d.A*x+sys_d.B*u;
end

q1_ref_deg=rad2deg(q1_ref_rad);
q2_ref_deg=rad2deg(q2_ref_rad);
err1_mpc=q1_ref_deg-q1_mpc;
err2_mpc=q2_ref_deg-q2_mpc;

figure('Name','MPC Tracking','Color','white','Position',[100 100 900 500]);
subplot(2,1,1); plot(t,q1_ref_deg,'r--','LineWidth',2); hold on; plot(t,q1_mpc,'b-','LineWidth',2); title('Joint 1 — MPC Tracking'); xlabel('Time (s)'); ylabel('Angle (deg)'); legend('Reference','MPC'); grid on;
subplot(2,1,2); plot(t,q2_ref_deg,'r--','LineWidth',2); hold on; plot(t,q2_mpc,'b-','LineWidth',2); title('Joint 2 — MPC Tracking'); xlabel('Time (s)'); ylabel('Angle (deg)'); legend('Reference','MPC'); grid on;
sgtitle('MPC Controller — Trajectory Tracking','FontSize',13,'FontWeight','bold');

figure('Name','MPC Error','Color','white','Position',[100 100 900 400]);
subplot(2,1,1); plot(t,err1_mpc,'b-','LineWidth',2); title('Joint 1 — MPC Error'); xlabel('Time (s)'); ylabel('Error (deg)'); grid on; yline(0,'r--');
subplot(2,1,2); plot(t,err2_mpc,'r-','LineWidth',2); title('Joint 2 — MPC Error'); xlabel('Time (s)'); ylabel('Error (deg)'); grid on; yline(0,'r--');
sgtitle('MPC Tracking Error','FontSize',13,'FontWeight','bold');

fprintf('\n========= PID vs MPC COMPARISON =========\n');
fprintf('%-20s %-15s %-15s\n','Metric','PID','MPC');
fprintf('%-20s %-15.4f %-15.4f\n','Max Error J1 (deg)',max(abs(err1_pid)),max(abs(err1_mpc)));
fprintf('%-20s %-15.4f %-15.4f\n','Max Error J2 (deg)',max(abs(err2_pid)),max(abs(err2_mpc)));
fprintf('%-20s %-15.2f %-15.2f\n','Final J1 (deg)',q1_pid(end),q1_mpc(end));
fprintf('%-20s %-15.2f %-15.2f\n','Final J2 (deg)',q2_pid(end),q2_mpc(end));
fprintf('==========================================\n');
disp('Phase 4 complete.');

%% =====================================================================
%  SAVE ALL FIGURES
%% =====================================================================
figs=findall(0,'Type','figure');
for i=1:length(figs)
    fname=sprintf('C:\\Users\\yoges\\Desktop\\Master_Figure_%d.png',i);
    saveas(figs(i),fname);
    fprintf('Saved: %s\n',fname);
end
disp('All figures saved to Desktop.');
disp('===== PHASES 1-4 COMPLETE =====');
