t = 1:60;
delt = 1;
q_tild = 0.1;
x_0 = 0;
y_0 = x_0;
x_dot0 = 0;
y_dot0 = 10;

%%%%%%%%%%% Task 1 %%%%%%%%%%%%%%%%%%%%%
xt = x_dot0*t;
yt = y_dot0*t;
figure;
plot(xt,yt);
title('Task 1: Deterministic Trajectory');
legend({'Deterministic Trajectory'});
xlabel('X-axis');
ylabel('Y-axis');

%%%%%%%%%%% Task 2 %%%%%%%%%%%%%%%%%%%%%
states_T1 = [x_0;x_dot0;y_0;y_dot0];
states_T2 = [x_0;x_dot0;y_0;y_dot0];
states_T3 = [x_0;x_dot0;y_0;y_dot0];
states_T4 = [x_0;x_dot0;y_0;y_dot0];

Traj_1 = zeros(4,60);
Traj_2 = zeros(4,60);
Traj_3 = zeros(4,60);
Traj_4 = zeros(4,60);
Q_mat =[ delt^3/3, delt^2/2, 0, 0; delt^2/2, delt,0,0; 0, 0,delt^3/3, delt^2/2; 0,0,delt^2/2, delt];
Q = Q_mat * q_tild;
F = [ 1, delt,0,0;0,1,0,0;0,0,1, delt;0,0,0,1];
for i= 1:60
    states_T1 = F*states_T1 + (mvnrnd([0;0;0;0],Q,1)).';
    Traj_1(:,i) = states_T1;
    states_T2 = F*states_T2 + (mvnrnd([0;0;0;0],Q,1)).';
    Traj_2(:,i) = states_T2;
    states_T3 = F*states_T3 + (mvnrnd([0;0;0;0],Q,1)).';
    Traj_3(:,i) = states_T3;
    states_T4 = F*states_T4 + (mvnrnd([0;0;0;0],Q,1)).';
    Traj_4(:,i) = states_T4;
end
figure;
plot(xt,yt);
hold on;
plot(Traj_1(1,:),Traj_1(3,:));
hold on;
plot(Traj_2(1,:),Traj_2(3,:));
hold on;
plot(Traj_3(1,:),Traj_3(3,:));
hold on;
plot(Traj_4(1,:),Traj_4(3,:));
hold on;
axis equal;
title('Task 2: Random vs Deterministic Trajectory');
legend({'Det^n Trajectory','Traj_1','Traj_2','Traj_3','Traj_4'},'Location','southwest')
xlabel('X-axis')
ylabel('Y-axis')

%%%%%%%%%% Task 3 %%%%%%%%%%%%%%%%%
sigmaY = 5;
sigmaX = sigmaY;
Measure = zeros(2,60);
R = [sigmaX^2,0;0,sigmaY^2];
H = [1,0;0,1];
for i=1:60
    z = H*[Traj_1(1,i);Traj_1(3,i)] + (mvnrnd([0;0],R,1)).';
    Measure(:,i) = z ;
end
figure;
scatter(Measure(1,:),Measure(2,:),'.')
hold on;
scatter(Traj_1(1,:),Traj_1(3,:),'.');
hold on;
axis equal
title('Task 3: Measurement vs Trajectory ');
legend({'Measurement','Trajectory_1'})
xlabel('X-axis')
ylabel('Y-axis')

