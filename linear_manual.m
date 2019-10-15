%% Linearization of bitcraze plant

% Linearization point 
x0 = zeros([5,1]);
u0 = zeros(4,1);

% variables and constants
p = sym('p',[3,1],'real'); 
v = sym('v',[3,1],'real');
w = sym('w',[3,1],'real');
angle = sym('angle',[2,1],'real'); 
Jx = 1.395e-5;
Jy = 1.436e-5;
Jz = 2.173e-5;
J = [Jx,0,0;0,Jy,0;0,0,Jz];  % inertia matrix
ctrl = sym('ctrl',[4,1],'real');  % input (torque) 
m = 0.027; %mass
g = 9.81;  %Gravity
d = 0.046; %Arm length 
k = 2.75e-11;  %Drag constant 
b = 1e-9;  %Lift constant

states = [angle; w];

% dynamics: dx/dt=f(x)
f = [w;
     inv(J)*(-cross(w,J*w) + [d/sqrt(2)*(-ctrl(1)-ctrl(2)+ctrl(3)+ctrl(4)) ; d/sqrt(2)*(-ctrl(1)+ctrl(2)+ctrl(3)-ctrl(4)) ; (k/b)*(-ctrl(1)+ctrl(2)-ctrl(3)+ctrl(4))])]; 
f = f([1,2,4,5,6]); %No yaw angle control 

% linearization (Jacobians + substitution of linearization state)
A = jacobian(f,states); 
AA = subs(A,states,{x0});
A_lin = subs(AA,ctrl,{u0});

B = jacobian(f,ctrl);
BB = subs(B,states,{x0});
B_lin = subs(BB,ctrl,{u0});

% measurements: y = h(x)
h = [angle;
     w];

 % linearization
C = jacobian(h,states); 
CC = subs(C,states,{x0});
C_lin = subs(CC,ctrl,{u0});

D = jacobian(h,ctrl);
DD = subs(D,states,{x0});
D_lin = subs(DD,ctrl,{u0});

% sym2double
A_lin = double(A_lin);
B_lin = double(B_lin);
C_lin = double(C_lin);
D_lin = double(D_lin);
 
%% Controlability 
% S = size(ctrb(sysd.A,sysd.B));
% R = rank(ctrb(sysd.A, sysd.B));
% O = rank(obsv(sysd.A,sysd.C));
%% LQR controller no reference
Ts = 0.01;
nofstates = 5; 
Q = diag([3.3*10^3 3.3*10^3 1*10^5 1*10^5 1*10^5]);                      
R = 10^(-2)*eye(4); 

sys = ss(A_lin, B_lin,C_lin,D_lin);
sysd = c2d(sys,Ts);


K = dlqr(sysd.A, sysd.B, Q, R);
Kang = K;
Kang(:,5) = zeros(4,1); 

% Set to zeros for Simulink model
Kr_pitch = zeros(4,1);
Kr_roll = zeros(4,1);
Kr_yawrate = zeros(4,1);
FF_roll = zeros(4,1);
FF_pitch = zeros(4,1);

%% Check Eigenvalues
E = eig(sysd.A - sysd.B*K);
 %% LQR WITH REF
K_Ref = pinv(sysd.C*((sysd.A-sysd.B*K-eye(nofstates))\sysd.B))*[1,0,0,0,0;0,1,0,0,0]';
FF_roll = K_Ref(:,1);
FF_pitch = 10*K_Ref(:,2);

% Set to zeros for Simulink model
Kr_pitch = zeros(4,1);
Kr_roll = zeros(4,1);
Kr_yawrate = zeros(4,1);
%% LQI 
% Augment state space for the relevant state/states  
Ts = 0.01; 
Ae = [A_lin zeros(5,1);
     -C_lin(2,:) 0];
Be = [B_lin; zeros(1,4)];
Ce = [C_lin zeros(5,1)];
De = [D_lin];

syse = ss(Ae, Be, Ce, De); 
sysde = c2d(syse,Ts);
Q = diag([3.3*10^3 3.3*10^3 10^5 10^5 10^5 3.3*10^(3)]);    
R = 1*10^(-4)*eye(4);

K = dlqr(sysde.A, sysde.B, Q, R);
Kang = K(:, [1 2 3 4]);
Kr_pitch = K(:,end); %Integral action for pitch 
Kr_roll = zeros(4,1);
Kr_yawrate = zeros(4,1);
% Set to zeros for Simulink model
FF_roll = zeros(:,1);
FF_pitch = zeros(:,2);