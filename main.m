clc,clear,close all;

%% Parameter
[ axis, ns, qb, tid, tc, tf, Tc, nid, nc, np, nh, nx, wf ] = Parameter();

%% 軌跡: 控制點
%     t    px1     vx1    ax1    px2    vx2   ax2
CP = [ 0,   -0.05,  0,     0,     0.1,   0,    0  ; ...
       5,   0,      0.25,  0.25,  0.12,  0,    0  ; ...
       10,  0.05,   0,     0,     0.15,  0,    0  ];
   
 %% 軌跡: 傅立葉級數
% 空間限制   
% l = 1 : nh;
% 
% % A*x < b
% k = 1 : 6*axis;
% for t = tid : tid : tf
%     % F_par = [a1_1, a1_2, ... a1_5, b1_1, b1_2, ... b1_5, q1,
%     %          a2_1, a2_2, ... a2_5, b2_1, b2_2, ... b2_5, q2 ]
%     A(k, :) = [ % axis1
%                 sin(wf*l*t)./(wf*l), -cos(wf*l*t)./(wf*l),  1,                    zeros(1, 11);    % < , q
%                -sin(wf*l*t)./(wf*l),  cos(wf*l*t)./(wf*l), -1,                    zeros(1, 11);    % > , q
%                 cos(wf*l*t),          sin(wf*l*t),          0,                    zeros(1, 11);    % < , dq
%                -cos(wf*l*t),         -sin(wf*l*t),          0,                    zeros(1, 11);    % > , dq
%                -sin(wf*l*t).*(wf*l),  cos(wf*l*t).*(wf*l),  0,                    zeros(1, 11);    % < , ddq
%                 sin(wf*l*t).*(wf*l), -cos(wf*l*t).*(wf*l),  0,                    zeros(1, 11);   % > , ddq
%                 % axis2
%                 zeros(1, 11),         sin(wf*l*t)./(wf*l), -cos(wf*l*t)./(wf*l),  1;    % < , q
%                 zeros(1, 11),        -sin(wf*l*t)./(wf*l),  cos(wf*l*t)./(wf*l), -1;    % > , q
%                 zeros(1, 11),         cos(wf*l*t),          sin(wf*l*t),          0;    % < , dq
%                 zeros(1, 11),        -cos(wf*l*t),         -sin(wf*l*t),          0;    % > , dq
%                 zeros(1, 11),        -sin(wf*l*t).*(wf*l),  cos(wf*l*t).*(wf*l),  0;   % < , ddq
%                 zeros(1, 11),         sin(wf*l*t).*(wf*l), -cos(wf*l*t).*(wf*l),  0    % > , ddq
%               ];
%     b(k, :) = [qb(1, 1); -qb(1, 2); qb(1, 3); qb(1, 3); qb(1, 4); qb(1, 4); ...
%                qb(2, 1); -qb(2, 2); qb(2, 3); qb(2, 3); qb(2, 4); qb(2, 4)]; 
%     k = k + 6;  
% end
% 
% % Aeq*x = beq ,在tf時位置速度加速度均為0
% Aeq  = [ % axis1
%          sin(wf*l*tf)./(wf*l), -cos(wf*l*tf)./(wf*l),  0,                    zeros(1, 11);    % = , q
%          cos(wf*l*tf),          sin(wf*l*tf),          0,                    zeros(1, 11);    % = , dq
%         -sin(wf*l*tf).*(wf*l),  cos(wf*l*tf).*(wf*l),  0,                    zeros(1, 11);    % = , ddq
%          % axis2
%          zeros(1, 11),         sin(wf*l*tf)./(wf*l), -cos(wf*l*tf)./(wf*l),  0;    % = , q
%          zeros(1, 11),         cos(wf*l*tf),          sin(wf*l*tf),          0;    % = , dq
%          zeros(1, 11),        -sin(wf*l*tf).*(wf*l),  cos(wf*l*tf).*(wf*l),  0;    % = , ddq
%          ];  
% beq = zeros(6, 1);
% 
% % 最佳化
% xb = 1 ;  % x0範圍
% x0 = 2 * rand( size( A , 2 ) , 1 ) * xb - xb ;  % x 初值
% 
% options = optimoptions( 'fmincon' , 'Algorithm' , 'sqp' , 'Display' ,'iter','PlotFcn',{@optimplotfval},'MaxIterations' ,2000 );
% [ x , Index ] = fmincon( @(x)optfun(x) , x0 , A , b , Aeq , beq , [] , [] , [] , options ) ;  % 軌跡優化計算
% X = reshape( x, nx, axis ) ;  % 排列 x
 
 %% main
theta = [1.4013, 10.0378, 6.9007, 62.6119, 11.2056, 0.8495, 2.6020, 2.1733, 1.9532];
robot1 = Robot();
robot2 = Robot();
robot3 = Robot();

% robot = setTtype(X, 'Fourier');
robot1.setTtype(CP, 'CtrlP');
robot2.setTtype(CP, 'CtrlP');
robot3.setTtype(CP, 'CtrlP');

% robot = setTheta(robot, theta);
% robot = setController(robot, 'CTC_id');
robot1.setController('Linear');
robot2.setController('CTC');
robot3.setController('DFF');
t = tc : tc : 10;
% Error = robot.par_test();
Error1 = robot1.PosError();
Error2 = robot2.PosError();
Error3 = robot3.PosError();

plot(t, Error1, t, Error2, t, Error3)
legend('Linear', 'CTC', 'DFF')
xlabel("time (s)"); ylabel('position error (m)')



