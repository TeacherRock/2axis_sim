%% Load Command
openfile = 'Trajectory4_10times.txt';
path  = ['D:\成大\碩一\新訓\我的\6_二軸手臂鑑別\3_robot simulation\Trajectory\', openfile];
Trajectory = load(path);
trajectory.P = Trajectory(:, 1 : 2);
trajectory.V = Trajectory(:, 3 : 4);
trajectory.A = Trajectory(:, 5 : 6);
sampTs = 0.001;
tf = 12;

%% 順項運動學
l1 = 0.24; l2 = 0.24;
% 末端位置
x = l1*cos(trajectory.P(:, 1)) + l2*cos(trajectory.P(:, 1) + trajectory.P(:, 2));
y = l1*sin(trajectory.P(:, 1)) + l2*sin(trajectory.P(:, 1) + trajectory.P(:, 2));

% 關節1位置
x1 = l1*cos(trajectory.P(:, 1));
y1 = l1*sin(trajectory.P(:, 1));

j = 0.1;
pic_num = 1;
for i = 1 : 100 : length(trajectory.P(:, 1))/10
    figure(1)
    static_rectangular([0, -0.3], [0.135, -0.135], [-0.02, -0.3], [ 0.6627 , 0.6627 , 0.6627 ]);
    
    static_rectangular([0, -0.05], [0.05, -0.05], [0, -0.02], [ 0.6627 , 0.6627 , 0.6627 ]);
    
    % link1
    p1 = [0, 0, 0] - ([x1(i), y1(i), 0] - [0, 0, 0])*0.1;
    p2 = [x1(i), y1(i), 0] + ([x1(i), y1(i), 0] - [0, 0, 0])*0.1;
    dynamic_rectangular(p1, p2, "up", 0.1, 0.025, 'r');
    
    % link2
    p3 = [x1(i), y1(i), -0.02] - ([x(i), y(i), -0.02] - [x1(i), y1(i), -0.02])*0.1;
    dynamic_rectangular(p3, [x(i), y(i), -0.02], "down", 0.075, 0.015, 'r');
    
    % bet. link1,2
    p4 = [x1(i), y1(i), 0] + ([x(i), y(i), 0] - [x1(i), y1(i), 0])/12;
    p5 = [x1(i), y1(i), 0] - ([x(i), y(i), 0] - [x1(i), y1(i), 0])/12;
    dynamic_rectangular(p4, p5, "down", 0.03, 0.02, [ 0.6627 , 0.6627 , 0.6627 ]);
    plot3(x(1: 100 : i), y(1: 100 : i), zeros(length(x(1: 100 : i))), 'r');
    axis equal;
    view([45, -45, 30])
    hold off
    title("t = " + j + " s")
    xlim([-0.5, 0.5]); ylim([-0.5, 0.5]); zlim([-0.5, 0.5])
    xlabel("x (m)"); ylabel("y (m)"); zlabel("z (m)")
    j = j + 0.1;
    pause(0.1);

    drawnow;
    F = getframe(gcf);
    I = frame2im(F);
    [I,map]=rgb2ind(I,256);
    
    if pic_num == 1
        imwrite(I,map,'fig.gif','gif','Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,'fig.gif','gif','WriteMode','append','DelayTime',0.2);
    end
    
    pic_num = pic_num + 1;
end

function static_rectangular(x, y, z, color)

fill3([x(1), x(1), x(1), x(1)], [y(1), y(2), y(2), y(1)], [z(1), z(1), z(2), z(2)], color)
hold on
fill3([x(2), x(2), x(2), x(2)], [y(1), y(2), y(2), y(1)], [z(1), z(1), z(2), z(2)], color)
hold on
fill3([x(1), x(1), x(2), x(2)], [y(1), y(2), y(2), y(1)], [z(1), z(1), z(1), z(1)], color)
hold on
fill3([x(1), x(1), x(2), x(2)], [y(1), y(2), y(2), y(1)], [z(2), z(2), z(2), z(2)], color)
hold on
fill3([x(1), x(2), x(2), x(1)], [y(1), y(1), y(1), y(1)], [z(1), z(1), z(2), z(2)], color)
hold on
fill3([x(1), x(2), x(2), x(1)], [y(2), y(2), y(2), y(2)], [z(1), z(1), z(2), z(2)], color)

end

function dynamic_rectangular(p1, p2, up_down, width, height, color)
if up_down == "up"
    z = height;
else
    z = -height;
end

% 方向
d = (p2 -p1)/sqrt((p2(1) - p1(1))^2 + (p2(2) - p1(2))^2);
d2 = width/2*[d(2), -d(1), 0];

% 8個頂點
P1 = p1 + d2;
P2 = p1 - d2;
P3 = P2 + [0, 0, z];
P4 = P1 + [0, 0, z];
P5 = p2 + d2;
P6 = p2 - d2;
P7 = P6 + [0, 0, z];
P8 = P5 + [0, 0, z];

patch([P1(1), P2(1), P3(1), P4(1)], [P1(2), P2(2), P3(2), P4(2)], [P1(3), P2(3), P3(3), P4(3)], color)

patch([P2(1), P6(1), P7(1), P3(1)], [P2(2), P6(2), P7(2), P3(2)], [P2(3), P6(3), P7(3), P3(3)], color)

patch([P5(1), P6(1), P7(1), P8(1)], [P5(2), P6(2), P7(2), P8(2)], [P5(3), P6(3), P7(3), P8(3)], color)

patch([P1(1), P5(1), P8(1), P4(1)], [P1(2), P5(2), P8(2), P4(2)], [P1(3), P5(3), P8(3), P4(3)], color)

patch([P1(1), P2(1), P6(1), P5(1)], [P1(2), P2(2), P6(2), P5(2)], [P1(3), P2(3), P6(3), P5(3)], color)

patch([P8(1), P7(1), P3(1), P4(1)], [P8(2), P7(2), P3(2), P4(2)], [P8(3), P7(3), P3(3), P4(3)], color)
xlim([-0.5, 0.5]); ylim([-0.5, 0.5]); zlim([-0.5, 0.5])

end

