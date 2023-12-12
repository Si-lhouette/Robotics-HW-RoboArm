function ring2()

%% 0. 输入路径点序列
Path = {};
Ts = {};
path=[];
p1 = [0 0 0 0 0 0];
p2 = [0.63183  -1.11537 -0.226353  0.645407   2.95664 -0.137018]; % 蓝
%p3 = [0 -1.10021 0.597412 0 0.4048 0];%0.26 % 中
p3 = [0    -1.10936   -0.665862 0.000813127     3.34522 0.000164436];%0.26 % 中
p4 = [-0.817554  -1.23918  0.200842 -0.890836   2.78592  0.405799]; % 红
%p5 = [0 -1.10021 0.597412 0 0.4048 0];
p5 = p3;
p6 = p2;

Path{1} = [p1;p2;p3;p4];
Ts{1} = [3.8;1.85;1.85];
Path{2} = [p4;p5;p6];
Ts{2} = [1.85;1.85];
Path{3} = [p2;p3;p4];
Ts{3} = [1.85;1.85];
Path{4} = [p4;p5;p6];
Ts{4} = [1.85;1.85];

%% 1. 轨迹生成参数初始化
Poly = {};
for k = 1:length(Path)
    path = Path{k};
    ts = Ts{k};
    n = 6; %6个关节
    d_order       = 4;
    n_order       = 5;% 轨迹多项式阶数
    n_seg         = size(path,1)-1;% 轨迹段数
    n_poly_perseg = n_order+1; % 每段轨迹的参数个数

    %% 2. Minijerk - QP问题求解
    poly_coef_qall=[];
    for i=1:6
        poly_coef_q = MinimumSnapQPSolver(path(:, i), ts, n_seg, n_order);
        poly_coef_qall=[poly_coef_qall,poly_coef_q];
    end

    [iend,jend] = size(poly_coef_qall);
    Poly{k} = poly_coef_qall;
end

Poly1 = [Poly{1};Poly{2}];
Poly2 = [Poly{3};Poly{4}];

t_base_1 = sum(Ts{1}) + sum(Ts{2});
t_base_2 = sum(Ts{3}) + sum(Ts{4});
delta = 0.05;

%ROS?
velpub = rospublisher('/probot_anno/arm_vel_controller/command',rostype.std_msgs_Float64MultiArray);
real_velpub = rospublisher('speed_chatter',rostype.std_msgs_Float32MultiArray);

velmsg = rosmessage(velpub);
real_velmsg=rosmessage(real_velpub);
joint_states = rossubscriber('/probot_anno/joint_states');

t_all = [Ts{1};Ts{2}];
t_step = [];
for i = 1:length(t_all)
    t_step = [t_step;sum(t_all(1:i))];
end

tic;
no = 0;
while 1 
    t = toc;
    if t > t_step(no+1)
        no = no + 1;
        if t >=t_step(end)
            break;
        end
    end
    if t <= t_base_1 
        joint_states_data = receive(joint_states,0.1);
        now_theta = joint_states_data.Position;

        vel_theta  = zeros(1,6);

        for j = 1:6
            start_idx = 6 * no;
            P = poly_coef_qall(start_idx+1 : start_idx+6,j);
            P = flipud(P);
            V=[];
            for m = 1:6
                if m<=5
                    V=[V;(6-m)*P(m)];
                else
                    V=[0;V];
                end
            end
            if no == 0
                vel_theta(j) = polyval(V,t);
            else
                vel_theta(j) = polyval(V,t-t_step(no));
            end
        end

        velmsg.Data = vel_theta;
        %閫熷害淇
        vel_theta(1) = vel_theta(1)*30*180/pi;
        vel_theta(2) = vel_theta(2)*205*180/(3*pi);
        vel_theta(3) = vel_theta(3)*50*180/pi;
        vel_theta(4) = vel_theta(4)*125*180/(2*pi);
        vel_theta(5) = vel_theta(5)*125*180/(2*pi);
        vel_theta(6) = vel_theta(6)*200*180/(9*pi);
        real_velmsg.Data = vel_theta;
        %鍙戝竷鎺у埗
        send(velpub,velmsg);
        send(real_velpub,real_velmsg);
    else
        velmsg.Data = [0 0 0 0 0 0];
        real_velmsg.Data = [0 0 0 0 0 0];
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
        break;
    end
end


t_all = [Ts{3};Ts{4}];
t_step = [];
for i = 1:length(t_all)
    t_step = [t_step;sum(t_all(1:i))];
end
no = 0;
while 1 
    t = toc;
    ti = mod((t - t_base_1),t_base_2);
    f = 0;
    if ti > t_step(no+1)
        no = no + 1;
        if no == 1
            f = 1;
        end
        if no >= 3
            no = 0;
        end
    end
    if t <= 100
        joint_states_data = receive(joint_states,0.1);
        now_theta = joint_states_data.Position;

        vel_theta  = zeros(1,6);

        for j = 1:6
            start_idx = 6 * no;
            P = poly_coef_qall(start_idx+1 : start_idx+6,j);
            P = flipud(P);
            V=[];
            for m = 1:6
                if m<=5
                    V=[V;(6-m)*P(m)];
                else
                    V=[0;V];
                end
            end
            if no == 0
                vel_theta(j) = polyval(V,t);
            else
                vel_theta(j) = polyval(V,t-t_step(no));
            end
        end

        velmsg.Data = vel_theta;
        if f == 1
            display(vel_theta)
        end
        %閫熷害淇
        vel_theta(1) = vel_theta(1)*30*180/pi;
        vel_theta(2) = vel_theta(2)*205*180/(3*pi);
        vel_theta(3) = vel_theta(3)*50*180/pi;
        vel_theta(4) = vel_theta(4)*125*180/(2*pi);
        vel_theta(5) = vel_theta(5)*125*180/(2*pi);
        vel_theta(6) = vel_theta(6)*200*180/(9*pi);
        real_velmsg.Data = vel_theta;
        %鍙戝竷鎺у埗
        send(velpub,velmsg);
        send(real_velpub,real_velmsg);
    else
        velmsg.Data = [0 0 0 0 0 0];
        real_velmsg.Data = [0 0 0 0 0 0];
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
    end
end
        
        

end
function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end

