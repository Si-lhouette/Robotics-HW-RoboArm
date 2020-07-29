function ring2()
% pos_list = [0.26 0.15  0.08  pi/2 0 0   %blue
%             0.402   0   0.13  pi/2 0 0   %绕过障碍物
%             0.28 -0.24  0.08  pi/2 0 0  %red
%             0.402   0   0.13  pi/2 0 0   %绕过障碍物
%             0.26 0.15  0.08  pi/2 0 0];

pos_list = [0.26 0.15  0.06  pi/2 0 0   %blue
            0.402   0   0.13  pi/2 0 0   %绕过障碍物
            0.28 -0.24  0.07  pi/2 0 0  %red
            0.402   0   0.13  pi/2 0 0   %绕过障碍物
            0.26 0.15  0.08  pi/2 0 0];
        
o_pos = [0 0 0 0 0 0];
blue_pos = traj_produce_ikine(pos_list(1,:),[0 0 0 0 0 0]); %目标
mid_pos = traj_produce_ikine(pos_list(2,:),blue_pos);%避障
red_pos = traj_produce_ikine(pos_list(3,:),mid_pos);%中间

%% 0. 输入路径点序列
Path = {};
Ts = {};
path=[];
p1 = [0 0 0 0 0 0];
p2 = [0.63183  -1.11537 -0.226353  0.645407   2.95664 -0.137018]; % 蓝
p3 = [0    -1.10936   -0.665862 0.000813127     3.34522 0.000164436];%0.26 % 中
p4 = [-0.817554  -1.23918  0.200842 -0.890836   2.78592  0.405799]; % 红
p5 = p3;
p6 = p2;

Path{1} = [o_pos;blue_pos;mid_pos;red_pos];
Ts{1} = [4;1.5;1.5];
Path{2} = [red_pos;mid_pos;blue_pos];
Ts{2} = [1.0;1.0];
Path{3} = [blue_pos;mid_pos;red_pos];
Ts{3} = [1.0;1.0];
Path{4} = [red_pos;mid_pos;blue_pos];
Ts{4} = [1.0;1.0];

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
delta = 0;

%% 3. ROS接口定义
velpub = rospublisher('/probot_anno/arm_vel_controller/command',rostype.std_msgs_Float64MultiArray);
real_velpub = rospublisher('speed_chatter',rostype.std_msgs_Float32MultiArray);

velmsg = rosmessage(velpub);
real_velmsg=rosmessage(real_velpub);

%% 4. Path1&2速度发布
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
%         joint_states_data = receive(joint_states,0.1);
%         now_theta = joint_states_data.Position;

        vel_theta  = zeros(1,6);

        for j = 1:6
            start_idx = 6 * no;
            P = Poly1(start_idx+1 : start_idx+6,j);
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
        vel_theta(1) = vel_theta(1)*30*180/pi;
        vel_theta(2) = vel_theta(2)*205*180/(3*pi);
        vel_theta(3) = vel_theta(3)*50*180/pi;
        vel_theta(4) = vel_theta(4)*125*180/(2*pi);
        vel_theta(5) = vel_theta(5)*125*180/(2*pi);
        vel_theta(6) = vel_theta(6)*200*180/(9*pi);
        real_velmsg.Data = vel_theta;
        vel_theta

        send(velpub,velmsg);
        send(real_velpub,real_velmsg);
        pause(0.005);
    else
        velmsg.Data = [0 0 0 0 0 0];
        real_velmsg.Data = [0 0 0 0 0 0];
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
        pause(0.005);
        break;
    end
end

%% 5. Path1&2速度发布
t_all = [Ts{3};Ts{4}];
t_step = [];
for i = 1:length(t_all)
    t_step = [t_step;sum(t_all(1:i))];
end
ti = 0;
no = 0;
while 1 
    t = toc;
    ti_last = ti;
    ti = mod((t - t_base_1),t_base_2);
    if ti > t_step(no+1)
        no = no + 1;
    end
    if ti - ti_last < 0
        no = 0;
    end
    if t <= 100
%         joint_states_data = receive(joint_states,0.1);
%         now_theta = joint_states_data.Position;

        vel_theta  = zeros(1,6);

        for j = 1:6
            start_idx = 6 * no;
            P = Poly2(start_idx+1 : start_idx+6,j);
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
                vel_theta(j) = polyval(V,ti);
            else
                vel_theta(j) = polyval(V,ti-t_step(no));
            end
        end

        velmsg.Data = vel_theta;
        vel_theta(1) = vel_theta(1)*30*180/pi;
        vel_theta(2) = vel_theta(2)*205*180/(3*pi);
        vel_theta(3) = vel_theta(3)*50*180/pi;
        vel_theta(4) = vel_theta(4)*125*180/(2*pi);
        vel_theta(5) = vel_theta(5)*125*180/(2*pi);
        vel_theta(6) = vel_theta(6)*200*180/(9*pi);
        real_velmsg.Data = vel_theta;

        vel_theta
        send(velpub,velmsg);
        send(real_velpub,real_velmsg);
        pause(0.005);
    else
        velmsg.Data = [0 0 0 0 0 0];
        real_velmsg.Data = [0 0 0 0 0 0];
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(real_velpub,real_velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
        send(velpub,velmsg);
        pause(0.005);
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

