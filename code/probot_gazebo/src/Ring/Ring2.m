%% 0. 输入路径点序列
Path = {};
Ts = {};
path=[];
p1 = [0 0 0 0 0 0];
p2 = [0.523405    -1.28345   0.0903509 -0.00186839      1.1927    0.525141]; % 蓝
%p3 = [0 -1.10021 0.597412 0 0.4048 0];%0.26 % 中
p3 = [0     -1.38925       1.3472 -0.000797031    0.0420541  3.35086e-05];%0.26 % 中
p4 = [-0.708536     -1.44723     0.565107 -0.000952057     0.882642    -0.707801]; % 红
%p5 = [0 -1.10021 0.597412 0 0.4048 0];
p5 = p3;
p6 = p2;

Path{1} = [p1;p2;p3;p4];
Ts{1} = [3.8;1.71;1.71];
Path{2} = [p4;p5;p6];
Ts{2} = [1.71;1.71];
Path{3} = [p2;p3;p4];
Ts{3} = [1.0;1.0];
Path{4} = [p4;p5;p6];
Ts{4} = [1.0;1.0];

%% 1. 轨迹生成参数初始化
fid=fopen('output.txt', 'wt');
for k = 1:length(Path)-1
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
    for i=1:iend
        for j=1:jend
            fprintf(fid,"%.5d, ", poly_coef_qall(i,j));
        end
        fprintf(fid,"\n");
    end
end
fclose(fid);



%% 3. 代入求解P,V,A多项式参数，画出轨迹P,V,A曲线
qall_n=[];
dqall_n=[];
ddqall_n=[];
tall=[];
ts_all=[];%key point
t_begin = 0;

tstep = 0.01;
for j = 1:n
    k = 1;
    for i=0:n_seg-1
        % 求每段轨迹的P,V,A多项式参数
        start_idx = n_poly_perseg * i;
        P = poly_coef_qall(start_idx+1 : start_idx+n_poly_perseg,j);
        P = flipud(P);
        V=[];
        A=[];
        for m = 1:n_order+1
            if m<=n_order
                V=[V;(n_order+1-m)*P(m)];
            else
                V=[0;V];
            end
        end
        
        for m = 1:n_order+1
            if m<=n_order-1
                A=[A;(n_order-m)*V(m+1)];
            else
                A=[0;A];
            end
        end
        ts_all=[ts_all;t_begin];
        % 在求解出的每段轨迹中进一步取点
        t = 1.0;
        qall_n(k,j)  = polyval(P,t);
        dqall_n(k,j)  = polyval(V,t);
        ddqall_n(k,j)  = polyval(A,t);
        k = k+1;
        if j==1
            tall = [tall;t+t_begin];
        end
        t_begin = t_begin+ts(i+1);
    end
end
ts_all=[ts_all;t_begin];

% 绘图
figure
subplot(3,1,1)
plot(tall, qall_n ,'LineWidth',1);
grid on
title('P');
xlabel('t/s')
ylabel('p/rad')
legend('t1','t2','t3','t4','t5','t6')

subplot(3,1,2)
plot(tall, dqall_n ,'LineWidth',1);
grid on
title('V');
xlabel('t/s')
ylabel('v/rad/s')
legend('t1','t2','t3','t4','t5','t6')

subplot(3,1,3)
plot(tall, ddqall_n ,'LineWidth',1);
grid on
title('A');
xlabel('t/s')
ylabel('a/rad/s^2')
legend('t1','t2','t3','t4','t5','t6')
hold on
set(gcf,'color','w');


%% QP求解器
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



