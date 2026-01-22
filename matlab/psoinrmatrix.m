%% 主程序：五连杆轮腿机器人LQR参数PSO优化与控制器生成
clc; clear; close all;

% =========================================================================
% 第一部分：参数定义
% =========================================================================
fprintf('========================================\n');
fprintf('  五连杆轮腿机器人LQR控制器优化程序\n');
fprintf('========================================\n');

% 腿长范围
L0s = 0.12:0.01:0.32;          % L0变化范围
Ks = zeros(2, 6, length(L0s)); % 存放不同L0对应的K

% 选择优化工作点（中间值）
step_to_optimize = round(length(L0s)/2);
L0_optim = L0s(step_to_optimize);
fprintf('优化工作点: L0 = %.3f m\n', L0_optim);
fprintf('腿长范围: %.2f ~ %.2f m\n', min(L0s), max(L0s));
fprintf('优化目标: 5m/s速度阶跃响应\n');

% =========================================================================
% 第二部分：获取系统模型
% =========================================================================
fprintf('\n计算系统模型...\n');
[G_optim, H_optim] = get_system_matrices(L0_optim);
fprintf('系统矩阵维度: G(%dx%d), H(%dx%d)\n', size(G_optim,1), size(G_optim,2), size(H_optim,1), size(H_optim,2));

% =========================================================================
% 第三部分：PSO优化QR参数
% =========================================================================
fprintf('\n开始PSO优化QR参数...\n');
fprintf('----------------------------------------\n');

% 定义PSO优化参数
n_vars = 8;  % 优化8个参数：6个Q对角元素 + 2个R对角元素

% 参数边界（根据工程经验设定）
lb = [500,   0.1,  100,   2,   5000, 0.1,  0.1,   0.05];  % 下界
ub = [5000,  20,   500,   50,  20000, 5,    2.0,   0.5];  % 上界

% 初始参数（基于之前的经验）
initial_params = [1000, 0.5, 150, 10, 10000, 0.5, 1, 0.25];

% 设置PSO选项
options = optimoptions('particleswarm', ...
    'SwarmSize', 30, ...          % 粒子数量
    'MaxIterations', 50, ...      % 最大迭代次数
    'Display', 'iter', ...        % 显示迭代信息
    'UseParallel', true, ...      % 使用并行计算
    'FunctionTolerance', 1e-4, ...% 函数容差
    'MaxStallIterations', 10, ... % 最大停滞迭代
    'InertiaRange', [0.4, 0.9], ...% 惯性权重范围
    'SelfAdjustmentWeight', 1.49, ... % 个体学习因子
    'SocialAdjustmentWeight', 1.49, ... % 社会学习因子
    'InitialSwarmMatrix', initial_params, ... % 初始猜测
    'PlotFcn', @pswplotbestf);    % 绘制收敛曲线

% 运行PSO优化
[x_opt, fval_opt, exitflag, output] = particleswarm(...
    @(params) lqr_cost_function_5mps(params, G_optim, H_optim), ...
    n_vars, lb, ub, options);

% 显示优化结果
fprintf('\n========================================\n');
fprintf('           优化结果\n');
fprintf('========================================\n');
fprintf('迭代次数: %d\n', output.iterations);
fprintf('函数评估次数: %d\n', output.funccount);
fprintf('最优目标函数值: %.4f\n', fval_opt);

% 提取最优参数
Q_opt = diag([x_opt(1:6)]);
R_opt = diag([x_opt(7:8)]);

fprintf('\n最优Q矩阵参数:\n');
fprintf('diag([');
fprintf('%.1f, ', x_opt(1:5));
fprintf('%.1f])\n', x_opt(6));

fprintf('\n最优R矩阵参数:\n');
fprintf('diag([%.4f, %.4f])\n', x_opt(7), x_opt(8));

% 保存优化结果
save('optimized_lqr_params.mat', 'x_opt', 'Q_opt', 'R_opt', 'fval_opt', 'L0s', 'step_to_optimize');

% =========================================================================
% 第四部分：验证优化参数在5m/s阶跃下的性能
% =========================================================================
fprintf('\n验证5m/s阶跃响应...\n');
verify_5mps_response(x_opt, L0_optim);

% =========================================================================
% 第五部分：使用优化参数计算所有腿长的K矩阵
% =========================================================================
fprintf('\n计算所有腿长的LQR控制器...\n');
fprintf('----------------------------------------\n');

% 使用优化后的QR参数计算所有L0对应的K矩阵
for step = 1:length(L0s)
    % 所需符号量
    syms theta dtheta ddtheta;
    syms x x1 x2;
    syms phi phi1 phi2;
    syms T Tp Nf t;
    
    % 机器人结构参数
    R = 0.0775;
    L = L0s(step) / 2;
    Lm = L0s(step) / 2;
    mw = 0.334 * 2;
    l = -0.004;
    mp = 1.482 * 2;
    M = (17.5 + 0.68 - mp*2 - mw*2);
    Iw = 0.5 * mw * R ^ 2;
    Ip = mp * ((L + Lm) ^ 2 + 0.144 ^ 2) / 12.0;
    Im = 0.23203539;
    g = 9.8;

    % 进行物理计算
    Nm = M * (x2 + (L + Lm) * (ddtheta  * cos(theta) - dtheta  ^ 2 * sin(theta)) - l * (phi2 * cos(phi) - phi1 ^ 2 * sin(phi)));
    Pm = M * g + M * ((L + Lm) * (-dtheta  ^ 2 * cos(theta) - ddtheta  * sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
    N = Nm + mp * (x2 + L * (ddtheta  * cos(theta) - dtheta  ^ 2 * sin(theta)));
    P = Pm + mp * g + mp * L * (-dtheta  ^ 2 * cos(theta) - ddtheta  * sin(theta));
    
    equ1 = x2 - (T - N * R) / (Iw / R + mw * R);
    equ2 = (P * L + Pm * Lm) * sin(theta) - (N * L + Nm * Lm) * cos(theta) - T + Tp - Ip * ddtheta;
    equ3 = Tp + Nm * l * cos(phi) + Pm * l * sin(phi) - Im * phi2;

    [x2, ddtheta, phi2] = solve(equ1, equ2, equ3, x2, ddtheta, phi2);
    
    % 求得雅克比矩阵，然后得到状态空间方程
    Ja = jacobian( [dtheta; ddtheta; x1; x2; phi1; phi2], [theta dtheta  x x1 phi phi1]);
    Jb = jacobian( [dtheta; ddtheta; x1; x2; phi1; phi2], [T Tp]);
    A = vpa(subs(Ja, [theta dtheta  x x1  phi phi1], [0 0 0 0 0 0]));
    B = vpa(subs(Jb, [theta dtheta  x x1  phi phi1], [0 0 0 0 0 0]));
    
    % 离散化
    [G, H] = c2d(eval(A), eval(B), 0.005);
    
    % 使用优化后的QR参数
    Q = diag([x_opt(1:6)]);
    R = diag([x_opt(7:8)]);

    % 求解反馈矩阵K
    Ks(:,:,step) = dlqr(G, H, Q, R);
    
    if mod(step, 5) == 0
        fprintf('  计算完成: L0 = %.3f m\n', L0s(step));
    end
end

fprintf('所有腿长的K矩阵计算完成！\n');

% =========================================================================
% 第六部分：对K矩阵进行多项式拟合
% =========================================================================
fprintf('\n对K矩阵进行多项式拟合...\n');

% 创建符号变量
K = sym('K', [2 6]);
syms L0;

% 对每个K矩阵元素进行3阶多项式拟合
fprintf('进行3阶多项式拟合...\n');
for x = 1:2
    for y = 1:6
        % 提取该元素在所有L0下的值
        element_data = squeeze(Ks(x, y, :))';
        
        % 3阶多项式拟合
        p = polyfit(L0s, element_data, 3);
        K(x, y) = p(1) * L0^3 + p(2) * L0^2 + p(3) * L0 + p(4);
        
        % 计算拟合误差
        fitted_values = polyval(p, L0s);
        rms_error = sqrt(mean((element_data - fitted_values).^2));
        max_error = max(abs(element_data - fitted_values));
        
        fprintf('  K(%d,%d): RMS误差 = %.6f, 最大误差 = %.6f\n', ...
            x, y, rms_error, max_error);
    end
end

% 输出到m函数
fprintf('\n生成MATLAB函数: lqr_k_optimized.m\n');
matlabFunction(K, 'File', 'lqr_k_optimized', 'Vars', L0);

% =========================================================================
% 第七部分：验证拟合结果
% =========================================================================
fprintf('\n验证拟合结果...\n');
fprintf('----------------------------------------\n');

% 测试几个典型腿长
test_L0s = [0.15, 0.20, 0.25, 0.30];
fprintf('测试腿长: ');
fprintf('%.2f ', test_L0s);
fprintf('m\n\n');

figure('Position', [100, 100, 1400, 800]);
for idx = 1:length(test_L0s)
    L0_test = test_L0s(idx);
    
    % 计算真实K
    [G_test, H_test] = get_system_matrices(L0_test);
    K_real = dlqr(G_test, H_test, Q_opt, R_opt);
    
    % 计算拟合K
    K_fit = double(subs(K, L0, L0_test));
    
    % 计算误差
    error = K_real - K_fit;
    max_error = max(abs(error(:)));
    rms_error = sqrt(mean(error(:).^2));
    
    fprintf('L0 = %.2f m:\n', L0_test);
    fprintf('  最大误差: %.6f\n', max_error);
    fprintf('  RMS误差: %.6f\n', rms_error);
    
    % 绘制对比
    subplot(2, 2, idx);
    bar(1:12, [K_real(:), K_fit(:)]);
    xlabel('K矩阵元素索引');
    ylabel('增益值');
    title(sprintf('L0 = %.2f m 的K矩阵对比', L0_test));
    legend('真实值', '拟合值', 'Location', 'best');
    grid on;
end
sgtitle('K矩阵拟合结果验证');

% =========================================================================
% 第八部分：验证控制器在不同腿长下的性能
% =========================================================================
fprintf('\n验证控制器在不同腿长下的性能...\n');
verify_controller_performance(L0s, Ks, x_opt);

% =========================================================================
% 第九部分：生成最终报告
% =========================================================================
fprintf('\n========================================\n');
fprintf('           优化完成！\n');
fprintf('========================================\n');
fprintf('优化参数已保存到: optimized_lqr_params.mat\n');
fprintf('LQR控制器函数: lqr_k_optimized.m\n');
fprintf('使用方法: K = lqr_k_optimized(L0)\n\n');

% 显示最终优化结果
fprintf('最终优化结果:\n');
fprintf('Q = diag([');
fprintf('%.1f, ', x_opt(1:5));
fprintf('%.1f])\n', x_opt(6));
fprintf('R = diag([%.4f, %.4f])\n\n', x_opt(7), x_opt(8));

% 测试最终函数
L0_final = 0.20;
K_final = lqr_k_optimized(L0_final);
fprintf('示例(L0=%.2f m):\n', L0_final);
disp('K =');
disp(K_final);

% =========================================================================
% 辅助函数定义
% =========================================================================

function [G, H] = get_system_matrices(L0)
    % 获取系统矩阵的函数
    
    % 所需符号量
    syms theta dtheta ddtheta;
    syms x x1 x2;
    syms phi phi1 phi2;
    syms T Tp Nf t;
    
    % 机器人结构参数
    R = 0.0775;
    L = L0 / 2;
    Lm = L0 / 2;
    mw = 0.334 * 2;
    l = -0.004;
    mp = 1.482 * 2;
    M = (17.5 + 0.68 - mp*2 - mw*2);
    Iw = 0.5 * mw * R ^ 2;
    Ip = mp * ((L + Lm) ^ 2 + 0.144 ^ 2) / 12.0;
    Im = 0.23203539;
    g = 9.8;

    % 进行物理计算
    Nm = M * (x2 + (L + Lm) * (ddtheta * cos(theta) - dtheta ^ 2 * sin(theta)) - ...
              l * (phi2 * cos(phi) - phi1 ^ 2 * sin(phi)));
    Pm = M * g + M * ((L + Lm) * (-dtheta ^ 2 * cos(theta) - ddtheta * sin(theta)) - ...
                      l * (phi1 ^ 2 * cos(phi) + phi2 * sin(phi)));
    N = Nm + mp * (x2 + L * (ddtheta * cos(theta) - dtheta ^ 2 * sin(theta)));
    P = Pm + mp * g + mp * L * (-dtheta ^ 2 * cos(theta) - ddtheta * sin(theta));
    
    equ1 = x2 - (T - N * R) / (Iw / R + mw * R);
    equ2 = (P * L + Pm * Lm) * sin(theta) - (N * L + Nm * Lm) * cos(theta) - ...
           T + Tp - Ip * ddtheta;
    equ3 = Tp + Nm * l * cos(phi) + Pm * l * sin(phi) - Im * phi2;

    [x2, ddtheta, phi2] = solve(equ1, equ2, equ3, x2, ddtheta, phi2);
    
    % 求得雅克比矩阵
    Ja = jacobian([dtheta; ddtheta; x1; x2; phi1; phi2], [theta, dtheta, x, x1, phi, phi1]);
    Jb = jacobian([dtheta; ddtheta; x1; x2; phi1; phi2], [T, Tp]);
    
    % 线性化在平衡点
    A = double(subs(Ja, [theta, dtheta, x, x1, phi, phi1], [0, 0, 0, 0, 0, 0]));
    B = double(subs(Jb, [theta, dtheta, x, x1, phi, phi1], [0, 0, 0, 0, 0, 0]));
    
    % 离散化
    [G, H] = c2d(A, B, 0.005);
end

function cost = lqr_cost_function_5mps(params, G, H)
    % 针对5m/s速度阶跃的LQR控制器性能评价函数
    % 主要目标：最小化轮毂最大输出力矩，同时保证系统稳定
    
    % 提取QR参数
    Q = diag(max(params(1:6), 1e-6));  % 确保正定
    R = diag(max(params(7:8), 1e-6));
    
    try
        % 计算LQR增益
        [K, ~, eig_cl] = dlqr(G, H, Q, R);
        
        % 稳定性检查
        if ~all(abs(eig_cl) < 1)
            cost = 1e6 + sum(abs(eig_cl(abs(eig_cl) >= 1))) * 1000;
            return;
        end
        
        % 仿真参数
        Ts = 0.005;
        sim_time = 4.0;  % 仿真时间
        n_steps = round(sim_time / Ts);
        
        % 初始状态
        x = zeros(6, 1);
        
        % 5m/s速度阶跃
        v_ref = 5.0;
        
        % 存储数据
        wheel_torque = zeros(1, n_steps);
        joint_torque = zeros(1, n_steps);
        state_history = zeros(6, n_steps);
        velocity_error = zeros(1, n_steps);
        
        stability_flag = true;
        
        for k = 1:n_steps
            t = (k-1) * Ts;
            
            % 参考状态
            x_ref = [0; 0; 0; v_ref; 0; 0];
            
            % 控制输入
            u = -K * (x - x_ref);
            
            % 记录力矩
            wheel_torque(k) = abs(u(1));
            joint_torque(k) = abs(u(2));
            
            % 记录状态
            state_history(:, k) = x;
            
            % 计算速度误差
            velocity_error(k) = abs(v_ref - x(4));
            
            % 状态更新
            x = G * x + H * u;
            
            % 稳定性检查
            if any(isnan(x)) || any(isinf(x)) || abs(x(1)) > 0.8 || abs(x(5)) > 0.8
                stability_flag = false;
                break;
            end
        end
        
        if ~stability_flag
            cost = 1e6;
            return;
        end
        
        % 性能指标计算
        max_wheel_torque = max(wheel_torque);
        avg_joint_torque = mean(joint_torque);
        
        % 控制能量
        control_energy = sum(wheel_torque.^2) * Ts;
        
        % 速度跟踪性能
        settling_time = calculate_settling_time(velocity_error, Ts, 0.05);  % 5%误差带
        itae_velocity = sum((1:n_steps)' .* Ts .* velocity_error') * Ts;
        
        % 角度稳定性
        max_theta = max(abs(state_history(1, :)));
        max_phi = max(abs(state_history(5, :)));
        
        % 代价函数
        cost = 0;
        
        % 1. 主要目标：最小化轮毂最大力矩
        cost = cost + 0.5 * max_wheel_torque;
        
        % 2. 速度跟踪性能
        if settling_time > 2.0
            cost = cost + 0.3 * (settling_time - 2.0) * 10;
        end
        
        % 3. 角度稳定性惩罚
        if max_theta > 0.3
            cost = cost + 0.1 * (max_theta - 0.3) * 20;
        end
        if max_phi > 0.3
            cost = cost + 0.1 * (max_phi - 0.3) * 20;
        end
        
        % 4. 控制能量消耗
        cost = cost + 0.05 * sqrt(control_energy);
        
        % 5. 关节出力（适当鼓励）
        if avg_joint_torque < 0.5
            cost = cost + 0.05 * (0.5 - avg_joint_torque) * 5;
        end
        
    catch ME
        cost = 1e6;
    end
end

function settling_time = calculate_settling_time(error, Ts, threshold_percent)
    % 计算调节时间
    threshold = 5.0 * threshold_percent;  % 5m/s的百分比
    
    % 找到误差首次进入阈值内的时间
    settled_indices = find(error <= threshold);
    
    if ~isempty(settled_indices)
        % 找到连续进入阈值的时间
        for i = 1:length(settled_indices)-100
            if all(error(settled_indices(i):settled_indices(i)+99) <= threshold)
                settling_time = (settled_indices(i) - 1) * Ts;
                return;
            end
        end
    end
    
    settling_time = length(error) * Ts;
end

function verify_5mps_response(params, L0)
    % 验证5m/s阶跃响应
    
    fprintf('验证5m/s阶跃响应 (L0=%.3f m)\n', L0);
    
    % 获取系统模型
    [G, H] = get_system_matrices(L0);
    
    % 提取QR参数
    Q = diag(max(params(1:6), 1e-6));
    R = diag(max(params(7:8), 1e-6));
    
    % 计算LQR控制器
    K = dlqr(G, H, Q, R);
    
    % 仿真参数
    Ts = 0.005;
    sim_time = 4.0;
    time = 0:Ts:sim_time;
    
    % 5m/s阶跃响应
    v_ref = 5.0;
    
    % 初始状态
    x = zeros(6, 1);
    
    % 存储数据
    x_history = zeros(6, length(time));
    u_history = zeros(2, length(time));
    
    for k = 1:length(time)
        % 参考状态
        x_ref = [0; 0; 0; v_ref; 0; 0];
        
        % 控制输入
        u = -K * (x - x_ref);
        
        % 记录
        x_history(:, k) = x;
        u_history(:, k) = u;
        
        % 状态更新
        x = G * x + H * u;
    end
    
    % 计算性能指标
    max_wheel_torque = max(abs(u_history(1, :)));
    avg_joint_torque = mean(abs(u_history(2, :)));
    
    fprintf('  轮毂最大力矩: %.3f Nm\n', max_wheel_torque);
    fprintf('  关节平均力矩: %.3f Nm\n', avg_joint_torque);
    
    % 绘制响应
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(3, 2, 1);
    plot(time, x_history(4, :), 'b-', 'LineWidth', 2);
    hold on;
    plot([0, sim_time], [v_ref, v_ref], 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    title('5m/s阶跃速度响应');
    legend('实际速度', '目标速度', 'Location', 'best');
    
    subplot(3, 2, 2);
    plot(time, x_history(1, :), 'b-', 'LineWidth', 1.5, 'DisplayName', '\theta');
    hold on;
    plot(time, x_history(5, :), 'r-', 'LineWidth', 1.5, 'DisplayName', '\phi');
    grid on;
    xlabel('时间 (s)');
    ylabel('角度 (rad)');
    title('角度响应');
    legend('Location', 'best');
    
    subplot(3, 2, 3);
    plot(time, u_history(1, :), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('轮毂力矩 (Nm)');
    title('轮毂控制输入');
    
    subplot(3, 2, 4);
    plot(time, u_history(2, :), 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('关节力矩 (Nm)');
    title('关节控制输入');
    
    subplot(3, 2, 5);
    plot(time, x_history(2, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'd\theta');
    hold on;
    plot(time, x_history(6, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'd\phi');
    grid on;
    xlabel('时间 (s)');
    ylabel('角速度 (rad/s)');
    title('角速度响应');
    legend('Location', 'best');
    
    subplot(3, 2, 6);
    plot(time, x_history(3, :), 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('时间 (s)');
    ylabel('位置 (m)');
    title('位置响应');
    
    sgtitle(sprintf('5m/s阶跃响应验证 (L0=%.3f m)', L0));
end

function verify_controller_performance(L0s, Ks, params)
    % 验证控制器在不同腿长下的性能
    
    fprintf('验证控制器在不同腿长下的性能...\n');
    
    n_points = length(L0s);
    performance_metrics = struct();
    
    for i = 1:n_points
        % 获取系统模型
        [G, H] = get_system_matrices(L0s(i));
        
        % 获取K矩阵
        K = Ks(:,:,i);
        
        % 仿真参数
        Ts = 0.005;
        sim_time = 3.0;
        time = 0:Ts:sim_time;
        
        % 5m/s阶跃响应
        v_ref = 5.0;
        
        % 初始状态
        x = zeros(6, 1);
        
        % 存储最大力矩
        max_wheel_torque = 0;
        
        for k = 1:length(time)
            % 参考状态
            x_ref = [0; 0; 0; v_ref; 0; 0];
            
            % 控制输入
            u = -K * (x - x_ref);
            
            % 更新最大力矩
            max_wheel_torque = max(max_wheel_torque, abs(u(1)));
            
            % 状态更新
            x = G * x + H * u;
        end
        
        % 存储性能指标
        performance_metrics.max_wheel_torque(i) = max_wheel_torque;
        
        if mod(i, 5) == 0
            fprintf('  L0=%.3f: 最大轮毂力矩=%.3f Nm\n', ...
                L0s(i), max_wheel_torque);
        end
    end
    
    % 绘制性能曲线
    figure('Position', [100, 100, 1000, 400]);
    
    subplot(1, 2, 1);
    plot(L0s, performance_metrics.max_wheel_torque, 'b-', 'LineWidth', 2);
    hold on;
    plot([min(L0s), max(L0s)], [30, 30], 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('腿长 L0 (m)');
    ylabel('最大轮毂力矩 (Nm)');
    title('5m/s阶跃下的最大轮毂力矩');
    legend('实际力矩', '30Nm阈值', 'Location', 'best');
    
    subplot(1, 2, 2);
    % 稳定性分析
    stability_flags = true(1, n_points);
    for i = 1:n_points
        [G, H] = get_system_matrices(L0s(i));
        K = Ks(:,:,i);
        eig_cl = eig(G - H*K);
        stability_flags(i) = all(abs(eig_cl) < 1);
    end
    
    bar(L0s, double(stability_flags), 'FaceColor', [0.2, 0.6, 0.2]);
    grid on;
    xlabel('腿长 L0 (m)');
    ylabel('稳定性 (1=稳定)');
    title('系统稳定性');
    ylim([0, 1.5]);
    
    fprintf('\n稳定性统计: %d/%d 个工作点稳定\n', sum(stability_flags), n_points);
    fprintf('力矩<30Nm的工作点: %d/%d\n', ...
        sum(performance_metrics.max_wheel_torque < 30), n_points);
end