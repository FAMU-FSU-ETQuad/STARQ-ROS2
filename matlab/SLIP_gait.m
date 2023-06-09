%% SLIP Model Trajectory
% Jonathan Boylan

clear
close all
clc

%% Parameters

% Hopping model
hoppingModel = struct;
hoppingModel.L = 0.3; % m
hoppingModel.m = 1; % kg
hoppingModel.k = 5000; % N/m
hoppingModel.b = 5; % N-s/m
hoppingModel.g = 9.81; % m/s^2
hoppingModel.v = 3; % m/s
hoppingModel.q_td = pi/6;
hoppingModel.q_lo = -pi/6;

% Motor struct
motor.R = 0.25; % Shaft length
motor.GR = 6; % Gear ratio
motor.Km = -37160; % RPM:Torque slope
motor.Tmax = 0.0267; % Max continous torque
motor.Vmax = 9550; % No load speed
motor.Tstall = 0.257; % Stall torque
motor.Fc = 0; % Constant force
motor.P = 0; % Power (0 or 1)

% Initial height
y0 = 0.3;

%% Motor Actuation

motor.GR = 4;
motor.P = [0, 1];
hoppingModel.k = hoppingModel.k(1);

results_MA = newt_raph(hoppingModel, motor, y0)

hops = 10;
tyvp = [0, y0, 0, 0];
for hp = 1:hops
    tyvp = simulate(hoppingModel, motor, tyvp);
end

% Get simulation vals
t = tyvp(:,1);
y = tyvp(:,2);
phi = tyvp(:,4);

% Plot
figure
hold on
plot(t,y)
plot([0 t(end)],[results_MA.y_ss results_MA.y_ss], '--k', 'LineWidth', 2)
xlabel("Time");
ylabel("Height");

%% Animate

% Animate
animate(hoppingModel, t, y, phi);


%% Newton Raphson

function yf = stride(model, motor, y)

    tyvp = simulate(model, motor, [0, y, 0, 0]);
    yf = tyvp(end,2);
    
end

function results = newt_raph(model, motor, y0)

    % Error function
    E = @(y) stride(model, motor, y) - y;

    % Central diff deriv
    slope = @(y, del) (E(y+del) - E(y-del)) / (2*del);

    % New height
    y_n = @(y, del) y - E(y)./slope(y,del);

    % params
    y = y0;
    del = 0.01;
    epsilon = 1E-10;

    % iterate
    E_last = E(y);
    results.num_iters = 0;
    while (abs(E_last) > epsilon)
        results.num_iters = results.num_iters + 1;
        
        y = abs(y_n(y, del));

        E_new = E(y);

        if (norm(E_new) > norm(E_last))
            results.fail = true;
            return
        end

        E_last = E_new;
    end

    % steady state y val
    results.y_ss = y;

    % slope at fixed point
    results.fp_slope = slope(y, del);

    % fixed point stable?
    results.is_stable = abs(results.fp_slope) < 1;

end

%% Simulate

function tyvp = simulate(model, motor, inits)

    % Initial conditions
    time = inits(:,1);
    position = inits(:,2);
    velocity = inits(:,3);
    phi = inits(:,4);
    
    % Motor power
    Km = -motor.Vmax/motor.Tstall;
    F_motV = motor.P.*(motor.GR^2/(motor.R^2*Km));
    F_motC = motor.Fc - F_motV .* motor.Vmax;
    

    % Free Fall Sim
    freeFallInit = [position(end), velocity(end)];
    freeFallOptions = odeset('Events',@hitGroundEventFcn);
    [ff_T,ff_Q] = ode45(@(t,q) slipODE(t,q,model.g, model.m, 0, 0, 0, 0),...
        [0 inf], freeFallInit, freeFallOptions);
    ff_P = interp1([0 ff_T(end)],[0 model.q_td], ff_T);

    % Compression Sim
    compressionInit = [ff_Q(end,1), ff_Q(end,2)];
    compressionOptions = odeset('Events', @maxCompressionEventFcn);
    [cmp_T,cmp_Q] = ode45(@(t,q) slipODE(t,q,model.g, model.m, model.b, model.k(1), F_motV(1), F_motC(1)),...
        [0 inf], compressionInit, compressionOptions);
    cmp_P = interp1([0 cmp_T(end)],[model.q_td 0], cmp_T);

    % Extension Sim
    extensionInit = [cmp_Q(end, 1), cmp_Q(end,2)];
    extensionOptions = odeset('Events', @leaveGroundEventFcn);
    [ext_T,ext_Q] = ode45(@(t,q) slipODE(t,q,model.g, model.m, model.b, model.k(end), F_motV(end), F_motC(end)),...
        [0 inf], extensionInit, extensionOptions);
    ext_P = interp1([0 ext_T(end)],[0 model.q_lo], ext_T);

    % Takeoff Sim
    takeoffInit = [ext_Q(end,1), ext_Q(end,2)];
    takeoffOptions = odeset('Events', @peakHeightEventFcn);
    [tko_T,tko_Q] = ode45(@(t,q) slipODE(t,q,model.g, model.m, 0, 0, 0, 0),...
        [0 inf], takeoffInit, takeoffOptions);
    tko_P = interp1([0 tko_T(end)],[model.q_lo 0], tko_T);

    % Add results to graph
    position = [position; ff_Q(2:end,1); cmp_Q(2:end,1); ext_Q(2:end,1); tko_Q(2:end,1)];
    velocity = [velocity; ff_Q(2:end,2); cmp_Q(2:end,2); ext_Q(2:end,2); tko_Q(2:end,2)];
    time = [time;
            time(end) + ff_T(2:end);
            time(end) + ff_T(end) + cmp_T(2:end);
            time(end) + ff_T(end) + cmp_T(end) + ext_T(2:end);
            time(end) + ff_T(end) + cmp_T(end) + ext_T(end) + tko_T(2:end)];
    phi = [phi; ff_P(2:end); cmp_P(2:end); ext_P(2:end); tko_P(2:end)];

    tyvp = [time, position, velocity, phi];
end

function [position, isterminal, direction] = hitGroundEventFcn(~,y)
    position = y(1);
    isterminal = 1;
    direction = -1;
end

function [position, isterminal, direction] = maxCompressionEventFcn(~,y)
    position = y(2);
    isterminal = 1;
    direction = 1;
end

function [position, isterminal, direction] = leaveGroundEventFcn(~,y)
    position = y(1);
    isterminal = 1;
    direction = 1;
end

function [position, isterminal, direction] = peakHeightEventFcn(~,y)
    position = y(2);
    isterminal = 1;
    direction = -1;
end

function dq = slipODE(~, q, g, m, b, k, Fmv, Fmc)
    x = q(1);
    v = q(2);
    a = (Fmc/m - g) + (Fmv/m - b/m)*v + (-k/m)*x;
    dq = [v; a];
end

%% Animation
function animate(model, t, y, phi)

    fig = figure;

    % get leg length
    L = model.L + y.*(y < 0);
    
    % get x position
    x = model.v .* t;
    
    hold on
    leg_plot = plot(0,0,'-b','LineWidth',8);
    hip_mark = plot(0,0,'ob','MarkerSize', 20, 'MarkerFaceColor','g','LineWidth',5);
    
    % ground
    plot([x(1)-5, x(end)+5],[0 0], '-k', 'LineWidth', 5);
    
    % animate
    tic
    for frame = 1:length(t)

        % current time
        ct = t(frame);
        
        % current phi
        cphi = phi(frame);
        
        % current leg length
        cl = L(frame);
        
        % model frame rotation
        n_R_a = [cos(cphi), -sin(cphi);
                 sin(cphi), cos(cphi)];
        
        % leg points
        hip_n = [x(frame); y(frame) + model.L];
        toe_a = [0; -cl];
        toe_n = hip_n + n_R_a * toe_a;
        
        % update fig
        figure(fig)
        set(leg_plot, 'xdata', [toe_n(1) hip_n(1)]);
        set(leg_plot, 'ydata', [toe_n(2) hip_n(2)]);
        set(hip_mark, 'xdata', hip_n(1));
        set(hip_mark, 'ydata', hip_n(2));
             
        % update view
        axis([x(frame)-2.5, x(frame)+2.5, -max(y+model.L)*0.1, max(y+model.L) * 2.0])
        
        % match real time
        pause(ct - toc);
    end
end