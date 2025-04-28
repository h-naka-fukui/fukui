% Example 1: Reproduce "On the Estimation and Control of the Domain of Attraction"

% Define state variables
syms x1 x2 real
x = [x1; x2];

% Define the system dynamics
f = [x2;
    -3*x1 -2*x2 + x1^2];

% Define the rational Lyapunov function numerator and denominator
vnum = (2*x1^2 + x1*x2 + x2^2)*(3*x1^2 + 2*x1*x2 + x2^2 + 1);
vden = 1 + x1^2 + x2^2;

v = vnum/vden;

% Compute the time derivative of v along the system trajectories
vdot = jacobian(v,x)*f;

% Plotting settings
figure;
hold on;

% Generate a grid for plotting
[x1_grid, x2_grid] = meshgrid(linspace(-10,10,200), linspace(-6,6,200));
v_vals = zeros(size(x1_grid));
vdot_vals = zeros(size(x1_grid));

for i = 1:numel(x1_grid)
    x1_val = x1_grid(i);
    x2_val = x2_grid(i);
    v_vals(i) = double(subs(v, [x1,x2], [x1_val,x2_val]));
    vdot_vals(i) = double(subs(vdot, [x1,x2], [x1_val,x2_val]));
end

% Plot the boundary v(x) = gamma (gamma = 24.179)
gamma = 24.179;
contour(x1_grid, x2_grid, v_vals, [gamma gamma], 'LineWidth', 2);

% Plot the curve vdot(x) = 0
contour(x1_grid, x2_grid, vdot_vals, [0 0], 'r--', 'LineWidth', 2);

% Plot the point found in M1
plot(1.259,2.289,'ko','MarkerSize',8,'MarkerFaceColor','k');

xlabel('x_1'); ylabel('x_2');
title('Example 1: LEDA and vdot = 0 curve');
grid on;
axis equal;
xlim([-10,10]); ylim([-6,6]);
legend('Boundary of LEDA (v=24.179)', 'Curve vdot=0', 'Point in M1');
hold off;

% Save the figure
% aaaaaaa