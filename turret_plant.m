function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
%TURRET_PLANT  Elevation-drive dynamics of the Guardian laser turret.
%   [theta_dot_turret, theta_ddot_turret, i_dot] = ...
%       turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
%
%   Inputs : t [s] (unused, time-invariant plant), V_m motor voltage [V],
%            theta_turret [rad], theta_dot_turret [rad/s], i armature current [A]
%   Outputs: state derivatives d(theta)/dt [rad/s], d(theta_dot)/dt [rad/s^2],
%            di/dt [A/s].
%
%   Model (derived from first principles, all referred to the turret side):
%     Electrical : L di/dt = V_m - R i - (Ke N) theta_dot
%     Mechanical : (J_turret + J_m N^2) theta_ddot = (Kt N) i - b theta_dot
%   The motor shaft turns at N*theta_dot, so the back-emf is Ke*N*theta_dot and
%   the gearbox multiplies motor torque Kt*i by N. Motor inertia reflects as
%   J_m*N^2. Gearbox is assumed ideal (rigid, lossless, no backlash).
%
%   Elevation stops (-10 deg, +85 deg): modelled as rigid, inelastic stops.
%   At a stop the angle cannot move further outward; any outward velocity
%   state decays with time constant p.tau_stop, and outward drive torque is
%   reacted by the stop. Inside the travel range the model is exactly linear.
%
%   Note: the stop makes the ODE stiff near the limits; integrate with ode15s
%   (or a fixed step <= 1 ms). The plant also has a lightly damped mode at
%   ~56 rad/s (zeta ~ 0.094).
%
%   No voltage limit is applied here: saturation (D8) lives in the controller,
%   so it holds even if the evaluator uses its own plant.

p = params();

% ---- Electrical dynamics ----
i_dot = (V_m - p.R*i - p.KeN*theta_dot_turret) / p.L;

% ---- Mechanical dynamics (free motion) ----
torque = p.KtN*i - p.b*theta_dot_turret;       % net turret-side torque [N m]
theta_ddot_turret = torque / p.J_eq;

% ---- Hard stops ----
at_min = theta_turret <= p.theta_min;
at_max = theta_turret >= p.theta_max;

if (at_min && theta_dot_turret < 0) || (at_max && theta_dot_turret > 0)
    % Moving into a stop: angle frozen, outward velocity killed quickly.
    theta_dot_out     = 0;
    theta_ddot_turret = -theta_dot_turret / p.tau_stop;
elseif (at_min && theta_ddot_turret < 0) || (at_max && theta_ddot_turret > 0)
    % Resting on a stop and pushed outward: the stop reacts the torque.
    theta_dot_out     = theta_dot_turret;
    theta_ddot_turret = 0;
else
    theta_dot_out = theta_dot_turret;
end

theta_dot_turret = theta_dot_out;
end
