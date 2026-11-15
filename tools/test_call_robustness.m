% VERIFICATION (tools/). D19/D20: the controller must give the same
% result when called like an ODE solver does (repeated times, small backward
% steps on rejected steps, irregular spacing) as on a clean 50 ms schedule.
p = params(); sc = 'C';
R0 = run_scenario(sc);                                    % clean reference
z = [p.theta0_test; 0; 0]; h = 1e-3; Nt = round(p.t_end/h);
clear turret_controller; pul = []; prevf = 0; tl = -Inf;
for k = 0:Nt                         % includes t = t_end, like run_scenario
    t = k*h;
    % solver-like call pattern: accepted point first, stage times, a repeated
    % time and a backward step (rejected step); outputs used = those at t
    tp_ = [t, t + 0.5*h, t + 0.5*h, t + 0.2*h];   % RK-stage-like, incl. repeat and step back
    for q = 1:numel(tp_)
        [rm, tm, mv] = target_motion_generator(tp_(q), sc);
        [Vq, fq] = turret_controller(tp_(q), rm, tm, mv, z(1), z(2));
        if q == 1, V = Vq; f = fq; end
    end
    if f == 1 && prevf == 0 && t - tl >= p.t_recharge - 1e-9, pul(end+1) = t; tl = t; end %#ok<AGROW>
    prevf = f;
    if k == Nt, break; end           % no plant step after the last instant
    fr = @(zz) plant_rhs(t, V, zz);
    k1 = fr(z); k2 = fr(z + h/2*k1); k3 = fr(z + h/2*k2); k4 = fr(z + h*k3);
    z = z + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
fprintf('clean pulses : %s\n', sprintf('%.3f ', [R0.pulses.t]));
fprintf('solver-style : %s\n', sprintf('%.3f ', pul));
fprintf('same pulse times: %d\n', isequal(round(pul*1e3), round([R0.pulses.t]*1e3)));
