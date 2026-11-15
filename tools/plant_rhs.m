function dz = plant_rhs(t, V, z)
%PLANT_RHS  Local test helper: turret_plant as a column-vector ODE right-hand side.
[a, b, c] = turret_plant(t, V, z(1), z(2), z(3)); dz = [a; b; c];
end
