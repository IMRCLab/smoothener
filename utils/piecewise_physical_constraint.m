function [c, ceq] = piecewise_physical_constraint(pp, mass)
% given a 4-dimensional piecewise polynomial representing quadcopter path,
% enforces constraints on linear/angular acceleration/velocity
% in a format appropriate for Matlab's nonlinear solvers.

    FEASIBLE_OMEGA = pi/2; % rad/sec?
    % this feasible_d_omega is conservative - i don't think real quadcopters
    % need 2 seconds to reach their peak angular velocity -
    % but we are attempting to stop trajectories that contain flips
    FEASIBLE_D_OMEGA = pi/2;
    FEASIBLE_THRUST_TO_WEIGHT = 1.2;
    
    BOX_MAX = [100 100 100]';
    BOX_MIN = -BOX_MAX;
    
    FEASIBLE_ANGLE = pi / 6; %rad?
    
    npts = 1000;
    [xyz, vel, acc, rotmtx_imu2world, quat_imu2world, rpy_imu2world, omega, t] = trajectory_eval_piecewise(pp, mass, npts);


	g = 9.81;
	thrust = acc;
	thrust(3,:) = thrust(3,:) + g;
    thrust = mass * thrust;
	thrust_mag = sqrt(sum(thrust.^2, 1));
	max_thrust_to_weight = max(thrust_mag) / (mass * g);

	% treat omegas independently for now
	max_omega = max(abs(omega), [], 2);
    
    d_omega = diff(omega, 2);
    max_d_omega = max(abs(d_omega), [], 2);
    
    angle = acos(squeeze(rotmtx_imu2world(3,3,:)));
    max_angle = max(abs(angle));

	% c is a vector of constraint terms, all of which must be <= 0
	c = [max_thrust_to_weight - FEASIBLE_THRUST_TO_WEIGHT; ...
         max_omega - FEASIBLE_OMEGA; ...
         %max_d_omega - FEASIBLE_D_OMEGA; ...
         max_angle - FEASIBLE_ANGLE; ...
         %max(xyz, [], 2) - BOX_MAX; ...
         %-(min(xyz, [], 2) - BOX_MIN); ...
    ];
    max_acc = max(abs(acc), [], 'all');
    max_vel = max(abs(vel), [], 'all');

    % dbcbs comparison
 	c = [max_acc - 2.0; ...
         max_vel - 0.5; ...
    ];   


	ceq = [];
end
