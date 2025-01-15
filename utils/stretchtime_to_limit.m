function [ppstretch, stretch] = stretchtime_to_limit(pp, mass)
	stretch_lb = 0.001;
	stretch_ub = 1 / stretch_lb;
	stretch = nan;
	while true
		stretch = sqrt(stretch_lb * stretch_ub);
		ppstretch = pp_stretchtime(pp, stretch);
		c = piecewise_physical_constraint(ppstretch, mass);
		cmax = max(c(:));
		TOL = 0.002;
		if cmax > TOL
			stretch_lb = stretch;
		elseif cmax < -TOL
			stretch_ub = stretch;
		else
			break;
		end
    end
end
