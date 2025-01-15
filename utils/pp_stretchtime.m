function ppstretch = pp_stretchtime(pp, scale)
	[breaks, coefs, k, order, dim] = unmkpp(pp);
	assert(breaks(1) == 0);
	breaks = scale * breaks;

	coefs = reshape(coefs, [dim k order]);
	for i=1:k
		for d=1:dim
			coefs(d,i,:) = polystretchtime(coefs(d,i,:), scale);
		end
	end

	ppstretch = mkpp(breaks, coefs, dim);
end
