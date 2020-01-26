function out = basis_func(basis,x,k,vec)
	if nargin == 3; vec = 0; end
	
	if vec == 0
		% returns the k_th element of the basis function
		if strcmp(basis,'poly') == 1
			out = x^(k-1);
		elseif strcmp(basis,'b-spline') == 1
			if k == 1
				out = 0;
			elseif k == 2
				out = 8/6*x^3 + 4*x^2 + 4*x + 4/3;
			elseif k == 3
				out = -4*x^2 - 4*x + 2/3;
			elseif k == 4
				out = 4*x^2 - 4*x + 2/3;
			elseif k == 5
				out = -8/6*x^3 + 4*x^2 - 4*x + 4/3;
			else
				out = 0;
			end
		else
			% polynomial basis by default
			out = x^(k-1);
		end
	else
		% returns the k_th element of the basis function
		if strcmp(basis,'poly') == 1
			out = x.^(k-1);
		elseif strcmp(basis,'b-spline') == 1
			if k == 1
				out = 0*x;
			elseif k == 2
				out = 8/6*x.^3 + 4*x.^2 + 4*x + 4/3;
			elseif k == 3
				out = -4*x.^2 - 4*x + 2/3;
			elseif k == 4
				out = 4*x.^2 - 4*x + 2/3;
			elseif k == 5
				out = -8/6*x.^3 + 4*x.^2 - 4*x + 4/3;
			else
				out = 0;
			end
		else
			% polynomial basis by default
			out = x.^(k-1);
		end
	end
end