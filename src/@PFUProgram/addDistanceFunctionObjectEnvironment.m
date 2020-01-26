function [c,dc]= addDistanceFunctionObjectEnvironment(obj,x)
	% hand and object vertices
	e = [obj.Cx_e, obj.Cy_e];
	v = [x(obj.Cx_b), x(obj.Cy_b)];

	% rotation matrices
	rot = [cos(x(obj.LPtheta_b)),-sin(x(obj.LPtheta_b));sin(x(obj.LPtheta_b)),cos(x(obj.LPtheta_b))];
	drot = [-sin(x(obj.LPtheta_b)),-cos(x(obj.LPtheta_b));cos(x(obj.LPtheta_b)),-sin(x(obj.LPtheta_b))];

	% computes the distance
	[dFunOE,p1,p2,collision] = GJK(e,v);
	
	% SDF
	if collision == 1
		dFunOE = -dFunOE;
	end

	% constraint
	c = [x(obj.dFun_be)-dFunOE];
	
	% gradients
	dc = sparse(1,obj.Nvars);
	if dFunOE ~= 0
		dc(obj.Cx_b) = (p2(1) - p1(1))/dFunOE;
		dc(obj.Cy_b) = (p2(2) - p1(2))/dFunOE;
		dc(obj.LPtheta_b) = drot*inv(rot)*(p2 - [x(obj.Cx_b);x(obj.Cy_b)]);
	else
		dc(obj.Cx_b) = 1;
		dc(obj.Cy_b) = 1;
		dc(obj.LPtheta_b) = drot*inv(rot)*(p2 - [x(obj.Cx_b);x(obj.Cy_b)]);
	end
end
