function [c,dc]= addDistanceFunctionObjectHandVertex(obj,x,vertex)
	% hand and object vertices
	h = [x(obj.Cx_h(vertex)), x(obj.Cy_h(vertex))];
	v = [x(obj.Cx_b), x(obj.Cy_b)];

	% rotation matrices
	rot = [cos(x(obj.LPtheta_b)),-sin(x(obj.LPtheta_b));sin(x(obj.LPtheta_b)),cos(x(obj.LPtheta_b))];
	drot = [-sin(x(obj.LPtheta_b)),-cos(x(obj.LPtheta_b));cos(x(obj.LPtheta_b)),-sin(x(obj.LPtheta_b))];

	% computes the distance
	[dFunOH,p1,p2,collision] = GJK(h,v);
	
	% SDF
	if collision == 1
		dFunOH = -dFunOH;
	end

	% constraint
	c = [x(obj.dFun_bh(vertex))-dFunOH];
	dc = sparse(1,obj.Nvars);

	% gradients
	if collision == 0
		dc(obj.Cx_h(vertex)) = (p1(1) - p2(1))/dFunOH;
		dc(obj.Cy_h(vertex)) = (p1(2) - p2(2))/dFunOH;
		dc(obj.Cx_b) = (p2(1) - p1(1))/dFunOH;
		dc(obj.Cy_b) = (p2(2) - p1(2))/dFunOH;
		dc(obj.LPtheta_b) = drot*inv(rot)*(p2 - [x(obj.Cx_b);x(obj.Cy_b)]);
	else
		dc(obj.Cx_h(vertex)) = 1;
		dc(obj.Cy_h(vertex)) = 1;
		dc(obj.Cx_b) = 1;
		dc(obj.Cy_b) = 1;
		dc(obj.LPtheta_b) = drot*inv(rot)*(p2 - [x(obj.Cx_b);x(obj.Cy_b)]);
	end
end
