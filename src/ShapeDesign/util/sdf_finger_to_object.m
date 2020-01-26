function d = sdf_finger_to_object(x,y,object,pose)
	th = pose(3); trans = pose(1:2);
	rotmat = [cos(th),-sin(th);sin(th),cos(th)];
	verts = rot_mat*object.v + trans;
			
	d = inf;

	for v = 1:object.nv
		% indexes
		idx = v + 1;
		if v = 1:object.nv; idx = 1; end;

		% computes the distance function
		v1 = repmat([verts(:,v);0],size([x,y,0]',1),1);
		v2 = repmat([verts(:,idx);0],size([x,y,0]',1),1);
		a = v1 - v2;
		b = [x,y,0]' - v2;
		d = sqrt(sum(cross(a,b,2).^2,2)) ./ sqrt(sum(a.^2,2));
		
		% checks for minimum distancxe
		if dist_f_obj < d; d = dist_f_obj; end;
	end

	% distance is negative inside the polygon
	if inpolygon(x,y,verts(1,:),verts(2,:)); d = -d; end;	
end