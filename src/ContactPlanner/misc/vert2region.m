function regions = vert2region(vers)
	% returns the set of convex regions that cover a polygon with vertices verts x,y (ordered clockwise) 
	% the number of regions is guaranteed to be minimal for all convex polygons. However, non-convex polygons, 
	% might return redundant regions or a non-minimal set of regions.

	regions = {};
	nv = size(vers,2)

	cvx = zeros(1,nv);

	% checks the convexity of each vertex
	for i = 1:nv
		% generates the points
		idx0 = i-1; idx1 = i; idx2 = i+1;

		% assigns a value to each
		if i == 1; idx0 = nv; end;
		if i == nv; idx2 = 1; end;

		% vectors of the vertx
		v1 = vers(:,idx1)-vers(:,idx0); v2 = vers(:,idx2)-vers(:,idx1);
		if cross(v1,v2) > 0; cvx(i) = 1; end;
	end

	% regions of the convex hull
	[k,av] = convhull(verts');
	cvx_verts = verts(:,k);
	[A,b] = vert2con(cvx_verts')

	% TODO: generate regions from convex hull
	for i = 1:size(cvx_verts,2)
		regions{i} = struct('A',-A(i,:),'b',-b(i));
	end

	% non-convex regions
	for i = 1:nv
		% generates the points
		idx0 = i-1; idx1 = i; idx2 = i+1;

		% assigns a value to each
		if i == 1; idx0 = nv; end;
		if i == nv; idx2 = 1; end;

		if cvx(idx1) == 0 && cvx(idx0) == 1
			v2 = [verts(:,idx0),verts(:,idx1)];
			i = v+1;
			while cvx(i) == 0
				idx1 = i;
				% assigns a value to each
				if i > nv 
					idx1 = idx1-nv;  
				end

				% appends the vertex if its non-convex
				v2 = [v2,verts(:,idx1)];
				i = idx1 + 1;
			end
			v2 = [v2, verts(:,idx1)];
			[A,b] = vert2con(v2')

			regions{end+1} = struct('A',A,'b',b);
		end
	end

end