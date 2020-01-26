classdef rectangle_pivot
	properties
		m = 0.01
		I = 1.2
		nv
		v
		regions
		env_regions
		lines
		traj
		g = 9.8
		ext_f
		env
	end

	methods
		function obj = rectangle_pivot()
			obj.nv = 4;
			h = 0.1; l = 0.1;
			obj.v = [-1,-1;1,-1;1,1;-1,1]'/20;
			dt = 0.1;
			obj.I = obj.m*0.01^2/3;

			% trajector

			th = linspace(0,-pi/4,5);
			NT = size(th,2);

			r = zeros(2,NT)

			for t = 1:NT
				rot = [cos(th(t)),-sin(th(t));sin(th(t)),cos(th(t))];
				r(:,t) = [0.05;0] - rot*obj.v(:,2);
			end

			y = r(2,:);
			x = r(1,:);

			obj.traj.r = [x;y;th];
			obj.traj.dr = zeros(3,NT);
			obj.traj.ddr = zeros(3,NT);

			% derivatives
			for t = 2:NT-1
				obj.traj.dr(:,t) = (obj.traj.r(:,t+1)-obj.traj.r(:,t-1))/(2*dt);
			end

			for t = 1:NT
				if t > 1 
					v1 = obj.traj.r(:,t-1)
				else 
					v1 = obj.traj.r(:,t); 
				end
				if t < NT 
					v2 = obj.traj.r(:,t+1); 
				else 
					v2 = obj.traj.r(:,t); 
				end

				obj.traj.ddr(:,t) = (-2*obj.traj.r(:,t)+v1+v2)/(dt)^2;
			end

			% external force constraints
			ext_f = {};
			ext_f{1} = struct(); ext_f{1}.fc1 = [0,0]; ext_f{1}.fc2 = [0,0];
			ext_f{2} = struct(); ext_f{2}.fc1 = [-0.1,1]; ext_f{2}.fc2 = [0.1,1];
			ext_f{3} = struct(); ext_f{3}.fc1 = [0,0]; ext_f{3}.fc2 = [0,0];
			ext_f{4} = struct(); ext_f{4}.fc1 = [0,0]; ext_f{4}.fc2 = [0,0];

			% constraints for the entire plan
			obj.ext_f = {};
			for v = 1:4
				obj.ext_f{v} = struct();
				obj.ext_f{v}.fc1 = zeros(2,NT);
				obj.ext_f{v}.fc2 = zeros(2,NT);
				obj.ext_f{v}.jac = zeros(2,3,NT);

				for t = 1:NT
					th = obj.traj.r(3,t);
					drot = 0.01/sqrt(2)*[cos(pi/4-th);sin(pi/4-th)];
					obj.ext_f{v}.fc1(:,t) = ext_f{v}.fc1;
					obj.ext_f{v}.fc2(:,t) = ext_f{v}.fc2;
					obj.ext_f{v}.jac(:,:,t) = 0*[eye(2), drot];
				end
			end

			% lines for reference
			lines_init = {};
			lines_init{1} = struct(); lines_init{1}.v1 = obj.v(:,1); lines_init{1}.v2 = obj.v(:,2);
			lines_init{1}.fc1 = [1,2]'; lines_init{1}.fc2 = [-1,2]';
			lines_init{1}.t = [-1,0];

			lines_init{2} = struct(); lines_init{2}.v1 = obj.v(:,2); lines_init{2}.v2 = obj.v(:,3);
			lines_init{2}.fc1 = [-2,1]'; lines_init{2}.fc2 = [-2,-1]';
			lines_init{2}.t = [0,-1];

			lines_init{3} = struct(); lines_init{3}.v1 = obj.v(:,3); lines_init{3}.v2 = obj.v(:,4);
			lines_init{3}.fc1 = [1,-2]'; lines_init{3}.fc2 = [-1,-2]';
			lines_init{3}.t = [1,0];

			lines_init{4} = struct(); lines_init{4}.v1 = obj.v(:,4); lines_init{4}.v2 = obj.v(:,1);
			lines_init{4}.fc1 = [2,-1]'; lines_init{4}.fc2 = [2,1]';
			lines_init{4}.t = [0,1];

			% lines for the entire plan
			obj.lines = {};

			% applies the transformation to each line segment
			for l = 1:4
				obj.lines{l} = struct();
				obj.lines{l}.v1 = zeros(2,NT);
				obj.lines{l}.v2 = zeros(2,NT);
				obj.lines{l}.fc1 = zeros(2,NT);
				obj.lines{l}.fc2 = zeros(2,NT);
				for t = 1:NT
					th = obj.traj.r(3,t); trans = obj.traj.r(1:2,t);
					rotmat = [cos(th),-sin(th);sin(th),cos(th)];
					obj.lines{l}.v1(:,t) = trans + rotmat*lines_init{l}.v1;
					obj.lines{l}.v2(:,t) = trans + rotmat*lines_init{l}.v2;

					obj.lines{l}.fc1(:,t) = rotmat*lines_init{l}.fc1;
					obj.lines{l}.fc2(:,t) = rotmat*lines_init{l}.fc2;
				end
			end

			% regions for reference
			regions_init = {};
			regions_init{1} = struct(); regions_init{1}.A = [1,0]; regions_init{1}.b = [-l/2];
			regions_init{2} = struct(); regions_init{2}.A = [-1,0]; regions_init{2}.b = [-l/2]; 
			regions_init{3} = struct(); regions_init{3}.A = [0,-1]; regions_init{3}.b = [-h/2]; 
			regions_init{4} = struct(); regions_init{4}.A = [0,1]; regions_init{4}.b = [-h/2]; 

			% regions during the motion
			obj.regions = {};
			for r = 1:4
				obj.regions{r} = struct();
				obj.regions{r}.A = zeros(size(regions_init{r}.A,1),size(regions_init{r}.A,2),NT);
				obj.regions{r}.b = zeros(size(regions_init{r}.b,1),NT);
				% reorients the planes through time
				for t = 1:NT
					th = obj.traj.r(3,t); trans = obj.traj.r(1:2,t);
					rotmat = [cos(th),-sin(th);sin(th),cos(th)];
					obj.regions{r}.A(:,:,t) = regions_init{r}.A*inv(rotmat);
					obj.regions{r}.b(:,t) = regions_init{r}.b + regions_init{r}.A*inv(rotmat)*trans;
				end
			end	

			% environment regiions
			obj.env_regions = {}
			obj.env_regions{1}.A = [0,-1]; obj.env_regions{1}.b = -0.001;
			obj.env = {}; 
			obj.env{1}.x = [-1,1,1,-1]; obj.env{1}.y = [-1,-1,0,0];	
			obj.env{1}.n = [0;1];	
		end
	end

end