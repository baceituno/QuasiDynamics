classdef triangle_picking
	properties
		m = 0.01
		I = 1.2
		nv
		g = 9.8
		v
		regions
		lines
		traj
		env
		env_regions
		ext_f
	end

	methods
		function obj = triangle_picking()
			obj.nv = 3;
			l = 0.1;
			obj.v = [-l/2,-l/(2*sqrt(3)); l/2,-l/(2*sqrt(3)); 0,l*sin(pi/3)-l/(2*sqrt(3))]';
			dt = 0.1;
			obj.I = obj.m*l^2/12;

			% trajectory

			th = linspace(0,0,5);
			y = (l/sqrt(3))*sin(pi/6 - th) + linspace(0,0.07,5);
			x = (l/2)*(1 - cos(-3*th/2));

			NT = size(th,2);

			obj.traj.r = [x;y;th];
			obj.traj.dr = zeros(3,5);
			obj.traj.ddr = zeros(3,5);

			% derivatives
			for t = 2:NT-1
				obj.traj.dr(:,t) = (obj.traj.r(:,t+1)-obj.traj.r(:,t-1))/(dt);
			end

			for t = 1:NT
				if t > 1; v1 = obj.traj.r(:,t-1); else; v1 = zeros(3,1); end;
				if t < NT; v2 = obj.traj.r(:,t+1); else; v2 = zeros(3,1); end;

				obj.traj.ddr(:,t) = (-2*obj.traj.r(:,t)+v1+v2)/(dt)^2;
			end

			% external force constraints
			ext_f = {};
			ext_f{1} = struct(); ext_f{1}.fc1 = [-0.0,0]; ext_f{1}.fc2 = [0.0,0];
			ext_f{2} = struct(); ext_f{2}.fc1 = [-0.0,0]; ext_f{2}.fc2 = [0.0,0];
			ext_f{3} = struct(); ext_f{3}.fc1 = [0,0]; ext_f{3}.fc2 = [0,0];

			% constraints for the entire plan
			obj.ext_f = {};
			for v = 1:obj.nv
				obj.ext_f{v} = struct();
				obj.ext_f{v}.fc1 = zeros(2,NT);
				obj.ext_f{v}.fc2 = zeros(2,NT);
				obj.ext_f{v}.jac = zeros(2,3,NT);

				for t = 1:NT
					obj.ext_f{v}.fc1(:,t) = ext_f{v}.fc1;
					obj.ext_f{v}.fc2(:,t) = ext_f{v}.fc2;
					obj.ext_f{v}.jac(:,:,t) = 0; % only contact point is static
				end
			end

			% verticwes of cones
			v1 = [-1,1];
			v2 = [1,1];

			% lines for reference
			lines_init = {};
			lines_init{1} = struct(); lines_init{1}.v1 = obj.v(:,1); lines_init{1}.v2 = obj.v(:,2);
			lines_init{1}.fc1 = v1'; lines_init{1}.fc2 = v2';

			lines_init{2} = struct(); lines_init{2}.v1 = obj.v(:,2); lines_init{2}.v2 = obj.v(:,3);
			rot_1 = [cos(pi/3+pi/2),-sin(pi/3+pi/2);sin(pi/3+pi/2),cos(pi/3+pi/2)];
			lines_init{2}.fc1 = rot_1*v1'; lines_init{2}.fc2 = rot_1*v2';

			lines_init{3} = struct(); lines_init{3}.v1 = obj.v(:,3); lines_init{3}.v2 = obj.v(:,1);
			rot_2 = [cos(-pi/3-pi/2),-sin(-pi/3-pi/2);sin(-pi/3-pi/2),cos(-pi/3-pi/2)];
			lines_init{3}.fc1 = rot_2*v1'; lines_init{3}.fc2 = rot_2*v2';

			% lines for the entire plan
			obj.lines = {};

			% applies the transformation to each line segment
			for l = 1:obj.nv
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
			for i = 1:obj.nv
				idx_1 = i;
				idx_2 = i+1;
				if idx_2 > obj.nv; idx_2 = 1; end;
				regions_init{i} = struct(); 
				res = inv([obj.v(:,idx_1)';obj.v(:,idx_2)'])*[1;1];
				regions_init{i}.A = -res'; regions_init{i}.b = -1;
			end 

			regions_init{4}.A = [0,-1]; regions_init{4}.b = -l*sin(pi/3)-l/(2*sqrt(3));

			% regions during the motion
			obj.regions = {};
			for r = 1:4
				obj.regions{r} = struct();
				obj.regions{r}.A = zeros(size(regions_init{r}.A,1),size(regions_init{r}.A,2),NT);
				obj.regions{r}.b = zeros(size(regions_init{r}.b,1),NT);
				% reorients the planes through time
				for t = 1:NT
					th = obj.traj.r(3,t); trans = -obj.traj.r(1:2,t);
					rotmat = [cos(th),-sin(th);sin(th),cos(th)];
					obj.regions{r}.A(:,:,t) = regions_init{r}.A*inv(rotmat);
					obj.regions{r}.b(:,t) = regions_init{r}.b - regions_init{r}.A*inv(rotmat)*trans;
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