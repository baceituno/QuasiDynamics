classdef sd_pivot
	properties
		m = 0.01
		I = 1.2
		g = 9.8
		nv
		v
		regions
		lines
		traj
		env
		env_regions
		ext_f
	end

	methods
		function obj = sd_pivot()
			obj.nv = 8;
			x = [-2 , -2.0, 1.0, 1.0, 2.0, 2.0, 1.0, 1];
			y = [1.0, -1.0, -1 , -0.5, -0.5, 0.5, 0.5, 1];
			obj.v = [x;y]/20;
			dt = 0.1;
			b = 3; h = 2;
			obj.I = obj.m*b*h*(b^2 + h^2)/12;

			% trajectory
			th = -[((linspace(0,(pi/8),5)))];
			NT = size(th,2);

			r = zeros(2,NT);

			for t = 1:NT
				rot = [cos(th(t)),-sin(th(t));sin(th(t)),cos(th(t))];
				r(:,t) = [0.05;0.0] - rot*obj.v(:,3);
			end

			y = r(2,:);
			x = r(1,:);

			obj.traj.r = [x;y;th];
			obj.traj.dr = zeros(3,NT);
			obj.traj.ddr = zeros(3,NT);

			obj.traj.dr = zeros(3,NT);
			obj.traj.ddr = zeros(3,NT);			

			% derivatives
			for t = 2:NT
				obj.traj.dr(:,t) = (obj.traj.r(:,t)-obj.traj.r(:,t-1))/(dt);
			end

			for t = 2:NT-1
				if t > 1; v1 = obj.traj.r(:,t-1); else; v1 = zeros(3,1); end;
				if t < NT; v2 = obj.traj.r(:,t+1); else; v2 = zeros(3,1); end;

				obj.traj.ddr(:,t) = (-2*obj.traj.r(:,t)+v1+v2)/(dt)^2;
			end

			% external force constraints
			ext_f = {};
			ext_f{1} = struct(); ext_f{1}.fc1 = [0,0]; ext_f{1}.fc2 = [0,0];
			ext_f{2} = struct(); ext_f{2}.fc1 = 0*[-0.1,1]; ext_f{2}.fc2 = 0*[0.1,1];
			ext_f{3} = struct(); ext_f{3}.fc1 = [-0.01,1]; ext_f{3}.fc2 = [0.01,1];
			ext_f{4} = struct(); ext_f{4}.fc1 = [0,0]; ext_f{4}.fc2 = [0,0];
			ext_f{5} = struct(); ext_f{5}.fc1 = [0,0]; ext_f{5}.fc2 = [0,0];
			ext_f{6} = struct(); ext_f{6}.fc1 = [0,0]; ext_f{6}.fc2 = [0,0];
			ext_f{7} = struct(); ext_f{7}.fc1 = [0,0]; ext_f{7}.fc2 = [0,0];
			ext_f{8} = struct(); ext_f{8}.fc1 = [0,0]; ext_f{8}.fc2 = [0,0];

			% constraints for the entire plan
			obj.ext_f = {};
			for v = 1:obj.nv
				obj.ext_f{v} = struct();
				obj.ext_f{v}.fc1 = zeros(2,5);
				obj.ext_f{v}.fc2 = zeros(2,5);
				obj.ext_f{v}.jac = zeros(2,3,5);
				for t = 1:NT
					obj.ext_f{v}.fc1(:,t) = ext_f{v}.fc1;
					obj.ext_f{v}.fc2(:,t) = ext_f{v}.fc2;
					obj.ext_f{v}.jac(:,:,t) = [eye(2),zeros(2,1)];
				end
			end

			% lines for reference
			lines_init = {};
			for i = 1:obj.nv
				% second index
				idx2 = i+1; 
				if idx2 > obj.nv; idx2 = 1; end;
				% computes the line and the direction friction cone
				lines_init{i} = struct(); lines_init{i}.v1 = obj.v(:,i); lines_init{i}.v2 = obj.v(:,idx2);
				lines_init{i}.t = (lines_init{i}.v2-lines_init{i}.v1)/norm(lines_init{i}.v2-lines_init{i}.v1);

				th = atan2(lines_init{i}.t(2),lines_init{i}.t(1));
				if th < 0; th = pi + th; end;
				rot = [cos(th),-sin(th);sin(th),cos(th)];
				lines_init{i}.fc1 = rot*[-1;1]; lines_init{i}.fc2 = rot*[1;1];
			end

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
			regions_init{1} = struct(); regions_init{1}.A = [1,0]; regions_init{1}.b = [-0.1];
			regions_init{2} = struct(); regions_init{2}.A = [0,1]; regions_init{2}.b = [-0.05];
			regions_init{3} = struct(); regions_init{3}.A = [0,-1]; regions_init{3}.b = [-0.05];
			regions_init{4} = struct(); regions_init{4}.A = [-1,0;0,-1]; regions_init{4}.b = [-0.05;-0.025];
			regions_init{5} = struct(); regions_init{5}.A = [-1,0;0,1]; regions_init{5}.b = [-0.05;-0.025];
			regions_init{6} = struct(); regions_init{6}.A = [-1,0]; regions_init{6}.b = [0.1];

			% regions during the motion
			obj.regions = {};
			for r = 1:6
				obj.regions{r} = struct();
				obj.regions{r}.A = zeros(size(regions_init{r}.A,1),size(regions_init{r}.A,2),5);
				obj.regions{r}.b = zeros(size(regions_init{r}.b,1),5);
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