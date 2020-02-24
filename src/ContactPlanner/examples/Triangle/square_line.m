classdef square_line
	properties
		m = 0.01
		I = 1.2
		g = 0
		nv
		v
		regions
		env_regions
		planar = 1
		lines
		traj
		ext_f
		env
	end

	methods
		function obj = square_line()
			obj.nv = 4;
			obj.v = [-1,-1;1,-1;1,1;-1,1]'/20;
			dt = 0.1;
			obj.I = obj.m*0.01^2/3;

			% trajectory

			x = linspace(0,0,5);
			y = linspace(0,0.1,5);
			th = [linspace(0,0*pi/6,5)];
			NT = size(th,2);

			obj.traj.r = [x;y;th];
			obj.traj.dr = zeros(3,NT);
			obj.traj.ddr = zeros(3,NT);

			% derivatives
			for t = 2:NT-1
				obj.traj.dr(:,t) = (obj.traj.r(:,t+1)-obj.traj.r(:,t-1))/(2*dt);
			end

			for t = 2:NT-1
				if t > 1; v1 = obj.traj.r(:,t-1); else; v1 = zeros(3,1); end;
				if t < NT; v2 = obj.traj.r(:,t+1); else; v2 = zeros(3,1); end;

				obj.traj.ddr(:,t) = (-2*obj.traj.r(:,t)+v1+v2)/(dt)^2;
			end

			% constraints for the entire plan
			obj.ext_f = {};
			for v = 1:obj.nv
				obj.ext_f{v} = struct();
				obj.ext_f{v}.fc1 = zeros(2,NT);
				obj.ext_f{v}.fc2 = zeros(2,NT);
				obj.ext_f{v}.jac = zeros(2,3,NT);

				for t = 2:NT-1
					% computes the jacobian and the velocity vector
					vs = [-sin(obj.traj.r(3,t)),-cos(obj.traj.r(3,t));cos(obj.traj.r(3,t)),-sin(obj.traj.r(3,t))]*obj.v(:,v);
					vel = [eye(2),vs]*obj.traj.dr(:,t);

					if v == 1
						vel(1) = vel(1);
					end

					obj.ext_f{v}.fc1(:,t) = -obj.m*9.8*normalize(vel)*0.3;
					obj.ext_f{v}.fc2(:,t) = -obj.m*9.8*normalize(vel)*0.3;
					obj.ext_f{v}.jac(:,:,t) = [eye(2),vs];
				end
			end

			% lines for reference
			lines_init = {};
			lines_init{1} = struct(); lines_init{1}.v1 = [-0.5,-0.5]'/10; lines_init{1}.v2 = [0.5,-0.5]'/10;
			lines_init{1}.fc1 = [1,2]'; lines_init{1}.fc2 = [-1,2]';
			lines_init{1}.t = [-1,0];

			lines_init{2} = struct(); lines_init{2}.v1 = [0.5,-0.5]'/10; lines_init{2}.v2 = [0.5,0.5]'/10;
			lines_init{2}.fc1 = [-2,1]'; lines_init{2}.fc2 = [-2,-1]';
			lines_init{2}.t = [0,-1];

			lines_init{3} = struct(); lines_init{3}.v1 = [0.5,0.5]'/10; lines_init{3}.v2 = [-0.5,0.5]'/10;
			lines_init{3}.fc1 = [1,-2]'; lines_init{3}.fc2 = [-1,-2]';
			lines_init{3}.t = [1,0];

			lines_init{4} = struct(); lines_init{4}.v1 = [-0.5,0.5]'/10; lines_init{4}.v2 = [-0.5,-0.5]'/10;
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
			regions_init{1} = struct(); regions_init{1}.A = [1,0;0,-1]; regions_init{1}.b = [-0.05;0.05];
			regions_init{2} = struct(); regions_init{2}.A = [-1,0;0,-1]; regions_init{2}.b = [-0.05;0.05]; 
			regions_init{3} = struct(); regions_init{3}.A = [0,-1]; regions_init{3}.b = [-0.05]; 
			regions_init{4} = struct(); regions_init{4}.A = [0,1]; regions_init{4}.b = [-0.05]; 

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
		end
	end

end