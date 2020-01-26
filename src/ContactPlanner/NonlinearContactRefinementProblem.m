classdef NonlinearContactRefinementProblem
	properties 
		prog
		object
		opti

		N_l
		N_c
		N_T
		N_v

		vars
		idx
		sol
		results
	end

	methods

		function obj = NonlinearContactRefinementProblem(prog)
			assert(nargin > 0);

			import casadi.*

			% loads parameters
			obj.prog = prog;
			obj.object = prog.object;
			obj.N_c = prog.N_c;
			obj.N_l = prog.N_l;
			obj.N_T = prog.N_T;
			obj.N_v = prog.object.nv;

			% defines the program
			obj.opti = Opti()
			obj.vars = struct();
			obj.idx = struct();
		end

		function obj = addDecisionVariables(obj)
			% defines the decision variables
			obj.vars.p = obj.opti.variable(2*obj.N_c*obj.N_l*obj.N_T);
			obj.vars.dp = obj.opti.variable(2*obj.N_c*obj.N_l*obj.N_T);
			obj.vars.ddp = obj.opti.variable(2*obj.N_c*obj.N_l*obj.N_T);

			obj.vars.p_lambda = obj.opti.variable(2*obj.N_c*obj.N_l*obj.N_T);
			obj.vars.f_lambda = obj.opti.variable(2*obj.N_c*obj.N_l*obj.N_T);
			obj.vars.f_ext_lambda = obj.opti.variable(2*obj.N_v*obj.N_T);

			obj.vars.f = obj.opti.variable(2*obj.N_c*obj.N_l*obj.N_T);
			obj.vars.f_ext = obj.opti.variable(2*obj.N_v*obj.N_T);

			% computes the indexes for each variable
			obj.idx.p = reshape(1:2*obj.N_c*obj.N_l*obj.N_T,[2,obj.N_c,obj.N_l,obj.N_T]);
			obj.idx.dp = reshape(1:2*obj.N_c*obj.N_l*obj.N_T,[2,obj.N_c,obj.N_l,obj.N_T]);
			obj.idx.ddp = reshape(1:2*obj.N_c*obj.N_l*obj.N_T,[2,obj.N_c,obj.N_l,obj.N_T]);

			obj.idx.p_lambda = reshape(1:2*obj.N_c*obj.N_l*obj.N_T,[2,obj.N_c,obj.N_l,obj.N_T]);
			obj.idx.f_lambda = reshape(1:2*obj.N_c*obj.N_l*obj.N_T,[2,obj.N_c,obj.N_l,obj.N_T]);
			obj.idx.f_ext_lambda = reshape(1:2*obj.N_v*obj.N_T,[2,obj.N_v,obj.N_T]);

			obj.idx.f = reshape(1:2*obj.N_c*obj.N_l*obj.N_T,[2,obj.N_c,obj.N_l,obj.N_T]);
			obj.idx.f_ext = reshape(1:2*obj.N_v*obj.N_T,[2,obj.N_v,obj.N_T]);
		end

		function obj = addDynamicsConstraints(obj)
			% loads the traj
			traj = obj.object.traj;
			% adds the dynamics constraints
			for t = 1:obj.N_T
				% linear dynamics	
				lhs_x = 0;
				lhs_y = 0;

				% active contacts
				for l = 1:obj.N_l
					for c = 1:obj.N_c
						lhs_x = lhs_x + obj.vars.f(obj.idx.f(1,c,l,t));
						lhs_y = lhs_y + obj.vars.f(obj.idx.f(2,c,l,t));
					end
				end

				% external contacts
				for v = 1:obj.N_v
					lhs_x = lhs_x + obj.vars.f_ext(obj.idx.f_ext(1,v,t));
					lhs_y = lhs_y + obj.vars.f_ext(obj.idx.f_ext(2,v,t));
				end

				obj.opti.subject_to(lhs_x == obj.object.m*obj.object.traj.ddr(1,t))
				obj.opti.subject_to(lhs_y == obj.object.m*9.8 + obj.object.m*obj.object.traj.ddr(2,t))

				% rotational dynamics	
				lhs = 0;

				% active contacts
				for l = 1:obj.N_l
					for c = 1:obj.N_c
						lhs = lhs + obj.vars.f(obj.idx.f(2,c,l,t))*(obj.vars.p(obj.idx.p(1,c,l,t)) - obj.object.traj.r(1,t));
						lhs = lhs - obj.vars.f(obj.idx.f(1,c,l,t))*(obj.vars.p(obj.idx.p(2,c,l,t)) - obj.object.traj.r(2,t));
					end
				end

				% external contacts
				for v = 1:obj.N_v
					rot_mat = [cos(traj.r(3,t)),-sin(traj.r(3,t));sin(traj.r(3,t)),cos(traj.r(3,t))];
					dp = rot_mat*obj.object.v(:,v);
					lhs = lhs + obj.vars.f(obj.idx.f(2,c,l,t))*dp(1);
					lhs = lhs - obj.vars.f(obj.idx.f(1,c,l,t))*dp(2);
				end

				obj.opti.subject_to(lhs == obj.object.I*obj.object.traj.ddr(3,t));
			end
		end

		function obj = addContactConstraints(obj)
			% adds the contact constraints for each time-step
			for t = 1:obj.N_T
				% Environmental forces
				for v = 1:obj.N_v
					% constraints the y value

					% lambda values
					obj.opti.subject_to(0 <= obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(1,v,t)) <= 1);
					obj.opti.subject_to(0 <= obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(2,v,t)) <= 1);

					lhs_x = 0;
					lhs_y = 0;

					lhs_x = lhs_x + obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(1,v,t))*obj.object.ext_f{v}.fc1(1,t);
					lhs_x = lhs_x + obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(2,v,t))*obj.object.ext_f{v}.fc2(1,t);

					lhs_y = lhs_y + obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(1,v,t))*obj.object.ext_f{v}.fc1(2,t);
					lhs_y = lhs_y + obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(2,v,t))*obj.object.ext_f{v}.fc2(2,t);

					obj.opti.subject_to(lhs_x - obj.vars.f_ext(obj.idx.f_ext(1,v,t)) == 0)
					obj.opti.subject_to(lhs_y - obj.vars.f_ext(obj.idx.f_ext(2,v,t)) == 0)

					% defines the contact mode
					if obj.prog.vars.ext_modes.value(1,v,t) == 1
						obj.opti.subject_to(obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(2,v,t)) == 0);
					elseif obj.prog.vars.ext_modes.value(3,v,t) == 1
						obj.opti.subject_to(obj.vars.f_ext_lambda(obj.idx.f_ext_lambda(1,v,t)) == 0);
					end
				end
				% active forces
				for l = 1:obj.N_l
					for c = 1:obj.N_c
						% constrains position lambda
						obj.opti.subject_to(obj.vars.p_lambda(obj.idx.p_lambda(1,c,l,t)) >= 0);
						obj.opti.subject_to(obj.vars.p_lambda(obj.idx.p_lambda(2,c,l,t)) >= 0);
						obj.opti.subject_to(obj.vars.p_lambda(obj.idx.p_lambda(1,c,l,t)) + obj.vars.p_lambda(obj.idx.p_lambda(2,c,l,t)) == 1);

						% constrains force lambda
						obj.opti.subject_to(obj.vars.f_lambda(obj.idx.f_lambda(1,c,l,t)) >= 0);
						obj.opti.subject_to(obj.vars.f_lambda(obj.idx.f_lambda(2,c,l,t)) >= 0);

						for f = 1:obj.prog.N_f
							% only add constraint if finger is in contact
							if sum(obj.prog.vars.L.value(f,c,l,t)) == 1
								% defines the position constraint
								lhs_x = 0;
								lhs_y = 0;

								lhs_x = lhs_x + obj.vars.p_lambda(obj.idx.p_lambda(1,c,l,t))*obj.object.lines{f}.v1(1,t);
								lhs_x = lhs_x + obj.vars.p_lambda(obj.idx.p_lambda(2,c,l,t))*obj.object.lines{f}.v2(1,t);

								lhs_y = lhs_y + obj.vars.p_lambda(obj.idx.p_lambda(1,c,l,t))*obj.object.lines{f}.v1(2,t);
								lhs_y = lhs_y + obj.vars.p_lambda(obj.idx.p_lambda(2,c,l,t))*obj.object.lines{f}.v2(2,t);

								obj.opti.subject_to(lhs_x - obj.vars.p(obj.idx.p(1,c,l,t)) == 0)
								obj.opti.subject_to(lhs_y - obj.vars.p(obj.idx.p(2,c,l,t)) == 0)

								% defines the force constraint
								lhs_x = 0;
								lhs_y = 0;

								lhs_x = lhs_x + obj.vars.f_lambda(obj.idx.f_lambda(1,c,l,t))*obj.object.lines{f}.fc1(1,t);
								lhs_x = lhs_x + obj.vars.f_lambda(obj.idx.f_lambda(2,c,l,t))*obj.object.lines{f}.fc2(1,t);

								lhs_y = lhs_y + obj.vars.f_lambda(obj.idx.f_lambda(1,c,l,t))*obj.object.lines{f}.fc1(2,t);
								lhs_y = lhs_y + obj.vars.f_lambda(obj.idx.f_lambda(2,c,l,t))*obj.object.lines{f}.fc2(2,t);

								obj.opti.subject_to(lhs_x - obj.vars.f(obj.idx.f(1,c,l,t)) == 0)
								obj.opti.subject_to(lhs_y - obj.vars.f(obj.idx.f(2,c,l,t)) == 0)
							end
						end

						% forces are off without contact
						if sum(obj.prog.vars.L.value(:,c,l,t)) == 0
							obj.opti.subject_to(obj.vars.f(obj.idx.f(1,c,l,t)) == 0);
							obj.opti.subject_to(obj.vars.f(obj.idx.f(2,c,l,t)) == 0);
							if t > 0
								% obj.opti.subject_to(obj.vars.p(obj.idx.p(1,c,l,t)) == 0);
								% obj.opti.subject_to(obj.vars.p(obj.idx.p(2,c,l,t)) == 0);
							end
						end
					end
				end
			end

			% time-step
			dt = obj.prog.dt;

			% performs euler integration for splines
			for t = 2:obj.N_T
				for c = 1:obj.N_c
					for l = 1:obj.N_l
						obj.opti.subject_to(obj.vars.p(obj.idx.p(1,c,l,t))-obj.vars.p(obj.idx.p(1,c,l,t-1)) - dt*obj.vars.dp(obj.idx.dp(1,c,l,t)) == 0);
						obj.opti.subject_to(obj.vars.p(obj.idx.p(2,c,l,t))-obj.vars.p(obj.idx.p(2,c,l,t-1)) - dt*obj.vars.dp(obj.idx.dp(2,c,l,t)) == 0);

						obj.opti.subject_to(obj.vars.p(obj.idx.dp(1,c,l,t))-obj.vars.p(obj.idx.dp(1,c,l,t-1)) - dt*obj.vars.ddp(obj.idx.ddp(1,c,l,t)) == 0);
						obj.opti.subject_to(obj.vars.p(obj.idx.dp(2,c,l,t))-obj.vars.p(obj.idx.dp(2,c,l,t-1)) - dt*obj.vars.ddp(obj.idx.ddp(2,c,l,t)) == 0);
					end
				end
			end
		end
		
		function obj = SolveOpti(obj)
			% solves the NLP
			obj.opti.minimize(sum(sum(obj.vars.ddp).^2));
			obj.opti.minimize(sum(sum(obj.vars.f).^2));
			obj.opti.minimize(sum(sum(obj.vars.f_ext).^2));
			obj.opti.solver('ipopt');
			obj.sol = obj.opti.solve();

			% retrives the results
			obj.results.p = reshape(obj.sol.value(obj.vars.p),[2,obj.N_c,obj.N_l,obj.N_T]);
			obj.results.f = reshape(obj.sol.value(obj.vars.f),[2,obj.N_c,obj.N_l,obj.N_T]);
			obj.results.f_ext= reshape(obj.sol.value(obj.vars.f_ext),[2,obj.N_v,obj.N_T]);
		end
	end

end