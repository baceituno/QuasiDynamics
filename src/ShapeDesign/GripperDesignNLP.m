classdef GripperDesignNLP
	properties 
		prog
		object
		opti

		N_l
		N_c
		N_T
		N_v
		traj
		poly_order = 3
		
		vars
		idx
		sol
		shape
		results
		basis = 'poly'

		pick_contacts = 0
	end

	methods

		function obj = GripperDesignNLP(prog, poly_order)
			assert(nargin > 0);
			if nargin < 2; poly_order = 3; end;

			import casadi.*

			% loads parameters
			obj.prog = prog;
			obj.object = prog.object;
			obj.N_c = prog.N_c;
			obj.N_l = prog.N_l;
			obj.N_T = prog.N_T;
			obj.N_v = prog.object.nv;
			obj.poly_order = poly_order;
			obj.traj = prog.object.traj;

			% defines the program
			obj.opti = Opti()
			obj.vars = struct();
			obj.idx = struct();
			obj.shape = struct();
		end

		function obj = addDecisionVariables(obj)
			% defines the decision variables
			obj.vars.coeff = obj.opti.variable(obj.N_l*(obj.poly_order+1));
			obj.vars.len = obj.opti.variable(obj.N_l);
			
			obj.vars.trans = obj.opti.variable(3*obj.N_l*obj.N_T);
			obj.vars.dtrans = obj.opti.variable(3*obj.N_l*obj.N_T);
			obj.vars.ddtrans = obj.opti.variable(3*obj.N_l*obj.N_T);

			obj.vars.contact_cons = obj.opti.variable(obj.N_l*obj.N_c*obj.N_T);
			% obj.vars.contact_pt = obj.opti.variable(2*obj.N_l*obj.N_c*obj.N_T);
			obj.vars.dcontact_cons = obj.opti.variable(obj.N_l*obj.N_c*obj.N_T);

			% computes the indexes for each variable
			obj.idx.coeff = reshape(1:obj.N_l*(obj.poly_order+1),[1,obj.N_l,obj.poly_order+1]);
			obj.idx.len = reshape(1:obj.N_l,[1,obj.N_l]);
			
			obj.idx.trans = reshape(1:3*obj.N_l*obj.N_T, [3,obj.N_l,obj.N_T]);
			obj.idx.dtrans = reshape(1:3*obj.N_l*obj.N_T, [3,obj.N_l,obj.N_T]);
			obj.idx.ddtrans = reshape(1:3*obj.N_l*obj.N_T, [3,obj.N_l,obj.N_T]);

			obj.idx.contact_cons = reshape(1:obj.N_l*obj.N_c*obj.N_T, [obj.N_c,obj.N_l,obj.N_T]);
			% obj.idx.contact_pt = reshape(1:2*obj.N_l*obj.N_c*obj.N_T, [2,obj.N_c,obj.N_l,obj.N_T]);
			obj.idx.dcontact_cons = reshape(1:obj.N_l*obj.N_c*obj.N_T, [obj.N_c,obj.N_l,obj.N_T]);

			% adds contact optimization variables
			if obj.pick_contacts
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
		end

		function obj = addDynamicsConstraints(obj)
			% only adds constraints when searching for contacts as well
			if(obj.pick_contacts == 1)
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
							obj.opti.subject_to(0.1 >= obj.vars.f_lambda(obj.idx.f_lambda(1,c,l,t)) >= 0);
							obj.opti.subject_to(0.1 >= obj.vars.f_lambda(obj.idx.f_lambda(2,c,l,t)) >= 0);

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
								if t > 1
									obj.opti.subject_to(obj.vars.p(obj.idx.p(1,c,l,t)) == obj.vars.p(obj.idx.p(1,c,l,t-1)));
									obj.opti.subject_to(obj.vars.p(obj.idx.p(2,c,l,t)) == obj.vars.p(obj.idx.p(2,c,l,t-1)));
								end
							end
						end
					end
				end

				% time-step
				dt = obj.prog.dt;

				% performs euler integration for the dynamics
				for t = 2:obj.N_T
					for c = 1:obj.N_c
						for l = 1:obj.N_l
							obj.opti.subject_to(obj.vars.p(obj.idx.p(1,c,l,t))-obj.vars.p(obj.idx.p(1,c,l,t-1)) - dt*obj.vars.dp(obj.idx.dp(1,c,l,t)) == 0);
							obj.opti.subject_to(obj.vars.p(obj.idx.p(2,c,l,t))-obj.vars.p(obj.idx.p(2,c,l,t-1)) - dt*obj.vars.dp(obj.idx.dp(2,c,l,t)) == 0);

							obj.opti.subject_to(obj.vars.dp(obj.idx.dp(1,c,l,t))-obj.vars.dp(obj.idx.dp(1,c,l,t-1)) - dt*obj.vars.ddp(obj.idx.ddp(1,c,l,t)) == 0);
							obj.opti.subject_to(obj.vars.dp(obj.idx.dp(2,c,l,t))-obj.vars.dp(obj.idx.dp(2,c,l,t-1)) - dt*obj.vars.ddp(obj.idx.ddp(2,c,l,t)) == 0);
						end
					end
				end 
			end
		end

		function obj = addContactConstraints(obj)
			% obj.opti.subject_to(obj.vars.coeff(obj.poly_order+2) == 0)

			% adds the contact constraints for each time-step
			for t = 1:obj.N_T
				for l = 1:obj.N_l
					% acceleration limits
					obj.opti.subject_to(-2 <= obj.vars.ddtrans(obj.idx.ddtrans(1,l,t)) <= 2);
					obj.opti.subject_to(-2 <= obj.vars.ddtrans(obj.idx.ddtrans(2,l,t)) <= 2);
					obj.opti.subject_to(-10 <= obj.vars.ddtrans(obj.idx.ddtrans(3,l,t)) <= 10);

					% length limits
					obj.opti.subject_to(0 <= obj.vars.len(obj.idx.len(1,l)) <= 0.02);
					
					for c = 1:obj.N_c
						% polynoimial variable has a range from 0 to len
						obj.opti.subject_to(0 <= obj.vars.contact_cons(obj.idx.contact_cons(c,l,t)) <= obj.vars.len(obj.idx.len(1,l)))

						% only add constraint if finger is in contact
						if sum(obj.prog.vars.L.value(:,c,l,t)) >= 1

							% Contact constraAint
							lhs_x = 0;
							lhs_y = 0;

							for i = 1:obj.poly_order+1
								% power computation
								elem = basis_func(obj.basis,obj.vars.contact_cons(obj.idx.contact_cons(c,l,t)),i);
								pow_y = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;

								% constraint over both axes
								lhs_x = lhs_x - sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
								lhs_y = lhs_y + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
							end
							elem = obj.vars.contact_cons(obj.idx.contact_cons(c,l,t));
							lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*elem;
							lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*elem;

							if obj.pick_contacts == 1
								obj.opti.subject_to(lhs_x + obj.vars.trans(obj.idx.trans(1,l,t)) - obj.vars.p(obj.idx.p(1,c,l,t)) == 0);
								obj.opti.subject_to(lhs_y + obj.vars.trans(obj.idx.trans(2,l,t)) - obj.vars.p(obj.idx.p(2,c,l,t)) == 0);
							else
								obj.opti.subject_to(lhs_x + obj.vars.trans(obj.idx.trans(1,l,t)) == obj.prog.vars.p.value(1,c,l,t))
								obj.opti.subject_to(lhs_y + obj.vars.trans(obj.idx.trans(2,l,t)) == obj.prog.vars.p.value(2,c,l,t))
							end

							% jacobian constraint
							vel1 = 0;
							vel2 = 0;
							lhs_x = 0;
							lhs_y = 0;

							% Çoefficients
							for i = 1:obj.poly_order+1
								% power computation
								elem = basis_func(obj.basis,obj.vars.contact_cons(obj.idx.contact_cons(c,l,t)),i);
								pow_y = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;

								% constraint over both axes
								lhs_x = lhs_x - sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
								lhs_y = lhs_y + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
							end

							elem = obj.vars.contact_cons(obj.idx.contact_cons(c,l,t));
							lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*elem;
							lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*elem;

							vel1 = lhs_x*obj.vars.dtrans(obj.idx.dtrans(3,l,t));
							vel2 = lhs_y*obj.vars.dtrans(obj.idx.dtrans(3,l,t));

							if obj.pick_contacts == 1
								fx = obj.vars.f(obj.idx.f(1,c,l,t));
								fy = obj.vars.f(obj.idx.f(2,c,l,t));
								% obj.opti.subject_to(vel1*fy - vel2*fx == 0)
								% obj.opti.subject_to(vel1*fx + vel2*fy >= 0.0)
							else
								fx = obj.prog.vars.f.value(1,c,l,t);
								fy = obj.prog.vars.f.value(1,c,l,t);
								% obj.opti.subject_to(vel1*fy - vel2*fx == 0)
								% obj.opti.subject_to(vel1*fx + vel2*fy >= 0.0)
							end

							% obj.opti.subject_to(lhs_x + obj.vars.trans(obj.idx.trans(1,l,t)) - obj.vars.contact_pt(obj.idx.contact_pt(1,c,l,t)) == 0);
							% obj.opti.subject_to(lhs_y + obj.vars.trans(obj.idx.trans(2,l,t)) - obj.vars.contact_pt(obj.idx.contact_pt(2,c,l,t)) == 0);
						end
					end
				end
			end

			% time-step
			dt = obj.prog.dt;

			% initial conditions
			for l = 1:obj.N_l
				% obj.opti.subject_to(obj.vars.trans(obj.idx.trans(3,l,1)) == 0);

				obj.opti.subject_to(obj.vars.dtrans(obj.idx.dtrans(1,l,1)) == 0);
				obj.opti.subject_to(obj.vars.dtrans(obj.idx.dtrans(2,l,1)) == 0);
				obj.opti.subject_to(obj.vars.dtrans(obj.idx.dtrans(3,l,1)) == 0);

				obj.opti.subject_to(obj.vars.ddtrans(obj.idx.dtrans(1,l,1)) == 0);
				obj.opti.subject_to(obj.vars.ddtrans(obj.idx.dtrans(2,l,1)) == 0);
				obj.opti.subject_to(obj.vars.ddtrans(obj.idx.dtrans(3,l,1)) == 0);
			end

			% performs euler integration for splines
			for t = 2:obj.N_T
				for l = 1:obj.N_l
					% contact spot
					for c = 1:obj.N_c
						for l = 1:obj.N_l
							obj.opti.subject_to(obj.vars.contact_cons(obj.idx.contact_cons(c,l,t))-obj.vars.contact_cons(obj.idx.contact_cons(c,l,t-1)) - dt*obj.vars.dcontact_cons(obj.idx.dcontact_cons(c,l,t)) == 0);
						end
					end
					obj.opti.subject_to(obj.vars.trans(obj.idx.trans(1,l,t))-obj.vars.trans(obj.idx.trans(1,l,t-1)) - dt*obj.vars.dtrans(obj.idx.dtrans(1,l,t)) == 0);
					obj.opti.subject_to(obj.vars.trans(obj.idx.trans(2,l,t))-obj.vars.trans(obj.idx.trans(2,l,t-1)) - dt*obj.vars.dtrans(obj.idx.dtrans(2,l,t)) == 0);
					obj.opti.subject_to(obj.vars.trans(obj.idx.trans(3,l,t))-obj.vars.trans(obj.idx.trans(3,l,t-1)) - dt*obj.vars.dtrans(obj.idx.dtrans(3,l,t)) == 0);

					obj.opti.subject_to(obj.vars.dtrans(obj.idx.dtrans(1,l,t))-obj.vars.dtrans(obj.idx.dtrans(1,l,t-1)) - dt*obj.vars.ddtrans(obj.idx.ddtrans(1,l,t)) == 0);
					obj.opti.subject_to(obj.vars.dtrans(obj.idx.dtrans(2,l,t))-obj.vars.dtrans(obj.idx.dtrans(2,l,t-1)) - dt*obj.vars.ddtrans(obj.idx.ddtrans(2,l,t)) == 0);
					obj.opti.subject_to(obj.vars.dtrans(obj.idx.dtrans(3,l,t))-obj.vars.dtrans(obj.idx.dtrans(3,l,t-1)) - dt*obj.vars.ddtrans(obj.idx.ddtrans(3,l,t)) == 0);
				end
			end
		end

		function obj = addSquareNonPenetrationConstraints(obj)
			% checks that we are using a square
			assert(obj.object.nv == 4)
			traj = obj.traj;
			
			% for all links
			for l = 1:obj.N_l
				for t = 1:obj.N_T
					% constrains the separation between knot point
					rang = linspace(0,1,11);
					for i = 1:length(rang)-1
						t_piv_1 = rang(i)*obj.vars.len(obj.idx.len(1,l));
						t_piv_2 = rang(i+1)*obj.vars.len(obj.idx.len(1,l));

						% computes the knot point in world frame
						lhs_x = 0; lhs_y = 0;
						x_piv_1 = 0; y_piv_1 = 0;
						for i = 1:obj.poly_order+1
							% power computation
							elem = basis_func(obj.basis,t_piv_1,i);
							pow_y = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
							lhs_x = lhs_x - sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
							lhs_y = lhs_y + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
						end

						elem = t_piv_1;
						lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*elem + obj.vars.trans(obj.idx.trans(1,l,t));
						lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*elem + obj.vars.trans(obj.idx.trans(2,l,t));

						% transforms to object frame
						rot_mat = [cos(traj.r(3,t)-pi/4),-sin(traj.r(3,t)-pi/4);sin(traj.r(3,t)-pi/4),cos(traj.r(3,t)-pi/4)];
						
						x_piv_1 = rot_mat(1,1)*(lhs_x - traj.r(1,t)) + rot_mat(1,2)*(lhs_y - traj.r(2,t));
						y_piv_1 = rot_mat(2,1)*(lhs_x - traj.r(1,t)) + rot_mat(2,2)*(lhs_y - traj.r(2,t));

						lhs_x = 0; lhs_y = 0;
						x_piv_2 = 0; y_piv_2 = 0;
						for i = 1:obj.poly_order+1
							elem = basis_func(obj.basis,t_piv_2,i);
							pow_y = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
							lhs_x = lhs_x - sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
							lhs_y = lhs_y + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
						end

						elem = t_piv_2;
						lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*elem + obj.vars.trans(obj.idx.trans(1,l,t));
						lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*elem + obj.vars.trans(obj.idx.trans(2,l,t));

						% transforms to object frame
						rot_mat = [cos(traj.r(3,t)-pi/4),-sin(traj.r(3,t)-pi/4);sin(traj.r(3,t)-pi/4),cos(traj.r(3,t)-pi/4)];
						
						x_piv_2 = rot_mat(1,1)*(lhs_x - traj.r(1,t)) + rot_mat(1,2)*(lhs_y - traj.r(2,t));
						y_piv_2 = rot_mat(2,1)*(lhs_x - traj.r(1,t)) + rot_mat(2,2)*(lhs_y - traj.r(2,t));

						obj.opti.subject_to((x_piv_1 - x_piv_2)^2 + (y_piv_1 - y_piv_2)^2 <= 1e-3);
					end
  
					% ensures that the shape is outside of the square
					for t_piv = linspace(0,1,44)
						% computes the knot point in world frame
						lhs_x = 0; lhs_y = 0;
						x_piv = 0; y_piv = 0;
						for i = 1:obj.poly_order+1
							elem = basis_func(obj.basis,t_piv*obj.vars.len(obj.idx.len(1,l)),i);
							pow_y = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
							lhs_x = lhs_x - sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
							lhs_y = lhs_y + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
						end

						elem = t_piv*obj.vars.len(obj.idx.len(1,l));
						lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*elem + obj.vars.trans(obj.idx.trans(1,l,t));
						lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*elem + obj.vars.trans(obj.idx.trans(2,l,t));

						% transforms to object frame
						rot_mat = [cos(traj.r(3,t)-pi/4),-sin(traj.r(3,t)-pi/4);sin(traj.r(3,t)-pi/4),cos(traj.r(3,t)-pi/4)];
						
						x_piv = rot_mat(1,1)*(lhs_x - traj.r(1,t)) + rot_mat(1,2)*(lhs_y - traj.r(2,t));
						y_piv = rot_mat(2,1)*(lhs_x - traj.r(1,t)) + rot_mat(2,2)*(lhs_y - traj.r(2,t));

						obj.opti.subject_to(abs(x_piv) + abs(y_piv) >= 0.1/sqrt(2));
					end

					% non-penetration between time-steps
					% for l = 1:obj.N_l
					% 	for c = 1:obj.N_c
					% 		% only add constraint if finger is in contact
					% 		if sum(obj.prog.vars.L.value(:,c,l,t)) >= 1 && t < obj.N_T
					% 			% computes the knot point in world frame
					% 			lhs_x = 0; lhs_y = 0;
					% 			x_piv = 0; y_piv = 0;
					% 			for i = 1:obj.poly_order+1
					% 				elem = basis_func(obj.basis,obj.vars.contact_cons(obj.idx.contact_cons(c,l,t)),i);
					% 				pow_x = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
					% 				pow_y = obj.vars.coeff(obj.idx.coeff(2,l,i))*elem;
					% 				lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_x - sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
					% 				lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
					% 			end

					% 			lhs_x = lhs_x + obj.vars.trans(obj.idx.trans(1,l,t));
					% 			lhs_y = lhs_y + obj.vars.trans(obj.idx.trans(2,l,t));

					% 			% transforms to object frame
					% 			rot_mat = [cos(traj.r(3,t)-pi/4),-sin(traj.r(3,t)-pi/4);sin(traj.r(3,t)-pi/4),cos(traj.r(3,t)-pi/4)];

					% 			x_piv = rot_mat(1,1)*(lhs_x - traj.r(1,t))/2 + rot_mat(1,2)*(lhs_y - traj.r(2,t))/2;
					% 			y_piv = rot_mat(2,1)*(lhs_x - traj.r(1,t))/2 + rot_mat(2,2)*(lhs_y - traj.r(2,t))/2;

					% 			lhs_x = 0; lhs_y = 0;
					% 			for i = 1:obj.poly_order+1
					% 				elem = basis_func(obj.basis,obj.vars.contact_cons(obj.idx.contact_cons(c,l,t)),i);
					% 				pow_x = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
					% 				pow_y = obj.vars.coeff(obj.idx.coeff(2,l,i))*elem;
					% 				lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t+1)))*pow_x - sin(obj.vars.trans(obj.idx.trans(3,l,t+1)))*pow_y;
					% 				lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t+1)))*pow_x + cos(obj.vars.trans(obj.idx.trans(3,l,t+1)))*pow_y;
					% 			end

					% 			lhs_x = lhs_x + obj.vars.trans(obj.idx.trans(1,l,t+1));
					% 			lhs_y = lhs_y + obj.vars.trans(obj.idx.trans(2,l,t+1));

					% 			% transforms to object frame
					% 			rot_mat = [cos(traj.r(3,t+1)-pi/4),-sin(traj.r(3,t+1)-pi/4);sin(traj.r(3,t+1)-pi/4),cos(traj.r(3,t+1)-pi/4)];
								
					% 			x_piv = x_piv + rot_mat(1,1)*(lhs_x - traj.r(1,t+1))/2 + rot_mat(1,2)*(lhs_y - traj.r(2,t+1))/2;
					% 			y_piv = y_piv + rot_mat(2,1)*(lhs_x - traj.r(1,t+1))/2 + rot_mat(2,2)*(lhs_y - traj.r(2,t+1))/2;

					% 			obj.opti.subject_to(abs(x_piv) + abs(y_piv) >= 0.1/(sqrt(2)));
					% 		end
					% 		if sum(obj.prog.vars.L.value(:,c,l,t)) >= 1 && t > 1
					% 			% computes the knot point in world frame
					% 			lhs_x = 0; lhs_y = 0;
					% 			x_piv = 0; y_piv = 0;
					% 			for i = 1:obj.poly_order+1
					% 				elem = basis_func(obj.basis,obj.vars.contact_cons(obj.idx.contact_cons(c,l,t)),i);
					% 				pow_x = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
					% 				pow_y = obj.vars.coeff(obj.idx.coeff(2,l,i))*elem;
					% 				lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_x - sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
					% 				lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_x + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
					% 			end

					% 			lhs_x = lhs_x + obj.vars.trans(obj.idx.trans(1,l,t));
					% 			lhs_y = lhs_y + obj.vars.trans(obj.idx.trans(2,l,t));

					% 			% transforms to object frame
					% 			rot_mat = [cos(traj.r(3,t)-pi/4),-sin(traj.r(3,t)-pi/4);sin(traj.r(3,t)-pi/4),cos(traj.r(3,t)-pi/4)];

					% 			x_piv = rot_mat(1,1)*(lhs_x - traj.r(1,t))/2 + rot_mat(1,2)*(lhs_y - traj.r(2,t))/2;
					% 			y_piv = rot_mat(2,1)*(lhs_x - traj.r(1,t))/2 + rot_mat(2,2)*(lhs_y - traj.r(2,t))/2;

					% 			lhs_x = 0; lhs_y = 0;
					% 			for i = 1:obj.poly_order+1
					% 				elem = basis_func(obj.basis,obj.vars.contact_cons(obj.idx.contact_cons(c,l,t)),i);
					% 				pow_x = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
					% 				pow_y = obj.vars.coeff(obj.idx.coeff(2,l,i))*elem;
					% 				lhs_x = lhs_x + cos(obj.vars.trans(obj.idx.trans(3,l,t-1)))*pow_x - sin(obj.vars.trans(obj.idx.trans(3,l,t-1)))*pow_y;
					% 				lhs_y = lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t-1)))*pow_x + cos(obj.vars.trans(obj.idx.trans(3,l,t-1)))*pow_y;
					% 			end

					% 			lhs_x = lhs_x + obj.vars.trans(obj.idx.trans(1,l,t-1));
					% 			lhs_y = lhs_y + obj.vars.trans(obj.idx.trans(2,l,t-1));

					% 			% transforms to object frame
					% 			rot_mat = [cos(traj.r(3,t-1)-pi/4),-sin(traj.r(3,t-1)-pi/4);sin(traj.r(3,t-1)-pi/4),cos(traj.r(3,t-1)-pi/4)];
								
					% 			x_piv = x_piv + rot_mat(1,1)*(lhs_x - traj.r(1,t-1))/2 + rot_mat(1,2)*(lhs_y - traj.r(2,t-1))/2;
					% 			y_piv = y_piv + rot_mat(2,1)*(lhs_x - traj.r(1,t-1))/2 + rot_mat(2,2)*(lhs_y - traj.r(2,t-1))/2;

					% 			obj.opti.subject_to(abs(x_piv) + abs(y_piv) >= 0.1/(sqrt(2)));
					% 		end
					% 	end
					% end

					% non-penetration with ground
					for t_piv = linspace(0,1,11)
						lhs_y = 0;
						for i = 1:obj.poly_order+1
							elem = basis_func(obj.basis,t_piv*obj.vars.len(obj.idx.len(1,l)),i);
							pow_y = obj.vars.coeff(obj.idx.coeff(1,l,i))*elem;
							lhs_y = lhs_y + cos(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_y;
						end
						pow_x = t_piv*obj.vars.len(obj.idx.len(1,l));
						obj.opti.subject_to(lhs_y + sin(obj.vars.trans(obj.idx.trans(3,l,t)))*pow_x + obj.vars.trans(obj.idx.trans(2,l,t)) >= 0)
					end
				end
			end
		end
		
		function obj = SolveOpti(obj,coeff,len,trans)
			% solves the NLP
			if nargin > 2
				if obj.pick_contacts
					obj.opti.minimize(sum(sum(obj.vars.ddtrans).^2)+sum(sum(obj.vars.f).^2));
				else 
					obj.opti.minimize(sum(sum(obj.vars.ddtrans).^2));
				end	
			end
			
			obj.opti.solver('ipopt');
			if nargin < 2
				obj.opti.set_initial(obj.vars.coeff,[0, zeros(1,obj.N_l*(obj.poly_order+1)-1)]);
				obj.opti.set_initial(obj.vars.len,0);
				obj.opti.set_initial(obj.vars.trans,zeros(1,3*obj.N_l*obj.N_T));
			else
				obj.opti.set_initial(obj.vars.coeff,[coeff',zeros(1,obj.N_l*(obj.poly_order-1))]);
				obj.opti.set_initial(obj.vars.len,len);
				obj.opti.set_initial(obj.vars.trans,trans);
			end
			% obj.opti.set_initial(obj.vars.p,reshape(obj.prog.vars.p.value,[1,[2*obj.N_c*obj.N_l*obj.N_T]]));
			% obj.opti.set_initial(obj.vars.contact_cons,ones(1,obj.N_l*obj.N_c*obj.N_T));
			obj.sol = obj.opti.solve();

			% retrieves the solution
			obj.shape.coeff = obj.sol.value(obj.vars.coeff);
			obj.shape.trans = obj.sol.value(obj.vars.trans);
			obj.shape.contact_cons = obj.sol.value(obj.vars.contact_cons);

			if obj.pick_contacts
				obj.results.p = reshape(obj.sol.value(obj.vars.p),[2,obj.N_c,obj.N_l,obj.N_T]);
				obj.results.f = reshape(obj.sol.value(obj.vars.f),[2,obj.N_c,obj.N_l,obj.N_T]);
				obj.results.f_ext= reshape(obj.sol.value(obj.vars.f_ext),[2,obj.N_v,obj.N_T]);
			else
				obj.results.p = obj.prog.vars.p.value;
				obj.results.f = obj.prog.vars.f.value;
				obj.results.f_ext= obj.prog.vars.f_ext.value;
			end
		end
	end

end