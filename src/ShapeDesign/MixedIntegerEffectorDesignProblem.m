classdef MixedIntegerEffectorDesignProblem < Quad_MixedIntegerConvexProgram
% Developed by Bernardo Aceituno-C (MIT MCube Lab)
  properties
    object
    N_c
    N_l
    N_T
    N_f
    N_r
    N_s
    N_task
    M = 3
    idx = 0
    idx_sos2 = 0
    dt = 0.1
    McCormick = 1
    McCormick2 = 0
    modes = 0
  end

  methods
    function obj = MixedIntegerEffectorDesignProblem(object, n_l, n_c, n_s)
      % Constructs the optimization problem and declares the variables for each contact
      % @param object: structure with elements:
      %               - object.v        : a 3XNv vector with the position of each vertex wrt to shape    
      %               - object.regions  : a set of convex regions that represent the complement 
      %                                   space of the shape through time
      %               - object.lines    : a set of line segments covering the shape
      %               - object.nv       : number of vertices in all the polygons of the shape
      %               - object.traj     : a 3xN_T vector with the object CoM = [x(t),y(t),th(t)]'
      %               - object.ext_f    : a N_v cell with the contact constraints through time
      % @param n_links: links on the gripper

      assert(nargin >= 1)

      if nargin < 4; n_s = 2; end;
      if nargin < 3; n_c = 2; end;
      if nargin < 2; n_l = 2; end;

      % parses the inputs
      obj.object = object;
      obj.N_T = size(object{1}.traj.r,2);
      obj.N_l = n_l;
      obj.N_c = n_c;
      obj.N_s = n_s;
      obj.N_f = length(object{1}.lines);
      obj.N_r = length(object{1}.regions);
      obj.N_task = length(object);

      % contact locatops
      obj = obj.addVariable('p', 'C', [2, obj.N_c, obj.N_l, obj.N_T, obj.N_task], -inf, inf);
      obj = obj.addVariable('dp', 'C', [2, obj.N_c, obj.N_l, obj.N_T, obj.N_task], -inf, inf);
      obj = obj.addVariable('ddp', 'C', [2, obj.N_c, obj.N_l, obj.N_T, obj.N_task], -inf, inf);

      % contact forces
      obj = obj.addVariable('f', 'C', [2, obj.N_c, obj.N_l, obj.N_T, obj.N_task], -inf, inf);
      obj = obj.addVariable('tau_aux', 'C', [2, obj.N_c, obj.N_l, obj.N_T, obj.N_task], -inf, inf);
      obj = obj.addVariable('f_ext', 'C', [2, obj.object{1}.nv, obj.N_T, obj.N_task], -1, 1);

      obj = obj.addVariable('f_app', 'C', [1, obj.N_T, obj.N_task], -inf, 0);
      obj = obj.addVariable('f_app_mode', 'B', [1, obj.N_T, obj.N_task], 0, 1);

      % pose
      obj = obj.addVariable('r', 'C', [3, obj.N_l, obj.N_T, obj.N_task], -inf, inf);
      obj = obj.addVariable('ddr', 'C', [3, obj.N_l, obj.N_T, obj.N_task], -inf, inf);

      obj = obj.addVariable('sin_th', 'C', [1, obj.N_l, obj.N_T, obj.N_task], -1, 1);
      obj = obj.addVariable('cos_th', 'C', [1, obj.N_l, obj.N_T, obj.N_task], -1, 1);

      % shape description
      obj = obj.addVariable('shape_knots_base', 'C', [2, obj.N_s+1, obj.N_l], -inf, inf);
      obj = obj.addVariable('dds', 'C', [2, obj.N_s+1, obj.N_l], -inf, inf);

      % gripper through time
      obj = obj.addVariable('shape_knots', 'C', [2, obj.N_s+1, obj.N_l, obj.N_T, obj.N_task], -inf, inf);

      % slack variables knot rotations
      obj = obj.addVariable('sin_knot', 'C', [2, obj.N_s+1, obj.N_l, obj.N_T, obj.N_task], -inf, inf);
      obj = obj.addVariable('cos_knot', 'C', [2, obj.N_s+1, obj.N_l, obj.N_T, obj.N_task], -inf, inf);

      % contact points
      obj = obj.addVariable('contact_assignment', 'B', [obj.N_s+1, obj.N_c, obj.N_l, obj.N_task], 0, 1);

      % non-penetration
      obj = obj.addVariable('R', 'B', [obj.N_r, obj.N_s, obj.N_l, obj.N_T, obj.N_task], 0, 1);

      % contact force relevant constraints
      obj = obj.addVariable('lambda', 'C', [2, obj.N_c, obj.N_l, obj.N_T, obj.N_task], 0.1, 0.9);
      obj = obj.addVariable('weight', 'C', [2, obj.N_c, obj.N_l, obj.N_T, obj.N_task], 0, 100);
      obj = obj.addVariable('weight_ext', 'C', [2, obj.object{1}.nv, obj.N_T, obj.N_task], 0, 1);
    end

    function obj = addSOS2Constraint(obj,x,y,x_rang,y_rang)
      % size of the range
      n = length(x_rang);

      % weights and binary variables
      lambda = strcat('sos2_',string(obj.idx_sos2));
      binvar = strcat('binsos2_',string(obj.idx_sos2));
      obj.idx_sos2 = obj.idx_sos2 + 1;

      % adds the varialbes for the problem
      obj = obj.addVariable(lambda, 'C', [n,1], 0, 1);
      obj = obj.addVariable(binvar, 'B', [n-1,1], 0, 1);

      % all weights must add to 1
      Aeq = sparse(2, obj.nv);
      beq = [1;1];

      Aeq(1,obj.vars.(lambda).i(:,1)) = 1;
      Aeq(2,obj.vars.(binvar).i(:,1)) = 1;

      obj = obj.addLinearConstraints([], [], Aeq, beq);

      % value assignment
      Aeq = sparse(2, obj.nv);
      beq = zeros(2,1);

      Aeq(1,x) = -1;
      Aeq(1,obj.vars.(lambda).i(:,1)) = x_rang;
      
      Aeq(2,y) = -1;
      Aeq(2,obj.vars.(lambda).i(:,1)) = y_rang;

      obj = obj.addLinearConstraints([], [], Aeq, beq);

      % at most two consecutive weights can add to 1
      for i = 1:n-1
        Ai = sparse(2, obj.nv);
        bi = [1;-1]+3;

        Ai(1,obj.vars.(lambda).i(i:i+1,1)) = 1;
        Ai(2,obj.vars.(lambda).i(i:i+1,1)) = -1;

        Ai(:,obj.vars.(binvar).i(i,1)) = 3;

        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
    end

    function obj = addBilinearSOS2Constraint(obj,w,x,y,x_ra,y_ra)
      % receives the indexes for variables w, x and y.
      % adds constrait w ~= x*y

      if nargin < 6
        y_ra = [-1,1];
      end

      if nargin < 5
        x_ra = [-1,1];
      end

      M = obj.M+1;

      % adds the binary variables for assignment
      vars_1 = strcat('mc_1_',string(obj.idx));
      vars_2 = strcat('mc_2_',string(obj.idx));

      gamma_ = strcat('gamma_',string(obj.idx));

      alpha_ = strcat('alpha_',string(obj.idx));
      beta_ = strcat('beta_',string(obj.idx));

      obj.idx = obj.idx + 1;
      
      obj = obj.addVariable(vars_1,'B', [M-1, 1], 0, 1);
      obj = obj.addVariable(vars_2,'B', [M-1, 1], 0, 1); 
      
      obj = obj.addVariable(gamma_,'C', [M, M], 0, 1);
      obj = obj.addVariable(alpha_,'C', [M, 1], 0, 1);
      obj = obj.addVariable(beta_,'C', [M, 1], 0, 1);

      % segmentation of x
      x_r = linspace(x_ra(1),x_ra(2),M);
      y_r = linspace(y_ra(1),y_ra(2),M);

      % adds the assignment constraints
      Aeq = sparse(2, obj.nv);
      beq = zeros(2,1);

      Aeq(1,x) = -1;
      Aeq(1,obj.vars.(alpha_).i(:,1)) = x_r(:);
      
      Aeq(2,y) = -1;
      Aeq(2,obj.vars.(beta_).i(:,1)) = y_r(:);
      
      obj = obj.addLinearConstraints([], [], Aeq, beq);

      % adds the approximation constraints
      Aeq = sparse(1, obj.nv);
      beq = 0;

      Aeq(:,w) = -1;

      for i = 1:M
        for j = 1:M
          Aeq(:,obj.vars.(gamma_).i(i,j)) = x_r(i)*y_r(j);
        end
      end

      obj = obj.addLinearConstraints([], [], Aeq, beq);

      % SOS2 constraint in alpha and beta
      Aeq = sparse(5, obj.nv);
      beq = ones(5,1);

      Aeq(1,obj.vars.(vars_1).i(:,1)) = 1;
      Aeq(2,obj.vars.(alpha_).i(:,1)) = 1;

      Aeq(3,obj.vars.(vars_2).i(:,1)) = 1;
      Aeq(4,obj.vars.(beta_).i(:,1)) = 1;

      Aeq(5,obj.vars.(gamma_).i(:,:)) = 1;

      obj = obj.addLinearConstraints([], [], Aeq, beq);

      for i = 1:M-1
        Ai = sparse(2, obj.nv);
        bi = [1;-1]+3;

        Ai(1,obj.vars.(alpha_).i(i:i+1,1)) = 1;
        Ai(2,obj.vars.(alpha_).i(i:i+1,1)) = -1;

        Ai(:,obj.vars.(vars_1).i(i,1)) = 3;

        obj = obj.addLinearConstraints(Ai, bi, [], []);

        Ai = sparse(2, obj.nv);
        bi = [1;-1]+3;

        Ai(1,obj.vars.(beta_).i(i:i+1,1)) = 1;
        Ai(2,obj.vars.(beta_).i(i:i+1,1)) = -1;

        Ai(:,obj.vars.(vars_2).i(i,1)) = 3;

        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end

      % composition of gamma
      for i = 1:M
        % sum of columns adds to alphas
        Aeq = sparse(1,obj.nv);
        beq = 0;

        Aeq(1,obj.vars.(gamma_).i(i,:)) = 1;
        Aeq(1,obj.vars.(alpha_).i(i,1)) = -1;
        
        obj = obj.addLinearConstraints([], [], Aeq, beq);

        % sum of rows adds to betas
        Aeq = sparse(1,obj.nv);
        beq = 0;

        Aeq(1,obj.vars.(gamma_).i(:,i)) = 1;
        Aeq(1,obj.vars.(beta_).i(i,1)) = -1;
        
        obj = obj.addLinearConstraints([], [], Aeq, beq);
      end
    end

    function obj = addMcCormickEnvelopeConstraints(obj,w,x,y,x_r)
      if nargin < 5
        x_r = [-1,1];
      end
      % receives the indexes for variables w, x and y.
      % adds constrait w ~= x*y
      M = obj.M;

      % adds the binary variables for assignment
      vars_1 = strcat('mc1_',string(obj.idx));
      vars_2 = strcat('mc2_',string(obj.idx));
      obj.idx = obj.idx + 1;
      obj = obj.addVariable(vars_1, 'B', [M, 1], 0, 1);
      obj = obj.addVariable(vars_2, 'B', [M, 1], 0, 1);

      % segmentation of x
      x_ra = linspace(x_r(1),x_r(2),M+1);
      x_u = x_ra(2:end);
      x_l = x_ra(1:end-1);

      % the equality must be in one of the regions
      Aeq = sparse(1, obj.nv);
      beq = 1;

      Aeq(1,obj.vars.(vars_1).i(:,1)) = 1;
      Aeq(1,obj.vars.(vars_2).i(:,1)) = 1;

      obj = obj.addLinearConstraints([], [], Aeq, beq);

      % large K
      K = 10;

      % adds the envelope constraints for v >= 0
      for i = 1:M
        H1 = [0, x_l(i), -1;... 
              1, x_u(i), -1;... 
             -1, -x_l(i), 1;... 
              0, -x_u(i), 1;... 
              1, 0, 0;... 
             -1, 0, 0];

        Ai = sparse(6, obj.nv);
        bi = [0; x_u(i); -x_l(i); 0; x_u(i); -x_l(i)] + K;

        Ai(:,[x,y,w]) = H1;
        Ai(:,obj.vars.(vars_1).i(i,1)) = K;

        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end

      % adds the envelope constraints for v < 0
      for i = 1:M
        H2 = [0, -x_l(i), -1;... 
             -1, -x_u(i), -1;... 
              1, x_l(i), 1;...
              0, x_u(i), 1;...
             -1, 0, 0;...
              1, 0, 0];
        
        Ai = sparse(6, obj.nv);
        bi = [0; x_u(i); -x_l(i); 0; x_u(i); -x_l(i)] + K;

        Ai(:,[x,y,w]) = H2;
        Ai(:,obj.vars.(vars_2).i(i,1)) = K;

        obj = obj.addLinearConstraints(Ai, bi, [], []);
      end
    end

    function obj = addDynamicsConstraints(obj)
      % constrains the object dynamics to be consistent with contact forces
      % and gravity

      for task = 1:obj.N_task
        % parameters
        m = obj.object{task}.m; I = obj.object{task}.I;
        g = 9.8; traj = obj.object{task}.traj;

        M = 10*m*g;

        % constrains translational dynamics
        for t = 1:obj.N_T
          % applied force is exists when vertical force is negative
          Ai = sparse(3,obj.nv);
          bi = M*ones(3,1);

          Ai(:,obj.vars.f_app_mode.i(1,t,task)) = M;

          Ai(1,obj.vars.f.i(2,:,:,t,task)) = -1;
          Ai(2,obj.vars.f_app.i(1,t,task)) = 1;

          Ai(1,obj.vars.f_app.i(1,t,task)) = 1;
          Ai(2,obj.vars.f.i(2,:,:,t,task)) = -1;

          Ai(3,obj.vars.f.i(2,:,:,t,task)) = 1;

          obj = obj.addLinearConstraints(Ai, bi, [], []);

          % applied force is zero if nothing is being applied
          Ai = sparse(2,obj.nv);
          bi = zeros(2,1);

          Ai(:,obj.vars.f_app_mode.i(1,t,task)) = -M;
          Ai(:,obj.vars.f_app.i(1,t,task)) = [1,-1]';

          obj = obj.addLinearConstraints(Ai, bi, [], []);        

          % vertical force cannot be more than the weight
          for v = 1:obj.object{task}.nv
            Ai = sparse(1,obj.nv);
            bi = m*g;

            Ai(1,obj.vars.f_ext.i(2,v,t,task)) = 1;
            Ai(1,obj.vars.f_app.i(1,t,task)) = 1;

            obj = obj.addLinearConstraints(Ai, bi, [], []);
          end

          Aeq = sparse(2,obj.nv);
          beq = m*traj.ddr(1:2,t) + [0;m*g];

          % adds environment force constraints
          for v = 1:obj.object{task}.nv
            Aeq(:,obj.vars.f_ext.i(:,v,t,task)) = eye(2);
          end

          % adds contact force constraints
          for c = 1:obj.N_c
            for l = 1:obj.N_l
              Aeq(:,obj.vars.f.i(:,c,l,t,task)) = eye(2);
            end
          end

          obj = obj.addLinearConstraints([], [], Aeq, beq);
        end

        % constrains rotational dynamics
        for t = 1:obj.N_T
          Aeq = sparse(1,obj.nv);
          beq = I*traj.ddr(3,t);

          % adds environment torque constraints 
          for v = 1:obj.object{task}.nv          
            rot_mat = [cos(traj.r(3,t)),-sin(traj.r(3,t));sin(traj.r(3,t)),cos(traj.r(3,t))];
            dp = rot_mat*(obj.object{task}.v(:,v));
            Aeq(:,obj.vars.f_ext.i(1,v,t,task)) = -dp(2);
            Aeq(:,obj.vars.f_ext.i(2,v,t,task)) = dp(1);
          end

          % adds contact torque constraints
          for c = 1:obj.N_c
            for l = 1:obj.N_l
              Aeq(:,obj.vars.tau_aux.i(1,c,l,t,task)) = 1;
              Aeq(:,obj.vars.tau_aux.i(2,c,l,t,task)) = -1;

              Aeq(:,obj.vars.f.i(2,c,l,t,task)) = -traj.r(1,t);
              Aeq(:,obj.vars.f.i(1,c,l,t,task)) = traj.r(2,t);
            end
          end

          % Aeq(:,obj.vars.ddth_aux.i(1,t)) = 1;

          obj = obj.addLinearConstraints([], [], Aeq, beq);

          % computes bilinear terms
          for c = 1:obj.N_c
            for l = 1:obj.N_l
              w1 = obj.vars.tau_aux.i(1,c,l,t,task);
              w2 = obj.vars.tau_aux.i(2,c,l,t,task);

              x1 = obj.vars.p.i(1,c,l,t,task);
              x2 = obj.vars.p.i(2,c,l,t,task);

              y1 = obj.vars.f.i(2,c,l,t,task);
              y2 = obj.vars.f.i(1,c,l,t,task); 

              if obj.McCormick
                obj = obj.addMcCormickEnvelopeConstraints(w1,x1,y1,[-.1,.1]);
                obj = obj.addMcCormickEnvelopeConstraints(w2,x2,y2,[-.2,.2]);
              else
                obj = obj.addBilinearSOS2Constraint(w1,x1,y1,[-0.1,0.1],[-0.06,0.06]);
                obj = obj.addBilinearSOS2Constraint(w2,x2,y2,[0,0.2],[-0.03,0.03]);
              end
            end
          end
        end

        % performs euler integration of the motion of each contact
        for t = 2:obj.N_T
          for c = 1:obj.N_c
            for l = 1:obj.N_l
              % integrates velocity
              Aeq = sparse(2,obj.nv);
              beq = zeros(2,1);

              Aeq(:,obj.vars.p.i(:,c,l,t,task)) = eye(2);
              Aeq(:,obj.vars.p.i(:,c,l,t-1,task)) = -eye(2);
              Aeq(:,obj.vars.dp.i(:,c,l,t,task)) = -obj.dt*eye(2);

              obj = obj.addLinearConstraints([], [], Aeq, beq);

              % integrates acceleration
              Aeq = sparse(2,obj.nv);
              beq = zeros(2,1);

              Aeq(:,obj.vars.dp.i(:,c,l,t,task)) = eye(2);
              Aeq(:,obj.vars.dp.i(:,c,l,t-1,task)) = -eye(2);
              Aeq(:,obj.vars.ddp.i(:,c,l,t,task)) = -obj.dt*eye(2);

              obj = obj.addLinearConstraints([], [], Aeq, beq);
            end
          end
        end

        % % defines initial conditions 
        Aeq = sparse(4,obj.nv);
        beq = zeros(4,1);
        
        Aeq(1:2,obj.vars.ddp.i(:,c,l,1,task)) = eye(2);
        Aeq(3:4,obj.vars.dp.i(:,c,l,1,task)) = eye(2);
        
        obj = obj.addLinearConstraints([], [], Aeq, beq);
      end
    end

    function obj = addContactConstraints(obj, object)
      % Constrains reaction forces to be active only during contact
      % and forces to lie in the respective friction cone

      % big-M
      M = 10;

      for task = 1:obj.N_task
        % relevant variables
        L = object{task}.vars.L.value;

        % activation of forces when in contact
        for t = 1:obj.N_T
          for v = 1:obj.object{task}.nv
            % vertex velocity
            Jac = obj.object{task}.ext_f{v}.jac(:,:,t);
            vel_vert = norm(Jac*obj.object{task}.traj.dr(1:3,t));

            if vel_vert > 0
              % friction cone constraint, sliding left
              Ai = sparse(4,obj.nv);
              bi = zeros(4,1);

              Ai(1:2,obj.vars.f_ext.i(:,v,t,task)) = eye(2);
              Ai(3:4,obj.vars.f_ext.i(:,v,t,task)) = -eye(2);

              Ai(1:2,obj.vars.weight_ext.i(1,v,t,task)) = -obj.object{task}.ext_f{v}.fc1(:,t);
              Ai(3:4,obj.vars.weight_ext.i(1,v,t,task)) = obj.object{task}.ext_f{v}.fc1(:,t);

              obj = obj.addLinearConstraints(Ai, bi, [], []);
            end

            % friction cone constraint, sticking
            if vel_vert == 0
              Ai = sparse(4,obj.nv);
              bi = zeros(4,1);

              Ai(1:2,obj.vars.f_ext.i(:,v,t,task)) = eye(2);
              Ai(1:2,obj.vars.weight_ext.i(1,v,t,task)) = -obj.object{task}.ext_f{v}.fc1(:,t);
              Ai(1:2,obj.vars.weight_ext.i(2,v,t,task)) = -obj.object{task}.ext_f{v}.fc2(:,t);

              Ai(3:4,obj.vars.f_ext.i(:,v,t,task)) = -eye(2);
              Ai(3:4,obj.vars.weight_ext.i(1,v,t,task)) = obj.object{task}.ext_f{v}.fc1(:,t);
              Ai(3:4,obj.vars.weight_ext.i(2,v,t,task)) = obj.object{task}.ext_f{v}.fc2(:,t);

              obj = obj.addLinearConstraints(Ai, bi, [], []);
            end 

            % friction cone constraint, sliding right
            if vel_vert < 0
              Ai = sparse(4,obj.nv);
              bi = zeros(4,1);

              Ai(1:2,obj.vars.f_ext.i(:,v,t,task)) = eye(2);
              Ai(1:2,obj.vars.weight_ext.i(2,v,t,task)) = -obj.object{task}.ext_f{v}.fc2(:,t);

              Ai(3:4,obj.vars.f_ext.i(:,v,t,task)) = -eye(2);
              Ai(3:4,obj.vars.weight_ext.i(2,v,t,task)) = obj.object{task}.ext_f{v}.fc2(:,t);

              obj = obj.addLinearConstraints(Ai, bi, [], []);
            end
          end

          for c = 1:obj.N_c
            for l = 1:obj.N_l
              % complementarity constraint
              Aeq = sparse(1,obj.nv);
              beq = ones(1,1);

              Aeq(1,obj.vars.lambda.i(1,c,l,t,task)) = 1;
              Aeq(1,obj.vars.lambda.i(2,c,l,t,task)) = 1;

              obj = obj.addLinearConstraints([], [], Aeq, beq);

              % forces are zero unless in contact with a facet
              if sum(L(:,c,l,t)) == 0
                Aeq = sparse(2,obj.nv);
                beq = zeros(2,1);

                Aeq(1:2,obj.vars.f.i(:,c,l,t,task)) = eye(2);

                % obj = obj.addLinearConstraints([], [], Aeq, beq);
              end

              for f = 1:obj.N_f
                if L(f,c,l,t) == 1
                  % facet assignment constraint
                  Aeq = sparse(4,obj.nv);
                  beq = zeros(4,1);

                  Aeq(1:2,obj.vars.p.i(:,c,l,t,task)) = eye(2);
                  Aeq(1:2,obj.vars.lambda.i(1,c,l,t,task)) = -obj.object{task}.lines{f}.v1(:,t);
                  Aeq(1:2,obj.vars.lambda.i(2,c,l,t,task)) = -obj.object{task}.lines{f}.v2(:,t);

                  obj = obj.addLinearConstraints([], [], Aeq, beq);

                  % friction cone constraint
                  Aeq = sparse(2,obj.nv);
                  beq = zeros(2,1);

                  Aeq(1:2,obj.vars.f.i(:,c,l,t,task)) = eye(2);
                  Aeq(1:2,obj.vars.weight.i(1,c,l,t,task)) = -obj.object{task}.lines{f}.fc1(:,t);
                  Aeq(1:2,obj.vars.weight.i(2,c,l,t,task)) = -obj.object{task}.lines{f}.fc2(:,t);

                  obj = obj.addLinearConstraints([], [], Aeq, beq);
                end
              end
            end 
          end 
        end
      end
    end

    function obj = addCostFunction(obj)
      % minimizes the following metrics:
      % 1. Acceleration of the contacts
      % 2. Mode switches
      % TODO: maximize stability of grasp
      for task = 1:obj.N_task
        for t = 1:obj.N_T
          for v = 1:obj.object{task}.nv 
            Qi = sparse(obj.nv,obj.nv);
            Qi(obj.vars.f_ext.i(:,v,t,task),obj.vars.f_ext.i(:,v,t,task)) = eye(2);
            % obj = obj.addCost(Qi,[],[]);
          end
          for c = 1:obj.N_c
            for l = 1:obj.N_l
              Qi = sparse(obj.nv,obj.nv);
              Qi(obj.vars.f.i(:,c,l,t,task),obj.vars.f.i(:,c,l,t,task)) = 10*eye(2);
              obj = obj.addCost(Qi,[],[]);
            end
          end
        end

        for l = 1:obj.N_l
          for t = 1:obj.N_T
            Qi = sparse(obj.nv,obj.nv);
            Qi(obj.vars.ddr.i(:,l,t,task),obj.vars.ddr.i(:,l,t,task)) = eye(3);
            obj = obj.addCost(Qi,[],[]);
          end
          for s = 1:obj.N_s
            Qi = sparse(obj.nv,obj.nv);
            Qi(obj.vars.dds.i(:,s,l),obj.vars.dds.i(:,s,l)) = eye(2);
            obj = obj.addCost(Qi,[],[]);
          end
        end
      end
    end

    function obj = addRigidShapeConstraints(obj)
      % knots smoothness
      for l = 1:obj.N_l
        % first knot is at the origin
        Aeq = sparse(2,obj.nv);
        beq = zeros(2,1);
      
        Aeq(:,obj.vars.shape_knots_base.i(:,1,l)) = eye(2);
        
        obj = obj.addLinearConstraints([],[],Aeq,beq);

        % finite differences for curvature
        for s = 2:obj.N_s
          Aeq = sparse(2,obj.nv);
          beq = zeros(2,1);

          Aeq(:,obj.vars.shape_knots_base.i(:,s-1,l)) = eye(2);
          Aeq(:,obj.vars.shape_knots_base.i(:,s+1,l)) = eye(2);
          Aeq(:,obj.vars.shape_knots_base.i(:,s,l)) = -2*eye(2);
          Aeq(:,obj.vars.dds.i(:,s,l)) = -eye(2)*obj.dt^2;

          obj = obj.addLinearConstraints([],[],Aeq,beq);
        end
      end

      for task = 1:obj.N_task
        % SOS2 constraint on sin and cos functions
        for t = 1:obj.N_T
          for l = 1:obj.N_l

            % finite differences for acceleration/effort
            if t > 1 && t < obj.N_T
              Aeq = sparse(3,obj.nv);
              beq = zeros(3,1);

              Aeq(:,obj.vars.r.i(:,l,t-1,task)) = eye(3);
              Aeq(:,obj.vars.r.i(:,l,t+1,task)) = eye(3);
              Aeq(:,obj.vars.r.i(:,l,t,task)) = -2*eye(3);
              Aeq(:,obj.vars.ddr.i(:,l,t,task)) = -eye(3)*obj.dt^2;

              obj = obj.addLinearConstraints([],[],Aeq,beq);
            end

            x = obj.vars.r.i(3,l,t,task);
            
            y1 = obj.vars.sin_th.i(1,l,t,task);
            y2 = obj.vars.cos_th.i(1,l,t,task);

            range_x = linspace(-pi/4,-pi/4,6);

            obj = obj.addSOS2Constraint(x,y1,range_x,sin(range_x));
            obj = obj.addSOS2Constraint(x,y2,range_x,cos(range_x));
          end
        end

        % performs product
        for t = 1:obj.N_T
          for l = 1:obj.N_l
            for k = 1:obj.N_s+1
              % cos*r product
              w = obj.vars.cos_knot.i(1,k,l,t,task);
              x = obj.vars.cos_th.i(1,l,t,task);
              y = obj.vars.shape_knots_base.i(1,k,l);

              if obj.McCormick2
                obj = obj.addMcCormickEnvelopeConstraints(w,x,y,[-1,1]);
              else
                obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[-0.1,0.1]);
              end

              w = obj.vars.cos_knot.i(2,k,l,t,task);
              x = obj.vars.cos_th.i(1,l,t,task);
              y = obj.vars.shape_knots_base.i(2,k,l);

              if obj.McCormick2
                obj = obj.addMcCormickEnvelopeConstraints(w,x,y,[-1,1]);
              else
                obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[-0.1,0.1]);
              end

              % sin*r product
              w = obj.vars.sin_knot.i(1,k,l,t,task);
              x = obj.vars.sin_th.i(1,l,t,task);
              y = obj.vars.shape_knots_base.i(1,k,l);

              if obj.McCormick2
                obj = obj.addMcCormickEnvelopeConstraints(w,x,y,[-1,1]);
              else
                obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[-0.1,0.1]);
              end

              w = obj.vars.sin_knot.i(2,k,l,t,task);
              x = obj.vars.sin_th.i(1,l,t,task);
              y = obj.vars.shape_knots_base.i(2,k,l);          

              if obj.McCormick2
                obj = obj.addMcCormickEnvelopeConstraints(w,x,y,[-1,1]);
              else
                obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[-0.1,0.1]);
              end   
            end
          end
        end

        % position of knot-points through time
        for t = 1:obj.N_T
          for l = 1:obj.N_l
            for k = 1:obj.N_s+1
              if k > 1
                % rotation + translation  at knots != 1
                Aeq = sparse(2,obj.nv);
                beq = zeros(2,1);

                Aeq(1,obj.vars.shape_knots.i(1,k,l,t,task)) = -1;
                Aeq(2,obj.vars.shape_knots.i(2,k,l,t,task)) = -1;

                Aeq(1,obj.vars.cos_knot.i(1,k,l,t,task)) = 1;
                Aeq(1,obj.vars.sin_knot.i(2,k,l,t,task)) = -1;

                Aeq(2,obj.vars.sin_knot.i(1,k,l,t,task)) = 1;
                Aeq(2,obj.vars.cos_knot.i(2,k,l,t,task)) = 1;

                Aeq(:,obj.vars.r.i(1:2,l,t)) = eye(2);

                obj = obj.addLinearConstraints([],[],Aeq,beq);
              else
                % no rotation at first knot
                Aeq = sparse(2,obj.nv);
                beq = zeros(2,1);

                Aeq(:,obj.vars.shape_knots.i(:,k,l,t,task)) = -eye(2);
                Aeq(:,obj.vars.r.i(1:2,l,t,task)) = eye(2);

                obj = obj.addLinearConstraints([],[],Aeq,beq);
              end
            end
          end
        end
      end
    end

    function obj = addAssignmentConstraints(obj,object)
      % assigns contacts to some vertex on the shape

      % big-K
      K = 10;

      for task = 1:obj.N_task
        % assigns contact to a segment
        L = object{task}.vars.L.value;
        for t = 1:obj.N_T
          for l = 1:obj.N_l
            for c = 1:obj.N_c
              % each contact can only lie in one segment
              Ai = sparse(obj.N_f,obj.nv);
              bi = zeros(obj.N_f,1);
              for f = 1:obj.N_f
                Ai(f,obj.vars.contact_assignment.i(:,c,l,task)) = -1;
                bi(f,1) = bi(f,1) - L(f,c,l,t);
              end
              obj = obj.addLinearConstraints(Ai,bi,[],[]);
              
              % contact assignment
              for k = 1:obj.N_s+1
                Ai = sparse(4,obj.nv);
                bi = K*ones(4,1);

                Ai(:,obj.vars.contact_assignment.i(k,c,l,task)) = K;

                Ai(1:2,obj.vars.shape_knots.i(:,k,l,t,task)) = eye(2);
                Ai(3:4,obj.vars.shape_knots.i(:,k,l,t,task)) = -eye(2);

                Ai(1:2,obj.vars.p.i(:,c,l,t,task)) = -eye(2);
                Ai(3:4,obj.vars.p.i(:,c,l,t,task)) = eye(2);

                obj = obj.addLinearConstraints(Ai,bi,[],[]);
              end
            end
          end
        end
      end
    end

    function obj = addNonPenetrationConstraints(obj)
      % Constrains each implicit contact point to lie in a convex region of 
      % space covering the the complement of the object.

      % region assignment constraints
      K = 10;
      for task = 1:obj.N_task
        regions = obj.object{task}.regions;
        for t = 1:obj.N_T
          for l = 1:obj.N_l
            for s = 1:obj.N_s
              % non-floor penetration
              Ai = sparse(1,obj.nv);
              bi = -0.0001;
              
              Ai(1,obj.vars.shape_knots.i(2,s,l,t,task)) = -1;
              
              obj = obj.addLinearConstraints(Ai, bi,[], []);

              % assignment constraints
              for r = 1:obj.N_r
                % loads the matrices
                A = regions{r}.A(:,:,t);
                b = regions{r}.b(:,t);            

                % knot lies in a region
                Ai = sparse(size(b,1),obj.nv);
                bi = b + K;
                
                Ai(:,obj.vars.shape_knots.i(:,s,l,t,task)) = A;
                Ai(:,obj.vars.R.i(r,s,l,t,task)) = K;
                
                obj = obj.addLinearConstraints(Ai, bi,[], []);

                if t < obj.N_T
                  Ai = sparse(size(b,1),obj.nv);
                  bi = b + K;
                  
                  Ai(:,obj.vars.shape_knots.i(:,s,l,t+1,task)) = A;
                  Ai(:,obj.vars.R.i(r,s,l,t,task)) = K;
                  
                  % obj = obj.addLinearConstraints(Ai, bi,[], []);
                end

                % subsequent knots share a common region
                Ai = sparse(size(b,1),obj.nv);
                bi = b + K;
                
                Ai(:,obj.vars.shape_knots.i(:,s+1,l,t,task)) = A;
                Ai(:,obj.vars.R.i(r,s,l,t,task)) = K;
                
                obj = obj.addLinearConstraints(Ai, bi,[], []);
              end

              % Each knot point must be in a region
              Aeq = sparse(1,obj.nv);
              beq = 1;
              
              Aeq(1,obj.vars.R.i(:,s,l,t,task)) = 1;

              obj = obj.addLinearConstraints([], [], Aeq, beq);
            end
          end
        end
      end
    end

    % end of methods
  end
end