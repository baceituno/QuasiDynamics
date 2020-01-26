classdef MixedIntegerShapeDesignProblem < Quad_MixedIntegerConvexProgram
% Developed by Bernardo Aceituno-C (MIT MCube Lab)
  properties
    contact_plan
    object
    N_c
    N_l
    N_T
    N_r
    N_s
    M = 6
    dt = 0.1
    idx = 0
    idx_sos2 = 0
  end

  methods

    function obj = MixedIntegerShapeDesignProblem(contact_plan, N_s)
      % Constructs the optimization problem and declares the variables for each contact
      % @param contact_plan:  A @Quad_MixedIntegerConvexProgram with the contact locations
      %                       required to achieve the motion of the object

      assert(nargin >= 1)

      if nargin < 2; N_s = contact_plan.N_c; end;

      % parses the inputs
      obj.object = contact_plan.object;
      obj.N_T = contact_plan.N_T;
      obj.N_l = contact_plan.N_l;
      obj.N_c = contact_plan.N_c;
      obj.N_r = contact_plan.N_r;
      obj.N_s = N_s;
      obj.contact_plan = contact_plan;

      % transformation over time
      obj = obj.addVariable('r', 'C', [3, obj.N_l, obj.N_T], -inf, inf);
      obj = obj.addVariable('ddr', 'C', [3, obj.N_l, obj.N_T], -inf, inf);

      obj = obj.addVariable('sin_th', 'C', [1, obj.N_l, obj.N_T], -1, 1);
      obj = obj.addVariable('cos_th', 'C', [1, obj.N_l, obj.N_T], -1, 1);

      % shape description
      obj = obj.addVariable('shape_knots_base', 'C', [2, obj.N_s+1, obj.N_l], -inf, inf);
      obj = obj.addVariable('dds', 'C', [2, obj.N_s+1, obj.N_l], -inf, inf);

      % gripper through time
      obj = obj.addVariable('shape_knots', 'C', [2, obj.N_s+1, obj.N_l, obj.N_T], -inf, inf);

      % contact points
      obj = obj.addVariable('contact_point', 'C', [2, obj.N_c, obj.N_l, obj.N_T], -inf, inf);
      obj = obj.addVariable('contact_assignment', 'B', [obj.N_s+1, obj.N_c, obj.N_l, obj.N_T], 0, 1);

      % non-penetration
      obj = obj.addVariable('R', 'B', [obj.N_r, obj.N_s, obj.N_l, obj.N_T], 0, 1);
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

      % SOS2 constraint on sin and cos functions
      for t = 1:obj.N_T
        for l = 1:obj.N_l

          % finite differences for acceleration/effort
          if t > 1 && t < obj.N_T
            Aeq = sparse(3,obj.nv);
            beq = zeros(3,1);

            Aeq(:,obj.vars.r.i(:,l,t-1)) = eye(3);
            Aeq(:,obj.vars.r.i(:,l,t+1)) = eye(3);
            Aeq(:,obj.vars.r.i(:,l,t)) = -2*eye(3);
            Aeq(:,obj.vars.ddr.i(:,l,t)) = -eye(3)*obj.dt^2;

            obj = obj.addLinearConstraints([],[],Aeq,beq);
          end

          x = obj.vars.r.i(3,l,t);
          
          y1 = obj.vars.sin_th.i(1,l,t);
          y2 = obj.vars.cos_th.i(1,l,t);

          range_x = linspace(-pi/2,-pi/2,3);

          obj = obj.addSOS2Constraint(x,y1,range_x,sin(range_x));
          obj = obj.addSOS2Constraint(x,y2,range_x,cos(range_x));
        end
      end

      % slack variables knot rotations
      obj = obj.addVariable('sin_knot', 'C', [2, obj.N_s+1, obj.N_l, obj.N_T], -inf, inf);
      obj = obj.addVariable('cos_knot', 'C', [2, obj.N_s+1, obj.N_l, obj.N_T], -inf, inf);

      % performs product
      for t = 1:obj.N_T
        for l = 1:obj.N_l
          for k = 1:obj.N_s+1
            % cos*r product
            w = obj.vars.cos_knot.i(1,k,l,t);
            x = obj.vars.cos_th.i(1,l,t);
            y = obj.vars.shape_knots_base.i(1,k,l);

            obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[-0.1,0.1]);

            w = obj.vars.cos_knot.i(2,k,l,t);
            x = obj.vars.cos_th.i(1,l,t);
            y = obj.vars.shape_knots_base.i(2,k,l);

            obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[0,0.1]);

            % sin*r product
            w = obj.vars.sin_knot.i(1,k,l,t);
            x = obj.vars.sin_th.i(1,l,t);
            y = obj.vars.shape_knots_base.i(1,k,l);

            obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[-0.1,0.1]);

            w = obj.vars.sin_knot.i(2,k,l,t);
            x = obj.vars.sin_th.i(1,l,t);
            y = obj.vars.shape_knots_base.i(2,k,l);          

            obj = obj.addBilinearSOS2Constraint(w,x,y,[-1,1],[0,0.1]);    
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

              Aeq(1,obj.vars.shape_knots.i(1,k,l,t)) = -1;
              Aeq(2,obj.vars.shape_knots.i(2,k,l,t)) = -1;

              Aeq(1,obj.vars.cos_knot.i(1,k,l,t)) = 1;
              Aeq(1,obj.vars.sin_knot.i(2,k,l,t)) = -1;

              Aeq(2,obj.vars.sin_knot.i(1,k,l,t)) = 1;
              Aeq(2,obj.vars.cos_knot.i(2,k,l,t)) = 1;

              Aeq(:,obj.vars.r.i(1:2,l,t)) = eye(2);

              obj = obj.addLinearConstraints([],[],Aeq,beq);
            else
              % no rotation at first knot
              Aeq = sparse(2,obj.nv);
              beq = zeros(2,1);

              Aeq(:,obj.vars.shape_knots.i(:,k,l,t)) = -eye(2);

              Aeq(:,obj.vars.r.i(1:2,l,t)) = eye(2);

              obj = obj.addLinearConstraints([],[],Aeq,beq);
            end
          end
        end
      end
    end

    function obj = addContactConstraints(obj)
      % big-K
      K = 10;

      % assigns contact to a segment
      for t = 1:obj.N_T
        for l = 1:obj.N_l
          for c = 1:obj.N_c
            % each contact can only lie in one segment
            if sum(obj.contact_plan.vars.L.value(:,c,l,t)) >= 1
              Aeq = sparse(1,obj.nv);
              beq = 1;
              Aeq(1,obj.vars.contact_assignment.i(:,c,l,t)) = 1;
              obj = obj.addLinearConstraints([],[],Aeq,beq);
            end

            % contact assignment
            for k = 1:obj.N_s+1
              Ai = sparse(4,obj.nv);
              bi = K*ones(4,1);

              Ai(:,obj.vars.contact_assignment.i(k,c,l,t)) = K;

              Ai(1:2,obj.vars.shape_knots.i(:,k,l,t)) = eye(2);
              Ai(3:4,obj.vars.shape_knots.i(:,k,l,t)) = -eye(2);

              bi(1:2) = bi(1:2) + obj.contact_plan.vars.p.value(:,c,l,t);
              bi(3:4) = bi(3:4) - obj.contact_plan.vars.p.value(:,c,l,t);

              obj = obj.addLinearConstraints(Ai,bi,[],[]);

              % a single knot can satisfy the contact point
              if t < obj.N_T
                Ai = sparse(1,obj.nv);
                bi = K;
                Ai(1,obj.vars.contact_assignment.i(k,c,l,t)) = K;
                Ai(1,obj.vars.contact_assignment.i([1:k-1,k+1:end],c,l,t+1)) = 1;

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
      regions = obj.object.regions;
      K = 1;

      for t = 1:obj.N_T
        for l = 1:obj.N_l
          for s = 1:obj.N_s
            % non-floor penetration
            Ai = sparse(1,obj.nv);
            bi = -0.0001;
            
            Ai(1,obj.vars.shape_knots.i(2,s,l,t)) = -1;
            
            obj = obj.addLinearConstraints(Ai, bi,[], []);

            % assignment constraints
            for r = 1:obj.N_r
              % loads the matrices
              A = regions{r}.A(:,:,t);
              b = regions{r}.b(:,t);            

              % knot lies in a single region
              Ai = sparse(size(b,1),obj.nv);
              bi = b + K;
              
              Ai(:,obj.vars.shape_knots.i(:,s,l,t)) = A;
              Ai(:,obj.vars.R.i(r,s,l,t)) = K;
              
              obj = obj.addLinearConstraints(Ai, bi,[], []);

              % subsequent knots share a common region
              Ai = sparse(size(b,1),obj.nv);
              bi = b + K;
              
              Ai(:,obj.vars.shape_knots.i(:,s+1,l,t)) = A;
              Ai(:,obj.vars.R.i(r,s,l,t)) = K;
              
              obj = obj.addLinearConstraints(Ai, bi,[], []);
            end

            % Each knot point must be in a region
            Aeq = sparse(1,obj.nv);
            beq = 1;
            
            Aeq(1,obj.vars.R.i(:,s,l,t)) = 1;

            obj = obj.addLinearConstraints([], [], Aeq, beq);
          end
        end
      end
    end

    function obj = addCostFunction(obj)
      % minimizes the following metrics:
      % 1. Effort
      % 2. Shape curvature
      for l = 1:obj.N_l
        for t = 1:obj.N_T
          Qi = sparse(obj.nv,obj.nv);
          Qi(obj.vars.ddr.i(:,l,t),obj.vars.ddr.i(:,l,t)) = eye(3);
          obj = obj.addCost(Qi,[],[]);
        end
        for s = 1:obj.N_s
          Qi = sparse(obj.nv,obj.nv);
          Qi(obj.vars.dds.i(:,s,l),obj.vars.dds.i(:,s,l)) = eye(2);
          obj = obj.addCost(Qi,[],[]);
        end
      end
    end

    % end of methods
  end
end