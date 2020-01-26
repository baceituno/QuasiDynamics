classdef PFUProgram < handle
    properties
        nlp;            %nonlinear program object used to solve problem
        
        Nvars;          %total number of decision variables
        Nch;            %number of vertices used to represent hand
        Ncb;            %number of vertices used to represent the object
        Nce;            %number of vertices used to represent the environment

        LPx_b;          %Indices of Px_b, which is the x position of the COM of the object
        LPy_b;          %Indices of Py_b, which is the y position of the COM of the object
        Ltheta_b;       %Indices of theta_b, which is the orientation of the object frame
        
        LVx_b;          %Indices of Vx_b, which is the x velocity of the COM of the object
        LVy_b;          %Indices of Vy_b, which is the y velocity of the COM of the object
        Lomega_b;       %Indices of omega_b, which is the angular velocity of the object frame
        
        LAx_b;          %Indices of Ax_b, which is the x acceleration of the COM of the object
        LAy_b;          %Indices of Ay_b, which is the y acceleration of the COM of the object
        Lalpha_b;       %Indices of alpha_b, which is the angular acceleration of the object frame

        LPx_h;          %Indices of Px_h, which is the x position of the hand frame
        LPy_h;          %Indices of Py_h, which is the y position of the hand frame
        Ltheta_h;       %Indices of theta_h, which is the orientation of the hand frame
        
        LVx_h;          %Indices of Vx_h, which is the x velocity of the hand frame
        LVy_h;          %Indices of Vy_h, which is the y velocity of the hand frame
        Lomega_h;       %Indices of omega_h, which is the angular velocity of the hand frame
        
        LAx_h;          %Indices of Ax_h, which is the x acceleration of the hand frame
        LAy_h;          %Indices of Ay_h, which is the y acceleration of the hand frame
        Lalpha_h;       %Indices of alpha_h, which is the angular acceleration of the hand frame

        LCx_h;          %Indices of Cx_h, which are the x coordinates of the hand contour (in its base position)
        LCy_h;          %Indices of Cy_h, which are the y coordinates of the hand contour (in its base position)

        Cx_b;           %x coordinates of the vertices of the object contour (in its base position, Cx_b=Cy_b=theta_b=0)
        Cy_b;           %y coordinates of the vertices of the object contour (in its base position, Cx_b=Cy_b=theta_b=0)

        LFx_h;          %Indices of x components of force of hand acting on object. This should be a matrix of indices where row corresponds to time step and column corresponds to a point on the hand
        LFy_h;          %Indices of y components of force of hand acting on object. This should be a matrix of indices where row corresponds to time step and column corresponds to a point on the hand

        LFx_e;          %Indices of x components of force of environment acting on object
        LFy_e;          %Indices of y components of force of environment acting on object

        LSlack_h;       %Indices of slack variables corresponding to complementarity constraint for hand contacting object
        LSlack_e;       %Indices of slack variables corresponding to complementarity constraint for environment contacting object

        M_b;            %Mass of the object
        I_b;            %Moment of inertia of the object about its center of mass
    end
    
    methods
        function addBasicIntegrationConstraints(obj)

        end

        function addLinearMomentumConstraint(obj)

        end

        function addAngularMomentumConstraint(obj)

        end

        function addDistanceComplementarityConstraint_ObjectHand(obj)

        end

        function addDistanceComplementarityConstraint_ObjectEnv(obj)

        end

        function mySignedDistanceFunction(obj)

        end
    end
end
