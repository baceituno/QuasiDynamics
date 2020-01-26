classdef nlpWrapper < handle
    properties
        nlp;
    end
    methods
        function obj=nlpWrapper(nvars)
            obj.NonlinearProgram(nvars);
        end
        
        function NonlinearProgram(obj,nvars)
            obj.nlp=NonlinearProgram(nvars);
        end
        
        function addConstraint(obj,constraintfun,indices)
            obj.nlp = obj.nlp.addConstraint(constraintfun,indices);
        end
        
        function addCost(obj,costfun,indices)
            obj.nlp = obj.nlp.addCost(costfun,indices);
        end
        
        function addDecisionVariable(obj,numVars)
            obj.nlp = obj.nlp.addDecisionVariable(numVars);
        end
        
        function setCheckGrad(obj,mybool)
            obj.nlp = obj.nlp.setCheckGrad(mybool);
        end
        
        function setSolver(obj,mystr)
            obj.nlp = obj.nlp.setSolver(mystr);
        end
        
        function setConstraintErrTol(obj,myval)
            obj.nlp = obj.nlp.setConstraintErrTol(myval);
        end
        
        function setSolverOptions(obj,input1,input2,input3)
            obj.nlp = obj.nlp.setSolverOptions(input1,input2,input3);
        end
        
        function Yresult=solve(obj,guess)
            Yresult=obj.nlp.solve(guess);
        end
        
        function [g,h,dg,dh] = nonlinearConstraints(obj,x)
            [g,h,dg,dh] = obj.nlp.nonlinearConstraints(x);
        end
            
    end
end