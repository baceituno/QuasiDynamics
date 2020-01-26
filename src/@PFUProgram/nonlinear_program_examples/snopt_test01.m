function res=snopt_test01()
%the input is the number of variables to be optimized
nlp1 = NonlinearProgram(3);

%this checks to see if the jacobian is actually correct
%usually I set to false, since this will take a while
nlp1 = nlp1.setCheckGrad(true);

%set your choice of solver
nlp1 = nlp1.setSolver('snopt');

%%%%%%%%%%%
%vector function represented by cnstr1_userfun
% -inf <=x1^2+4*x2^2<=4 
% -inf <=(x1-2)^2+x2^2<=5
% 0 <= x1 <= inf

%set the constraint bounds
cnstr1 = FunctionHandleConstraint([-inf(2,1);0],[4;5;inf],2,@cnstr1_userfun);

%indices of the 
xind1 = [2;3];

%set it so that cnstr1_userfun(x(xind1)) meets the constraints
nlp1 = nlp1.addConstraint(cnstr1,xind1);

%initial guess
x1 = [1;2;1];

%call the solver with initial guess seed
x=nlp1.solve(x1)

%check to see if the output satisifies constraints
cnstr1_userfun(x(xind1'))

end

%c is the constraint function
%dc is the jacobian of the constraint with respect to the input variables
function [c,dc] = cnstr1_userfun(x)
c = [x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2;x(1)];
dc = [2*x(1) 8*x(2);2*(x(1)-2) 2*x(2);1 0];
end

