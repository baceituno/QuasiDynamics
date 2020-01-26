close all; clear all; clc;

M = 6;

x_ra = linspace(-1,1,M+1);
x_u = x_ra(2:end);
x_l = x_ra(1:end-1);

Poly = {};


% w = xy, y >= 0
for i = 1:length(x_l)
	Poly{i} = struct();
	Poly{i}.H = [0, x_l(i), -1; 
				 1, x_u(i), -1;
				 -1, -x_l(i), 1;
				 0, -x_u(i), 1;
				 1, 0, 0;
				 -1, 0, 0]; 

	Poly{i}.b = [0; 
				 x_u(i);
				 -x_l(i);
				 0;
				 x_u(i);
				 -x_l(i)]; 

	R = polytope(Poly{i}.H,Poly{i}.b);
	plot(R,'b')
	hold on;
end

% w = xy, y <= 0
for i = 1:length(x_l)
	Poly{i} = struct();
	Poly{i}.H = [0, -x_l(i), -1; 
				 -1, -x_u(i), -1;
				 1, x_l(i), 1;
				 0, x_u(i), 1;
				 -1, 0, 0;
				 1, 0, 0]; 

	Poly{i}.b = [0; 
				 x_u(i);
				 -x_l(i);
				 0;
				 x_u(i);
				 -x_l(i)]; 

	R = polytope(Poly{i}.H,Poly{i}.b);
	plot(R,'r')
	hold on;
end
