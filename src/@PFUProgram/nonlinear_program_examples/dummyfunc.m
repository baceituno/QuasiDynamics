function out = dummyfunc(x,y)
	A = [1,0;0,1];
	b = [1;1];

	val = A*[x;y] - b;


	if val(1) >= 0 || val(2) >= 0
		out = 0;
	else
		out = 1;
	end 
end