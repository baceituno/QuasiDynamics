function animation_shape(object, vid_on)

	% record video?
	if nargin < 2; vid_on = false; end;

	% Animates the execution of the contact optimization plan
	drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'linewidth',1,'color','r')   
	drawArrow2 = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'linewidth',1,'color','b')    
	drawArrow3 = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'linewidth',1,'color','g')    

	% reads all relevant parameters of the problem
	r = object.traj.r(1:2,:);
	dr = object.traj.dr(1:2,:);
	th = object.traj.r(3,:);
	N_T = size(r,2);
	verts = object.v;

	% draws the key frames
	h0 = figure(210)
	clf(h0);
	hold on;

	% draws the floor
	for i = 1:length(object.env)
		x = object.env{i}.x;
		y = object.env{i}.y;
		pgon = polyshape(x,y);
		plot(pgon,'EdgeColor','black','FaceColor','blue')
		hold on;
	end

	for t = 1:N_T
		% transformation matrix
		rot = [cos(th(t)),-sin(th(t));sin(th(t)),cos(th(t))];
		tran = r(:,t);	

		% draws the regions
		v = -0.2:0.002:0.2;  % plotting range from -5 to 5
		[x, y] = meshgrid(v);  % Get 2-D mesh for x and y based on r
		[x, y] = meshgrid(v);  % get 2-D mesh for x and y
		cond = ones(length(v)); % Initialize
		for re = 1:length(object.regions)
			A = object.regions{re}.A;
			b = object.regions{re}.b;
			% condition = (A(1,1,t)*x + A(1,2,t)*y < b(t));
			% cond(condition) = 0;
		end
		% surf(x, y, cond)
		% view(0,90)
		hold on
		% pause();		

		% draws the polygon
		x = [];
		y = [];
		for v = 1:object.nv
			new_vert = tran + rot*verts(:,v);

			x = [x, new_vert(1)];
			y = [y, new_vert(2)];

			fext_1 = zeros(1,N_T);
			fext_2 = zeros(1,N_T);
		end

		pgon = polyshape(x,y);
		plot(pgon,'FaceAlpha',0.5*t/N_T,'FaceColor','white','EdgeColor','black')
		hold on;

		xlim([-0.2,0.2]);
		ylim([-0.2,0.2]);
	end

	set(gca,'color',[1 1 1])

	pause()

	time = linspace(0,1,N_T);
	t1 = linspace(0,1,10*N_T);

	r = [interp1(time,r(1,:),t1); interp1(time,r(2,:),t1)];
	dr = [interp1(time,dr(1,:),t1); interp1(time,dr(2,:),t1)];
	th = interp1(time,th,t1);

	if vid_on
		name = input('movie file name: ','s')

		writerObj = VideoWriter(name);
		writerObj.FrameRate = 10;
		open(writerObj);
	end

	% does the animation
	h = figure(420)
	clf(h);
	hold on;
	for t = 1:10*N_T
		% figure(420)
		clf(h)
		% figure(420)
		% transformation matrix
		rot = [cos(th(t)),-sin(th(t));sin(th(t)),cos(th(t))];
		tran = r(:,t);

		% draws the regions
		v = -0.2:0.002:0.2;  % plotting range from -5 to 5
		[x, y] = meshgrid(v);  % Get 2-D mesh for x and y based on r

		[x, y] = meshgrid(v);  % get 2-D mesh for x and y
		cond = ones(length(v)); % Initialize
		for re = 1:length(object.regions)
			A = object.regions{re}.A;
			b = object.regions{re}.b;
			% condition = (A(1,1,t)*x + A(1,2,t)*y < b(t));
			% cond(condition) = 0;
		end
		% surf(x, y, cond)
		% view(0,90)
		hold on
		% pause();		

		% draws the polygon
		x = [];
		y = [];
		for v = 1:object.nv
			new_vert = tran + rot*verts(:,v);

			x = [x, new_vert(1)];
			y = [y, new_vert(2)];

			hold on;
		end

		pgon = polyshape(x,y);
		plot(pgon)
		hold on;

		% draws the floor
		for i = 1:length(object.env)
			x = object.env{i}.x;
			y = object.env{i}.y;
			pgon = polyshape(x,y);
			plot(pgon)
			hold on;
		end

		xlim([-0.2,0.2]);
		ylim([-0.2,0.2]);

		hold on;
		pause(0.001);
		if vid_on
			frame = getframe(gcf);
			writeVideo(writerObj, frame);
		end
	end
	if vid_on; close(writerObj); end;

	close all;
end