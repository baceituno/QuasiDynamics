function animation_mip(object, plan, agd, vid_on)

	% record video?
	if nargin < 4; vid_on = false; end;
	if nargin < 3
		no_shape_mip = true;
	else
		no_shape_mip = false;
	end

	% Animates the execution of the contact optimization plan
	drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'linewidth',1,'color','r')   
	drawArrow2 = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'linewidth',1,'color','b')    
	drawArrow3 = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'linewidth',1,'color','g')    

	% reads all relevant parameters of the problem
	N_l = plan.N_l;
	N_c = plan.N_c;
	N_T = plan.N_T;
	r = object.traj.r(1:2,:);
	dr = object.traj.dr(1:2,:);
	th = object.traj.r(3,:);
	verts = object.v;

	% draws the key frames
	h0 = figure(210)
	clf(h0);
	hold on;
	for t = 1:N_T
		subplot(N_T,1,t)

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

			fext_1(1,:) = plan.vars.f_ext.value(1,v,:,1);
			fext_2(1,:) = plan.vars.f_ext.value(2,v,:,1);

			x_ = [new_vert(1), new_vert(1) + fext_1(t)]; 
			y_ = [new_vert(2), new_vert(2) + fext_2(t)];
			% drawArrow(x_,y_);
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

		for l = 1:N_l
			for c = 1:N_c

				p1 = zeros(1,N_T); p2 = zeros(1,N_T);
				f1 = zeros(1,N_T); f2 = zeros(1,N_T);

				p1(1,:) = plan.vars.p.value(1,c,l,:,1);
				p2(1,:) = plan.vars.p.value(2,c,l,:,1);

				f1(1,:) = plan.vars.f.value(1,c,l,:,1);
				f2(1,:) = plan.vars.f.value(2,c,l,:,1);

				% viscircles([p1(1,t),p2(1,t)],0.002);
				% hold on;

				x = [p1(t), p1(t) + f1(t)/2]; 
				y = [p2(t), p2(t) + f2(t)/2];
				
				drawArrow2(x,y);
				hold on;

				x = [r(1,t), r(1,t) + 2*dr(1,t)]; 
				y = [r(2,t), r(2,t) + 2*dr(2,t)];
				% drawArrow3(x,y);
				hold on;
			end
		end

		if no_shape_mip == false
			for l = 1:agd.N_l
				gripper = [];
				for s = 1:agd.N_s+1
					pt = agd.vars.shape_knots_base.value(:,s,l);
					th_e = agd.vars.r.value(3,l,t,1);
					rot = [cos(th_e),-sin(th_e);sin(th_e),cos(th_e)];
					pt_rot = agd.vars.r.value(1:2,l,t,1) + rot*pt;
					gripper = [gripper, pt_rot];
				end

				% scatter(gripper(1,:),gripper(2,:),'r');
				% hold on;

				gripper = [];
				for s = 1:agd.N_s+1
					pt_rot = agd.vars.shape_knots.value(:,s,l,t,1);
					gripper = [gripper, pt_rot];
				end

				plot(gripper(1,:),gripper(2,:),'r.-','LineWidth',3);
				hold on;
			end
		end

		xlim([-0.2,0.2]);
		ylim([-0.2,0.2]);
	end

	% pause()

	% time = linspace(0,1,N_T);
	% t1 = linspace(0,1,10*N_T);

	% r = [interp1(time,r(1,:),t1); interp1(time,r(2,:),t1)];
	% dr = [interp1(time,dr(1,:),t1); interp1(time,dr(2,:),t1)];
	% th = interp1(time,th,t1);

	% if vid_on
	% 	name = input('movie file name: ','s')

	% 	writerObj = VideoWriter(name);
	% 	writerObj.FrameRate = 10;
	% 	open(writerObj);
	% end

	% % does the animation
	% h = figure(420)
	% clf(h);
	% hold on;
	% for t = 1:10*N_T
	% 	% figure(420)
	% 	clf(h)
	% 	% figure(420)
	% 	% transformation matrix
	% 	rot = [cos(th(t)),-sin(th(t));sin(th(t)),cos(th(t))];
	% 	tran = r(:,t);

	% 	% draws the regions
	% 	v = -0.2:0.002:0.2;  % plotting range from -5 to 5
	% 	[x, y] = meshgrid(v);  % Get 2-D mesh for x and y based on r

	% 	[x, y] = meshgrid(v);  % get 2-D mesh for x and y
	% 	cond = ones(length(v)); % Initialize
	% 	for re = 1:length(object.regions)
	% 		A = object.regions{re}.A;
	% 		b = object.regions{re}.b;
	% 		% condition = (A(1,1,t)*x + A(1,2,t)*y < b(t));
	% 		% cond(condition) = 0;
	% 	end
	% 	% surf(x, y, cond)
	% 	% view(0,90)
	% 	hold on
	% 	% pause();		

	% 	% draws the polygon
	% 	x = [];
	% 	y = [];
	% 	for v = 1:object.nv
	% 		new_vert = tran + rot*verts(:,v);

	% 		x = [x, new_vert(1)];
	% 		y = [y, new_vert(2)];

	% 		fext_1 = zeros(1,N_T);
	% 		fext_2 = zeros(1,N_T);

	% 		fext_1(1,:) = plan.vars.f_ext.value(1,v,:);
	% 		fext_2(1,:) = plan.vars.f_ext.value(2,v,:);

	% 		fext_1 = interp1(time,fext_1(1,:),t1);
	% 		fext_2 = interp1(time,fext_2(1,:),t1);

	% 		x_ = [new_vert(1), new_vert(1) + fext_1(t)]; 
	% 		y_ = [new_vert(2), new_vert(2) + fext_2(t)];
	% 		drawArrow(x_,y_);
	% 		hold on;
	% 	end

	% 	pgon = polyshape(x,y);
	% 	plot(pgon)
	% 	hold on;

	% 	% draws the floor
	% 	for i = 1:length(object.env)
	% 		x = object.env{i}.x;
	% 		y = object.env{i}.y;
	% 		pgon = polyshape(x,y);
	% 		plot(pgon)
	% 		hold on;
	% 	end

	% 	for l = 1:N_l
	% 		for c = 1:N_c

	% 			p1 = zeros(1,N_T); p2 = zeros(1,N_T);
	% 			f1 = zeros(1,N_T); f2 = zeros(1,N_T);
	% 			p1(1,:) = plan.vars.p.value(1,c,l,:);
	% 			p2(1,:) = plan.vars.p.value(2,c,l,:);

	% 			f1(1,:) = plan.vars.f.value(1,c,l,:);
	% 			f2(1,:) = plan.vars.f.value(2,c,l,:);

	% 			p1 = interp1(time,p1,t1);
	% 			p2 = interp1(time,p2,t1);

	% 			viscircles([p1(1,t),p2(1,t)],0.002);
	% 			hold on;

	% 			f1 = interp1(time,f1,t1);
	% 			f2 = interp1(time,f2,t1);

	% 			x = [p1(t), p1(t) + f1(t)]; 
	% 			y = [p2(t), p2(t) + f2(t)];
				
	% 			drawArrow2(x,y);
	% 			hold on;

	% o			x = [r(1,t), r(1,t) + 2*dr(1,t)]; 
	% 			y = [r(2,t), r(2,t) + 2*dr(2,t)];
	% 			drawArrow3(x,y);
	% 			hold on;
	% 		end
	% 	end

	% 	if no_shape == false
	% 		if t >= 1
	% 			t_x = (t - mod(t,10))/10;
	% 			for l = 1:nlp.N_l
	% 				xd = nlp.shape.trans(nlp.idx.trans(1,l,:));
	% 				xp = interp1(time,xd,t1);

	% 				yd = nlp.shape.trans(nlp.idx.trans(2,l,:));
	% 				yp = interp1(time,yd,t1);

	% 				thd = nlp.shape.trans(nlp.idx.trans(3,l,:));
	% 				th_2 = interp1(time,thd,t1);

	% 				x = linspace(0,nlp.sol.value(nlp.vars.len(1,l)),100);
	% 				s = zeros(2,100);
	% 				for o = 1:nlp.poly_order+1
	% 					s_y = nlp.shape.coeff(nlp.idx.coeff(l,o))*basis_func(nlp.basis,x,o,1);
	% 					s = s + [-sin(th_2(t))*s_y;...
	% 							 cos(th_2(t))*s_y];
	% 				end
	% 				s_x = basis_func(nlp.basis,x,2,1);
	% 				s = s + [cos(th_2(t))*s_x;...
	% 						 sin(th_2(t))*s_x];
	% 				plot(xp(t) + s(1,:), yp(t) + s(2,:),'LineWidth',2)
	% 				xlim([-0.1,0.1])
	% 				ylim([-0.1,0.2])
	% 				hold on;
	% 			end
	% 			if mod(t,10) == 0
	% 				xlim([-0.2,0.2]);
	% 				ylim([-0.2,0.2]);
	% 				% pause();
	% 			end
	% 		end
	% 	end
	% 	xlim([-0.2,0.2]);
	% 	ylim([-0.2,0.2]);

	% 	hold on;
	% 	pause(0.001);
	% 	if vid_on
	% 		frame = getframe(gcf);
	% 		writeVideo(writerObj, frame);
	% 	end
	% end
	% if vid_on; close(writerObj); end;
end