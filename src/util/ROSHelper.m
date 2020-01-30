classdef ROSHelper < handle
	properties
		x0
		y0
		z0
		angleZR
		angleZL
		handH
	end
	methods
		% Initializer
		% Suggested values: (400.0, 0.0, 180.0)
		function obj = ROSHelper(x0, y0, z0)
			if nargin < 3
				obj.x0 = 400.0;
				obj.y0 = 0.0;
				obj.z0 = 180.0;
			else
				obj.x0 = x0;
				obj.y0 = y0;
				obj.z0 = z0;
			end
			obj.angleZR = 0;
			obj.angleZL = 0;
			obj.handH = 132;
		end
		
		function setAngleZR(obj, theta)
				obj.angleZR = theta;
		end
		
		function setAngleZL(obj, theta)
				obj.angleZL = theta;
		end

		% Generic robot node services
		function ret = getCartesian(obj, arm)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_GetCartesian'));
			ret = call(serv, rosmessage(serv));
		end

		function out = setCartesian(obj, arm, x, y, z, q0, qx, qy, qz)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_SetCartesian'));
			req = rosmessage(serv);
			req.X = x;
			req.Y = y;
			req.Z = z;
			req.Q0 = q0;
			req.Qx = qx;
			req.Qy = qy;
			req.Qz = qz;
			out = call(serv, req);
		end
				
		function out = setQuaternion(obj, arm, q0, qx, qy, qz)
			get = obj.getCartesian(arm);
			out = obj.setCartesian(arm, get.X, get.Y, get.Z, q0, qx, qy, qz);
		end

		function ret = getJoints(obj, arm)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_GetJoints'));
			ret = call(serv, rosmessage(serv));
		end

		function out = setJoints(obj, arm, j1, j2, j3, j4, j5, j6, j7)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_SetJoints'));
			req = rosmessage(serv);
			req.J1 = j1;
			req.J2 = j2;
			req.J3 = j3;
			req.J4 = j4;
			req.J5 = j5;
			req.J6 = j6;
			req.J7 = j7;
			out = call(serv, req);
		end

		function out = setSimJoints(obj, ik1, ik2)
			obj.clearJointBuffers();
			obj.addJointBuffer(1, ik1.J1, ik1.J2, ik1.J3, ik1.J4, ik1.J5, ik1.J6, ik1.J7);
			obj.addJointBuffer(2, ik2.J1, ik2.J2, ik2.J3, ik2.J4, ik2.J5, ik2.J6, ik2.J7);
			% Twice because joint buffers require at least 2 orders (dummy)
			obj.addJointBuffer(1, ik1.J1, ik1.J2, ik1.J3, ik1.J4, ik1.J5, ik1.J6, ik1.J7);
			obj.addJointBuffer(2, ik2.J1, ik2.J2, ik2.J3, ik2.J4, ik2.J5, ik2.J6, ik2.J7);
			obj.executeSimJointBuffers();
		end


		function out = setSimTraj(obj, ik)
			obj.clearJointBuffers();
			for t = 1:length(ik)
				ik1 = ik{t}.ik1;
				ik2 = ik{t}.ik2;
				obj.addJointBuffer(1, ik1.J1, ik1.J2, ik1.J3, ik1.J4, ik1.J5, ik1.J6, ik1.J7);
				obj.addJointBuffer(2, ik2.J1, ik2.J2, ik2.J3, ik2.J4, ik2.J5, ik2.J6, ik2.J7);
				% Twice because joint buffers require at least 2 orders (dummy)
				% obj.addJointBuffer(1, ik1.J1, ik1.J2, ik1.J3, ik1.J4, ik1.J5, ik1.J6, ik1.J7);
				% obj.addJointBuffer(2, ik2.J1, ik2.J2, ik2.J3, ik2.J4, ik2.J5, ik2.J6, ik2.J7);
			end
			obj.executeSimJointBuffers();
		end

		function out = addBuffer(obj, arm, x, y, z, q0, qx, qy, qz)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_AddBuffer'));
			req = rosmessage(serv);
			req.X = x;
			req.Y = y;
			req.Z = z;
			req.Q0 = q0;
			req.Qx = qx;
			req.Qy = qy;
			req.Qz = qz;
			req.Handpose = 0.0;
			out = call(serv, req);
		end

		function out = clearBuffer(obj, arm)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_ClearBuffer'));
			req = rosmessage(serv);
			out = call(serv, req);
		end

		function clearBuffers(obj)
			obj.clearBuffer(1);
			obj.clearBuffer(2);
		end

		function out = executeSimBuffer(obj, arm, usehandpose)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_ExecuteBuffer'));
			req = rosmessage(serv);
			req.Simultaneous = true;
			req.UseHandPose = usehandpose;
			out = call(serv, req);
		end

		function executeSimBuffers(obj, usehandpose)
			obj.executeSimBuffer(1, usehandpose);
			obj.executeSimBuffer(2, usehandpose);
		end

		function out = addJointBuffer(obj, arm, j1, j2, j3, j4, j5, j6, j7)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_AddJointPosBuffer'));
			req = rosmessage(serv);
			req.J1 = j1;
			req.J2 = j2;
			req.J3 = j3;
			req.J4 = j4;
			req.J5 = j5;
			req.J6 = j6;
			req.J7 = j7;
			out = call(serv, req);
		end

		function out = clearJointBuffer(obj, arm)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_ClearJointPosBuffer'));
			req = rosmessage(serv);
			out = call(serv, req);
		end

		function clearJointBuffers(obj)
			obj.clearJointBuffer(1);
			obj.clearJointBuffer(2);
		end

		function out = executeSimJointBuffer(obj, arm)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_ExecuteJointPosBuffer'));
			req = rosmessage(serv);
			req.Simultaneous = 1;
			out = call(serv, req);
		end

		function executeSimJointBuffers(obj)
			obj.executeSimJointBuffer(1);
			obj.executeSimJointBuffer(2);
		end

		function ret = calibrateHand(obj, arm)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_HandCalibrate'));
			ret = call(serv, rosmessage(serv));
		end

		function ret = getIK(obj, arm, x, y, z, q0, qx, qy, qz, ang)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_GetIK'));
			req.X = x;
			req.Y = y;
			req.Z = z;
			req.Q0 = q0;
			req.Qx = qx;
			req.Qy = qy;
			req.Qz = qz;
			req.ang = ang;
			ret = call(serv, rosmessage(serv));
		end

		function pose = getHandPose(obj, arm)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_HandGetPose'));
			ret = call(serv, rosmessage(serv));
			pose = ret.Pose;
		end

		function out = setHandPose(obj, arm, pose)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_HandMoveTo'));
			req = rosmessage(serv);
			if (pose >= 0 && pose <= 20.5)
				req.HandPose = pose;
				ret = call(serv, req);
			else
				error("Pose must be between 0 and 20.5 mm.");
			end
		end

		% Computations
		function [qw, qx, qy, qz] = quatFromAngleRespectToX(obj, arm, theta)
			q1 = quaternion([0.0, 0.0, 1.0, 0.0]);
			q2 = quaternion([cos(-theta/2), 0.0, 0.0, sin(-theta/2)]);
			[qw, qx, qy, qz] = parts(q1*q2);
		end
		
		function [qw, qx, qy, qz] = quatFromAngleRespectToXandY(obj, arm, theta1, theta2)
			q1 = quaternion([0.0, 0.0, 1.0, 0.0]);
			q2 = quaternion([cos(-theta1/2), 0.0, 0.0, sin(-theta1/2)]);
						q3 = quaternion([cos(-theta2/2), sin(-theta2/2), 0.0, 0.0]);
			[qw, qx, qy, qz] = parts(q1*q2*q3);
		end

		function [qw, qx, qy, qz] = quatFromAngleRespectToZ(obj, arm, theta)
			q1 = quaternion([0.0, 0.0, 1.0, 0.0]);
			if arm == 1
				q2 = quaternion([cos(-theta/2), sin(-theta/2), 0.0, 0.0]);
				obj.angleZR = theta;
			else
				q2 = quaternion([cos(theta/2), sin(theta/2), 0.0, 0.0]);
				obj.angleZL = theta;
			end
			[qw, qx, qy, qz] = parts(q1*q2);
		end

		function setSimDXYZ(obj,x1,y1,z1,x2,y2,z2)
			obj.clearJointBuffers();
			cart1 = obj.getCartesian(1);
			cart2 = obj.getCartesian(2);

			arm_ang = 100;

			ik1 = obj.getIK(cart1.X+x1,cart1.Y+y1,cart1.Z+z1,cart1.Q0,cart1.Qx,cart1.Qy,cart1.Qz,arm_ang)
			ik2 = obj.getIK(cart1.X+x1,cart1.Y+y1,cart1.Z+z1,cart1.Q0,cart1.Qx,cart1.Qy,cart1.Qz,-arm_ang)

			obj.clearJointBuffers();
			obj.addJointBuffer(1, ik1.j1, ik1.j2, ik1.j3, ik1.j4, ik1.j5, ik1.j6, ik1.j7);
			obj.addJointBuffer(2, ik2.j1, ik2.j2, ik2.j3, ik2.j4, ik2.j5, ik2.j6, ik2.j7);
			% Twice because joint buffers require at least 2 orders (dummy)
			obj.addJointBuffer(1, ik1.j1, ik1.j2, ik1.j3, ik1.j4, ik1.j5, ik1.j6, ik1.j7);
			obj.addJointBuffer(2, ik2.j1, ik2.j2, ik2.j3, ik2.j4, ik2.j5, ik2.j6, ik2.j7);
			obj.executeSimJointBuffers();
		end

		% Specific motions
		function setHomePlanar(obj)
			obj.clearJointBuffers();
			obj.addJointBuffer(1, 74.94, -44.11, 22.62, -100.38, 21.99, -124.06, -98.72);
			obj.addJointBuffer(2, -74.62, -44.12, 23, 101.05, 22.22, 124.06, 98.60);
			% Twice because joint buffers require at least 2 orders (dummy)
			obj.addJointBuffer(1, 74.94, -44.11, 22.62, -100.38, 21.99, -124.06, -98.72);
			obj.addJointBuffer(2, -74.62, -44.12, 23, 101.05, 22.22, 124.06, 98.60);
			obj.executeSimJointBuffers();
		end

		function setHomeSagittal(obj)
			obj.clearJointBuffers();
			obj.addJointBuffer(1, 74.94, -44.11, 22.62, -100.38, 21.99, 55.06, -98.72);
			obj.addJointBuffer(2, -74.62, -44.12, 23, 101.05, 22.22, -55.06, 98.60);
			% Twice because joint buffers require at least 2 orders (dummy)
			obj.addJointBuffer(1, 74.94, -44.11, 22.62, -100.38, 21.99, 55.06, -98.72);
			obj.addJointBuffer(2, -74.62, -44.12, 23, 101.05, 22.22, -55.06, 98.60);
			obj.executeSimJointBuffers();
		end

		function out = setXY(obj, arm, x, y)
			getret = obj.getCartesian(arm);
			out = obj.setCartesian(arm, x, y, getret.Z, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
        end

        function out = setXYZ(obj, arm, x, y, z)
			getret = obj.getCartesian(arm);
			out = obj.setCartesian(arm, x, y, z, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
        end
        
        function out = setdXYZ(obj, arm, dx, dy, dz)
			getret = obj.getCartesian(arm);
			out = obj.setCartesian(arm, getret.X + dx, getret.Y + dy, getret.Z + dz, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
		end

		function out = adddXYZ(obj, dx1, dy1, dz1, dx2, dy2, dz2)
			obj.clearBuffers();
			getret = obj.getCartesian(1);
			out = obj.setCartesian(1, getret.X + dx1, getret.Y + dy1, getret.Z + dz1, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
			getret = obj.getCartesian(2);
			out = obj.setCartesian(2, getret.X + dx2, getret.Y + dy2, getret.Z + dz2, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
		end

		function out = setInitialPositionPlanar(obj, x1, y1, x2, y2)
			getret = obj.getCartesian(1);
			out = obj.setCartesian(1, 395  + x1, -67.5 + y1, 225, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
			getret = obj.getCartesian(2);
			out = obj.setCartesian(2, 403 + x2, 67.5 + y2, 218, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
		end

		function out = setInitialPositionSagittal(obj, x1, y1, x2, y2)
			getret = obj.getCartesian(1);
			out = obj.setCartesian(1, 392, -67.5 + x1 - 11.6 + 1.1, 175 + 10.6 + y1, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
			getret = obj.getCartesian(2);
			out = obj.setCartesian(2, 403, 67.5 + x2 + 9.6 - 2.1, 170 + 10.6 + y2, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
		end


		function out = setdTraj(obj, dx1, dy1, dz1, dx2, dy2, dz2)
			getret = obj.getCartesian(1);
			out = obj.setCartesian(1, getret.X + dx1, getret.Y + dy1, getret.Z + dz1, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
			getret = obj.getCartesian(2);
			out = obj.setCartesian(2, getret.X + dx2, getret.Y + dy2, getret.Z + dz2, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
		end

        
		function setArmDeltaXYZ(obj, arm, dx, dy, dz)
			obj.clearBuffers();
			get1 = obj.getCartesian(arm);
			obj.addBuffer(arm, get1.X+dx, get1.Y+dy, get1.Z+dz, get1.Q0, get1.Qx, get1.Qy, get1.Qz, 0.0);
			obj.executeSimBuffers(false);
		end

		function out = setDeltaXYZ(obj, arm, dx, dy, dz)
			getret = obj.getCartesian(arm);
			out = obj.setCartesian(arm, getret.X+dx, getret.Y+dy, getret.Z+dz, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
		end

		function setSimDeltaXYZ(obj, dx, dy, dz)
			obj.clearBuffers();
			get1 = obj.getCartesian(1);
			get2 = obj.getCartesian(2);
			obj.addBuffer(1, get1.X+dx, get1.Y+dy, get1.Z+dz, get1.Q0, get1.Qx, get1.Qy, get1.Qz);
			obj.addBuffer(2, get2.X+dx, get2.Y+dy, get2.Z+dz, get2.Q0, get2.Qx, get2.Qy, get2.Qz);
			obj.executeSimBuffers(false);
		end

		function setSimDifDeltaXYZ(obj, dx1, dy1, dz1, dx2, dy2, dz2, hp1, hp2)
			obj.clearBuffers();
			get1 = obj.getCartesian(1);
			get2 = obj.getCartesian(2);
			obj.addBuffer(1, get1.X+dx1, get1.Y+dy1, get1.Z+dz1, get1.Q0, get1.Qx, get1.Qy, get1.Qz, hp1);
			obj.addBuffer(2, get2.X+dx2, get2.Y+dy2, get2.Z+dz2, get2.Q0, get2.Qx, get2.Qy, get2.Qz, hp2);
			obj.executeSimBuffers(true);
		end

		function setSimHandPose(obj, handpose)
			obj.clearBuffers();
			get1 = obj.getCartesian(1);
			get2 = obj.getCartesian(2);
			obj.addBuffer(1, get1.X, get1.Y, get1.Z, get1.Q0, get1.Qx, get1.Qy, get1.Qz, handpose);
			obj.addBuffer(2, get2.X, get2.Y, get2.Z, get2.Q0, get2.Qx, get2.Qy, get2.Qz, handpose);
			obj.executeSimBuffers(true);
		end


		function addHandsToBufferPlanar(obj, p1, p2, angleX)
			% p1 correspond to right Yumi hand in the plane
			% p2 correspond to left Yumi hand in the plane
			% Assumes p1, p2 in mm centered in (0, 0)
			% For gaps: calibrated hand has a gap of 3 mm and fingers have radius of around 1.25 mm
			obj.handH = 131;
			hR = [obj.x0+p1(1)+obj.handH*sin(obj.angleZR), obj.y0+p1(2)];
			hL = [obj.x0+p2(1)-obj.handH*sin(obj.angleZL), obj.y0+p2(2)];

			% Using previous orientation and height
			getR = obj.getCartesian(1);
			getL = obj.getCartesian(2);
			obj.addBuffer(1, hR(1), hR(2), getR.Z, getR.Q0, getR.Qx, getR.Qy, getR.Qz, 0);
			obj.addBuffer(2, hL(1), hL(2), getL.Z, getL.Q0, getL.Qx, getL.Qy, getL.Qz, 0);
		end


		function addHandsToBufferSagittal(obj, p1, p2, angleX)
			% p1 correspond to right Yumi hand in the plane
			% p2 correspond to left Yumi hand in the plane
			% Assumes p1, p2 in mm centered in (0, 0)
			% For gaps: calibrated hand has a gap of 3 mm and fingers have radius of around 1.25 mm
			obj.handH = 131;
			hR = [obj.x0+p1(1)+obj.handH*sin(obj.angleZR), obj.z0+p1(2)];
			hL = [obj.x0+p2(1)-obj.handH*sin(obj.angleZL), obj.z0+p2(2)];

			% Using previous orientation and height
			getR = obj.getCartesian(1);
			getL = obj.getCartesian(2);
			obj.addBuffer(1, hR(1), getR.Z, hR(2), getR.Q0, getR.Qx, getR.Qy, getR.Qz, 0);
			obj.addBuffer(2, hL(1), getL.y, hL(2), getL.Q0, getL.Qx, getL.Qy, getL.Qz, 0);
		end
	end
end