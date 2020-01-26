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
			addpath('./inc');
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

		function out = addBuffer(obj, arm, x, y, z, q0, qx, qy, qz, handpose)
			serv = rossvcclient(strcat('/robot', num2str(arm, '%d'), '_AddBuffer'));
			req = rosmessage(serv);
			req.X = x;
			req.Y = y;
			req.Z = z;
			req.Q0 = q0;
			req.Qx = qx;
			req.Qy = qy;
			req.Qz = qz;
			req.Handpose = handpose;
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
			req.Simultaneous = true;
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

		% Specific motions
		function setHomePlanar(obj, angleZ, handpose)
			obj.clearJointBuffers();
			obj.addJointBuffer(1, 79.99, -99.71, 35.63, -143.85, 90.8, 40.76, -66.45);
			obj.addJointBuffer(2, -79.49, -99.73, 35.87, 143.95, 90.89, -39.62, 66.36);
			% Twice because joint buffers require at least 2 orders (dummy)
			obj.addJointBuffer(1, 79.99, -99.71, 35.63, -143.85, 90.8, 40.76, -66.45);
			obj.addJointBuffer(2, -79.49, -99.73, 35.87, 143.95, 90.89, -39.62, 66.36);
			obj.executeSimJointBuffers();

			pause(5.0);

			obj.clearBuffers();
			get1 = obj.getCartesian(1);
			get2 = obj.getCartesian(2);
			[qwR, qxR, qyR, qzR] = obj.quatFromAngleRespectToZ(1, angleZ);
			[qwL, qxL, qyL, qzL] = obj.quatFromAngleRespectToZ(2, angleZ);
			obj.addBuffer(1, get1.X, get1.Y, get1.Z, qwR, qxR, qyR, qzR, 0);
			obj.addBuffer(2, get2.X, get2.Y, get2.Z, qwL, qxL, qyL, qzL, 0);
			obj.executeSimBuffers(true);
		end

		function setHomePlanarSagittal(obj, angleZ)
			obj.clearJointBuffers();
			obj.addJointBuffer(1, 79.99, -99.71, 35.63, -143.85, 90.8, 40.76, -66.45);
			obj.addJointBuffer(2, -79.49, -99.73, 35.87, 143.95, 90.89, -39.62, 66.36);
			% Twice because joint buffers require at least 2 orders (dummy)
			obj.addJointBuffer(1, 79.99, -99.71, 35.63, -143.85, 90.8, 40.76, -66.45);
			obj.addJointBuffer(2, -79.49, -99.73, 35.87, 143.95, 90.89, -39.62, 66.36);
			obj.executeSimJointBuffers();

			pause(5.0);

			angleZ = angleZ - pi/2;

			obj.clearBuffers();
			get1 = obj.getCartesian(1);
			get2 = obj.getCartesian(2);
			[qwR, qxR, qyR, qzR] = obj.quatFromAngleRespectToZ(1, angleZ);
			[qwL, qxL, qyL, qzL] = obj.quatFromAngleRespectToZ(2, angleZ);
			obj.addBuffer(1, get1.X+10, get1.Y, get1.Z, qwR, qxR, qyR, qzR, 0);
			obj.addBuffer(2, get2.X-10, get2.Y, get2.Z, qwL, qxL, qyL, qzL, 0);
			obj.executeSimBuffers(true);
		end

		function out = setXY(obj, arm, x, y)
			getret = obj.getCartesian(arm);
			out = obj.setCartesian(arm, x, y, getret.Z, getret.Q0, getret.Qx, getret.Qy, getret.Qz);
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
			obj.addBuffer(1, get1.X+dx, get1.Y+dy, get1.Z+dz, get1.Q0, get1.Qx, get1.Qy, get1.Qz, 0.0);
			obj.addBuffer(2, get2.X+dx, get2.Y+dy, get2.Z+dz, get2.Q0, get2.Qx, get2.Qy, get2.Qz, 0.0);
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