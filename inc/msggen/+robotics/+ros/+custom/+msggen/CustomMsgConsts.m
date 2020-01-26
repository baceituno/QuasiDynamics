classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    properties (Constant)
        robot_comm_robot_ActivateCSS = 'robot_comm/robot_ActivateCSS'
        robot_comm_robot_ActivateCSSRequest = 'robot_comm/robot_ActivateCSSRequest'
        robot_comm_robot_ActivateCSSResponse = 'robot_comm/robot_ActivateCSSResponse'
        robot_comm_robot_ActivateEGM = 'robot_comm/robot_ActivateEGM'
        robot_comm_robot_ActivateEGMRequest = 'robot_comm/robot_ActivateEGMRequest'
        robot_comm_robot_ActivateEGMResponse = 'robot_comm/robot_ActivateEGMResponse'
        robot_comm_robot_AddBuffer = 'robot_comm/robot_AddBuffer'
        robot_comm_robot_AddBufferRequest = 'robot_comm/robot_AddBufferRequest'
        robot_comm_robot_AddBufferResponse = 'robot_comm/robot_AddBufferResponse'
        robot_comm_robot_AddJointPosBuffer = 'robot_comm/robot_AddJointPosBuffer'
        robot_comm_robot_AddJointPosBufferRequest = 'robot_comm/robot_AddJointPosBufferRequest'
        robot_comm_robot_AddJointPosBufferResponse = 'robot_comm/robot_AddJointPosBufferResponse'
        robot_comm_robot_Approach = 'robot_comm/robot_Approach'
        robot_comm_robot_ApproachRequest = 'robot_comm/robot_ApproachRequest'
        robot_comm_robot_ApproachResponse = 'robot_comm/robot_ApproachResponse'
        robot_comm_robot_CartesianLog = 'robot_comm/robot_CartesianLog'
        robot_comm_robot_ClearBuffer = 'robot_comm/robot_ClearBuffer'
        robot_comm_robot_ClearBufferRequest = 'robot_comm/robot_ClearBufferRequest'
        robot_comm_robot_ClearBufferResponse = 'robot_comm/robot_ClearBufferResponse'
        robot_comm_robot_ClearJointPosBuffer = 'robot_comm/robot_ClearJointPosBuffer'
        robot_comm_robot_ClearJointPosBufferRequest = 'robot_comm/robot_ClearJointPosBufferRequest'
        robot_comm_robot_ClearJointPosBufferResponse = 'robot_comm/robot_ClearJointPosBufferResponse'
        robot_comm_robot_DeactivateCSS = 'robot_comm/robot_DeactivateCSS'
        robot_comm_robot_DeactivateCSSRequest = 'robot_comm/robot_DeactivateCSSRequest'
        robot_comm_robot_DeactivateCSSResponse = 'robot_comm/robot_DeactivateCSSResponse'
        robot_comm_robot_ExecuteBuffer = 'robot_comm/robot_ExecuteBuffer'
        robot_comm_robot_ExecuteBufferRequest = 'robot_comm/robot_ExecuteBufferRequest'
        robot_comm_robot_ExecuteBufferResponse = 'robot_comm/robot_ExecuteBufferResponse'
        robot_comm_robot_ExecuteJointPosBuffer = 'robot_comm/robot_ExecuteJointPosBuffer'
        robot_comm_robot_ExecuteJointPosBufferRequest = 'robot_comm/robot_ExecuteJointPosBufferRequest'
        robot_comm_robot_ExecuteJointPosBufferResponse = 'robot_comm/robot_ExecuteJointPosBufferResponse'
        robot_comm_robot_ForceLog = 'robot_comm/robot_ForceLog'
        robot_comm_robot_GetCartesian = 'robot_comm/robot_GetCartesian'
        robot_comm_robot_GetCartesianRequest = 'robot_comm/robot_GetCartesianRequest'
        robot_comm_robot_GetCartesianResponse = 'robot_comm/robot_GetCartesianResponse'
        robot_comm_robot_GetFK = 'robot_comm/robot_GetFK'
        robot_comm_robot_GetFKRequest = 'robot_comm/robot_GetFKRequest'
        robot_comm_robot_GetFKResponse = 'robot_comm/robot_GetFKResponse'
        robot_comm_robot_GetIK = 'robot_comm/robot_GetIK'
        robot_comm_robot_GetIKRequest = 'robot_comm/robot_GetIKRequest'
        robot_comm_robot_GetIKResponse = 'robot_comm/robot_GetIKResponse'
        robot_comm_robot_GetJoints = 'robot_comm/robot_GetJoints'
        robot_comm_robot_GetJointsRequest = 'robot_comm/robot_GetJointsRequest'
        robot_comm_robot_GetJointsResponse = 'robot_comm/robot_GetJointsResponse'
        robot_comm_robot_GetRobotAngle = 'robot_comm/robot_GetRobotAngle'
        robot_comm_robot_GetRobotAngleRequest = 'robot_comm/robot_GetRobotAngleRequest'
        robot_comm_robot_GetRobotAngleResponse = 'robot_comm/robot_GetRobotAngleResponse'
        robot_comm_robot_GetState = 'robot_comm/robot_GetState'
        robot_comm_robot_GetStateRequest = 'robot_comm/robot_GetStateRequest'
        robot_comm_robot_GetStateResponse = 'robot_comm/robot_GetStateResponse'
        robot_comm_robot_HandCalibrate = 'robot_comm/robot_HandCalibrate'
        robot_comm_robot_HandCalibrateRequest = 'robot_comm/robot_HandCalibrateRequest'
        robot_comm_robot_HandCalibrateResponse = 'robot_comm/robot_HandCalibrateResponse'
        robot_comm_robot_HandGetPose = 'robot_comm/robot_HandGetPose'
        robot_comm_robot_HandGetPoseRequest = 'robot_comm/robot_HandGetPoseRequest'
        robot_comm_robot_HandGetPoseResponse = 'robot_comm/robot_HandGetPoseResponse'
        robot_comm_robot_HandGetPressure = 'robot_comm/robot_HandGetPressure'
        robot_comm_robot_HandGetPressureRequest = 'robot_comm/robot_HandGetPressureRequest'
        robot_comm_robot_HandGetPressureResponse = 'robot_comm/robot_HandGetPressureResponse'
        robot_comm_robot_HandGripIn = 'robot_comm/robot_HandGripIn'
        robot_comm_robot_HandGripInRequest = 'robot_comm/robot_HandGripInRequest'
        robot_comm_robot_HandGripInResponse = 'robot_comm/robot_HandGripInResponse'
        robot_comm_robot_HandGripOut = 'robot_comm/robot_HandGripOut'
        robot_comm_robot_HandGripOutRequest = 'robot_comm/robot_HandGripOutRequest'
        robot_comm_robot_HandGripOutResponse = 'robot_comm/robot_HandGripOutResponse'
        robot_comm_robot_HandIsCalibrated = 'robot_comm/robot_HandIsCalibrated'
        robot_comm_robot_HandIsCalibratedRequest = 'robot_comm/robot_HandIsCalibratedRequest'
        robot_comm_robot_HandIsCalibratedResponse = 'robot_comm/robot_HandIsCalibratedResponse'
        robot_comm_robot_HandJogIn = 'robot_comm/robot_HandJogIn'
        robot_comm_robot_HandJogInRequest = 'robot_comm/robot_HandJogInRequest'
        robot_comm_robot_HandJogInResponse = 'robot_comm/robot_HandJogInResponse'
        robot_comm_robot_HandJogOut = 'robot_comm/robot_HandJogOut'
        robot_comm_robot_HandJogOutRequest = 'robot_comm/robot_HandJogOutRequest'
        robot_comm_robot_HandJogOutResponse = 'robot_comm/robot_HandJogOutResponse'
        robot_comm_robot_HandMoveTo = 'robot_comm/robot_HandMoveTo'
        robot_comm_robot_HandMoveToRequest = 'robot_comm/robot_HandMoveToRequest'
        robot_comm_robot_HandMoveToResponse = 'robot_comm/robot_HandMoveToResponse'
        robot_comm_robot_HandOffBlow = 'robot_comm/robot_HandOffBlow'
        robot_comm_robot_HandOffBlowRequest = 'robot_comm/robot_HandOffBlowRequest'
        robot_comm_robot_HandOffBlowResponse = 'robot_comm/robot_HandOffBlowResponse'
        robot_comm_robot_HandOffVacuum = 'robot_comm/robot_HandOffVacuum'
        robot_comm_robot_HandOffVacuumRequest = 'robot_comm/robot_HandOffVacuumRequest'
        robot_comm_robot_HandOffVacuumResponse = 'robot_comm/robot_HandOffVacuumResponse'
        robot_comm_robot_HandOnBlow = 'robot_comm/robot_HandOnBlow'
        robot_comm_robot_HandOnBlowRequest = 'robot_comm/robot_HandOnBlowRequest'
        robot_comm_robot_HandOnBlowResponse = 'robot_comm/robot_HandOnBlowResponse'
        robot_comm_robot_HandOnVacuum = 'robot_comm/robot_HandOnVacuum'
        robot_comm_robot_HandOnVacuumRequest = 'robot_comm/robot_HandOnVacuumRequest'
        robot_comm_robot_HandOnVacuumResponse = 'robot_comm/robot_HandOnVacuumResponse'
        robot_comm_robot_HandSetForce = 'robot_comm/robot_HandSetForce'
        robot_comm_robot_HandSetForceRequest = 'robot_comm/robot_HandSetForceRequest'
        robot_comm_robot_HandSetForceResponse = 'robot_comm/robot_HandSetForceResponse'
        robot_comm_robot_HandSetSpeed = 'robot_comm/robot_HandSetSpeed'
        robot_comm_robot_HandSetSpeedRequest = 'robot_comm/robot_HandSetSpeedRequest'
        robot_comm_robot_HandSetSpeedResponse = 'robot_comm/robot_HandSetSpeedResponse'
        robot_comm_robot_HandStop = 'robot_comm/robot_HandStop'
        robot_comm_robot_HandStopRequest = 'robot_comm/robot_HandStopRequest'
        robot_comm_robot_HandStopResponse = 'robot_comm/robot_HandStopResponse'
        robot_comm_robot_IOSignal = 'robot_comm/robot_IOSignal'
        robot_comm_robot_IOSignalRequest = 'robot_comm/robot_IOSignalRequest'
        robot_comm_robot_IOSignalResponse = 'robot_comm/robot_IOSignalResponse'
        robot_comm_robot_IsMoving = 'robot_comm/robot_IsMoving'
        robot_comm_robot_IsMovingRequest = 'robot_comm/robot_IsMovingRequest'
        robot_comm_robot_IsMovingResponse = 'robot_comm/robot_IsMovingResponse'
        robot_comm_robot_JointsLog = 'robot_comm/robot_JointsLog'
        robot_comm_robot_Ping = 'robot_comm/robot_Ping'
        robot_comm_robot_PingRequest = 'robot_comm/robot_PingRequest'
        robot_comm_robot_PingResponse = 'robot_comm/robot_PingResponse'
        robot_comm_robot_SetAcc = 'robot_comm/robot_SetAcc'
        robot_comm_robot_SetAccRequest = 'robot_comm/robot_SetAccRequest'
        robot_comm_robot_SetAccResponse = 'robot_comm/robot_SetAccResponse'
        robot_comm_robot_SetCartesianA = 'robot_comm/robot_SetCartesianA'
        robot_comm_robot_SetCartesianARequest = 'robot_comm/robot_SetCartesianARequest'
        robot_comm_robot_SetCartesianAResponse = 'robot_comm/robot_SetCartesianAResponse'
        robot_comm_robot_SetCartesianJ = 'robot_comm/robot_SetCartesianJ'
        robot_comm_robot_SetCartesianJRequest = 'robot_comm/robot_SetCartesianJRequest'
        robot_comm_robot_SetCartesianJResponse = 'robot_comm/robot_SetCartesianJResponse'
        robot_comm_robot_SetCartesian = 'robot_comm/robot_SetCartesian'
        robot_comm_robot_SetCartesianRequest = 'robot_comm/robot_SetCartesianRequest'
        robot_comm_robot_SetCartesianResponse = 'robot_comm/robot_SetCartesianResponse'
        robot_comm_robot_SetComm = 'robot_comm/robot_SetComm'
        robot_comm_robot_SetCommRequest = 'robot_comm/robot_SetCommRequest'
        robot_comm_robot_SetCommResponse = 'robot_comm/robot_SetCommResponse'
        robot_comm_robot_SetDefaults = 'robot_comm/robot_SetDefaults'
        robot_comm_robot_SetDefaultsRequest = 'robot_comm/robot_SetDefaultsRequest'
        robot_comm_robot_SetDefaultsResponse = 'robot_comm/robot_SetDefaultsResponse'
        robot_comm_robot_SetInertia = 'robot_comm/robot_SetInertia'
        robot_comm_robot_SetInertiaRequest = 'robot_comm/robot_SetInertiaRequest'
        robot_comm_robot_SetInertiaResponse = 'robot_comm/robot_SetInertiaResponse'
        robot_comm_robot_SetJoints = 'robot_comm/robot_SetJoints'
        robot_comm_robot_SetJointsRequest = 'robot_comm/robot_SetJointsRequest'
        robot_comm_robot_SetJointsResponse = 'robot_comm/robot_SetJointsResponse'
        robot_comm_robot_SetMotionSupervision = 'robot_comm/robot_SetMotionSupervision'
        robot_comm_robot_SetMotionSupervisionRequest = 'robot_comm/robot_SetMotionSupervisionRequest'
        robot_comm_robot_SetMotionSupervisionResponse = 'robot_comm/robot_SetMotionSupervisionResponse'
        robot_comm_robot_SetSpeed = 'robot_comm/robot_SetSpeed'
        robot_comm_robot_SetSpeedRequest = 'robot_comm/robot_SetSpeedRequest'
        robot_comm_robot_SetSpeedResponse = 'robot_comm/robot_SetSpeedResponse'
        robot_comm_robot_SetTool = 'robot_comm/robot_SetTool'
        robot_comm_robot_SetToolRequest = 'robot_comm/robot_SetToolRequest'
        robot_comm_robot_SetToolResponse = 'robot_comm/robot_SetToolResponse'
        robot_comm_robot_SetTrackDist = 'robot_comm/robot_SetTrackDist'
        robot_comm_robot_SetTrackDistRequest = 'robot_comm/robot_SetTrackDistRequest'
        robot_comm_robot_SetTrackDistResponse = 'robot_comm/robot_SetTrackDistResponse'
        robot_comm_robot_SetWorkObject = 'robot_comm/robot_SetWorkObject'
        robot_comm_robot_SetWorkObjectRequest = 'robot_comm/robot_SetWorkObjectRequest'
        robot_comm_robot_SetWorkObjectResponse = 'robot_comm/robot_SetWorkObjectResponse'
        robot_comm_robot_SetZone = 'robot_comm/robot_SetZone'
        robot_comm_robot_SetZoneRequest = 'robot_comm/robot_SetZoneRequest'
        robot_comm_robot_SetZoneResponse = 'robot_comm/robot_SetZoneResponse'
        robot_comm_robot_Stop = 'robot_comm/robot_Stop'
        robot_comm_robot_StopRequest = 'robot_comm/robot_StopRequest'
        robot_comm_robot_StopResponse = 'robot_comm/robot_StopResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(103, 1);
                msgList{1} = 'robot_comm/robot_ActivateCSSRequest';
                msgList{2} = 'robot_comm/robot_ActivateCSSResponse';
                msgList{3} = 'robot_comm/robot_ActivateEGMRequest';
                msgList{4} = 'robot_comm/robot_ActivateEGMResponse';
                msgList{5} = 'robot_comm/robot_AddBufferRequest';
                msgList{6} = 'robot_comm/robot_AddBufferResponse';
                msgList{7} = 'robot_comm/robot_AddJointPosBufferRequest';
                msgList{8} = 'robot_comm/robot_AddJointPosBufferResponse';
                msgList{9} = 'robot_comm/robot_ApproachRequest';
                msgList{10} = 'robot_comm/robot_ApproachResponse';
                msgList{11} = 'robot_comm/robot_CartesianLog';
                msgList{12} = 'robot_comm/robot_ClearBufferRequest';
                msgList{13} = 'robot_comm/robot_ClearBufferResponse';
                msgList{14} = 'robot_comm/robot_ClearJointPosBufferRequest';
                msgList{15} = 'robot_comm/robot_ClearJointPosBufferResponse';
                msgList{16} = 'robot_comm/robot_DeactivateCSSRequest';
                msgList{17} = 'robot_comm/robot_DeactivateCSSResponse';
                msgList{18} = 'robot_comm/robot_ExecuteBufferRequest';
                msgList{19} = 'robot_comm/robot_ExecuteBufferResponse';
                msgList{20} = 'robot_comm/robot_ExecuteJointPosBufferRequest';
                msgList{21} = 'robot_comm/robot_ExecuteJointPosBufferResponse';
                msgList{22} = 'robot_comm/robot_ForceLog';
                msgList{23} = 'robot_comm/robot_GetCartesianRequest';
                msgList{24} = 'robot_comm/robot_GetCartesianResponse';
                msgList{25} = 'robot_comm/robot_GetFKRequest';
                msgList{26} = 'robot_comm/robot_GetFKResponse';
                msgList{27} = 'robot_comm/robot_GetIKRequest';
                msgList{28} = 'robot_comm/robot_GetIKResponse';
                msgList{29} = 'robot_comm/robot_GetJointsRequest';
                msgList{30} = 'robot_comm/robot_GetJointsResponse';
                msgList{31} = 'robot_comm/robot_GetRobotAngleRequest';
                msgList{32} = 'robot_comm/robot_GetRobotAngleResponse';
                msgList{33} = 'robot_comm/robot_GetStateRequest';
                msgList{34} = 'robot_comm/robot_GetStateResponse';
                msgList{35} = 'robot_comm/robot_HandCalibrateRequest';
                msgList{36} = 'robot_comm/robot_HandCalibrateResponse';
                msgList{37} = 'robot_comm/robot_HandGetPoseRequest';
                msgList{38} = 'robot_comm/robot_HandGetPoseResponse';
                msgList{39} = 'robot_comm/robot_HandGetPressureRequest';
                msgList{40} = 'robot_comm/robot_HandGetPressureResponse';
                msgList{41} = 'robot_comm/robot_HandGripInRequest';
                msgList{42} = 'robot_comm/robot_HandGripInResponse';
                msgList{43} = 'robot_comm/robot_HandGripOutRequest';
                msgList{44} = 'robot_comm/robot_HandGripOutResponse';
                msgList{45} = 'robot_comm/robot_HandIsCalibratedRequest';
                msgList{46} = 'robot_comm/robot_HandIsCalibratedResponse';
                msgList{47} = 'robot_comm/robot_HandJogInRequest';
                msgList{48} = 'robot_comm/robot_HandJogInResponse';
                msgList{49} = 'robot_comm/robot_HandJogOutRequest';
                msgList{50} = 'robot_comm/robot_HandJogOutResponse';
                msgList{51} = 'robot_comm/robot_HandMoveToRequest';
                msgList{52} = 'robot_comm/robot_HandMoveToResponse';
                msgList{53} = 'robot_comm/robot_HandOffBlowRequest';
                msgList{54} = 'robot_comm/robot_HandOffBlowResponse';
                msgList{55} = 'robot_comm/robot_HandOffVacuumRequest';
                msgList{56} = 'robot_comm/robot_HandOffVacuumResponse';
                msgList{57} = 'robot_comm/robot_HandOnBlowRequest';
                msgList{58} = 'robot_comm/robot_HandOnBlowResponse';
                msgList{59} = 'robot_comm/robot_HandOnVacuumRequest';
                msgList{60} = 'robot_comm/robot_HandOnVacuumResponse';
                msgList{61} = 'robot_comm/robot_HandSetForceRequest';
                msgList{62} = 'robot_comm/robot_HandSetForceResponse';
                msgList{63} = 'robot_comm/robot_HandSetSpeedRequest';
                msgList{64} = 'robot_comm/robot_HandSetSpeedResponse';
                msgList{65} = 'robot_comm/robot_HandStopRequest';
                msgList{66} = 'robot_comm/robot_HandStopResponse';
                msgList{67} = 'robot_comm/robot_IOSignalRequest';
                msgList{68} = 'robot_comm/robot_IOSignalResponse';
                msgList{69} = 'robot_comm/robot_IsMovingRequest';
                msgList{70} = 'robot_comm/robot_IsMovingResponse';
                msgList{71} = 'robot_comm/robot_JointsLog';
                msgList{72} = 'robot_comm/robot_PingRequest';
                msgList{73} = 'robot_comm/robot_PingResponse';
                msgList{74} = 'robot_comm/robot_SetAccRequest';
                msgList{75} = 'robot_comm/robot_SetAccResponse';
                msgList{76} = 'robot_comm/robot_SetCartesianARequest';
                msgList{77} = 'robot_comm/robot_SetCartesianAResponse';
                msgList{78} = 'robot_comm/robot_SetCartesianJRequest';
                msgList{79} = 'robot_comm/robot_SetCartesianJResponse';
                msgList{80} = 'robot_comm/robot_SetCartesianRequest';
                msgList{81} = 'robot_comm/robot_SetCartesianResponse';
                msgList{82} = 'robot_comm/robot_SetCommRequest';
                msgList{83} = 'robot_comm/robot_SetCommResponse';
                msgList{84} = 'robot_comm/robot_SetDefaultsRequest';
                msgList{85} = 'robot_comm/robot_SetDefaultsResponse';
                msgList{86} = 'robot_comm/robot_SetInertiaRequest';
                msgList{87} = 'robot_comm/robot_SetInertiaResponse';
                msgList{88} = 'robot_comm/robot_SetJointsRequest';
                msgList{89} = 'robot_comm/robot_SetJointsResponse';
                msgList{90} = 'robot_comm/robot_SetMotionSupervisionRequest';
                msgList{91} = 'robot_comm/robot_SetMotionSupervisionResponse';
                msgList{92} = 'robot_comm/robot_SetSpeedRequest';
                msgList{93} = 'robot_comm/robot_SetSpeedResponse';
                msgList{94} = 'robot_comm/robot_SetToolRequest';
                msgList{95} = 'robot_comm/robot_SetToolResponse';
                msgList{96} = 'robot_comm/robot_SetTrackDistRequest';
                msgList{97} = 'robot_comm/robot_SetTrackDistResponse';
                msgList{98} = 'robot_comm/robot_SetWorkObjectRequest';
                msgList{99} = 'robot_comm/robot_SetWorkObjectResponse';
                msgList{100} = 'robot_comm/robot_SetZoneRequest';
                msgList{101} = 'robot_comm/robot_SetZoneResponse';
                msgList{102} = 'robot_comm/robot_StopRequest';
                msgList{103} = 'robot_comm/robot_StopResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(50, 1);
                svcList{1} = 'robot_comm/robot_ActivateCSS';
                svcList{2} = 'robot_comm/robot_ActivateEGM';
                svcList{3} = 'robot_comm/robot_AddBuffer';
                svcList{4} = 'robot_comm/robot_AddJointPosBuffer';
                svcList{5} = 'robot_comm/robot_Approach';
                svcList{6} = 'robot_comm/robot_ClearBuffer';
                svcList{7} = 'robot_comm/robot_ClearJointPosBuffer';
                svcList{8} = 'robot_comm/robot_DeactivateCSS';
                svcList{9} = 'robot_comm/robot_ExecuteBuffer';
                svcList{10} = 'robot_comm/robot_ExecuteJointPosBuffer';
                svcList{11} = 'robot_comm/robot_GetCartesian';
                svcList{12} = 'robot_comm/robot_GetFK';
                svcList{13} = 'robot_comm/robot_GetIK';
                svcList{14} = 'robot_comm/robot_GetJoints';
                svcList{15} = 'robot_comm/robot_GetRobotAngle';
                svcList{16} = 'robot_comm/robot_GetState';
                svcList{17} = 'robot_comm/robot_HandCalibrate';
                svcList{18} = 'robot_comm/robot_HandGetPose';
                svcList{19} = 'robot_comm/robot_HandGetPressure';
                svcList{20} = 'robot_comm/robot_HandGripIn';
                svcList{21} = 'robot_comm/robot_HandGripOut';
                svcList{22} = 'robot_comm/robot_HandIsCalibrated';
                svcList{23} = 'robot_comm/robot_HandJogIn';
                svcList{24} = 'robot_comm/robot_HandJogOut';
                svcList{25} = 'robot_comm/robot_HandMoveTo';
                svcList{26} = 'robot_comm/robot_HandOffBlow';
                svcList{27} = 'robot_comm/robot_HandOffVacuum';
                svcList{28} = 'robot_comm/robot_HandOnBlow';
                svcList{29} = 'robot_comm/robot_HandOnVacuum';
                svcList{30} = 'robot_comm/robot_HandSetForce';
                svcList{31} = 'robot_comm/robot_HandSetSpeed';
                svcList{32} = 'robot_comm/robot_HandStop';
                svcList{33} = 'robot_comm/robot_IOSignal';
                svcList{34} = 'robot_comm/robot_IsMoving';
                svcList{35} = 'robot_comm/robot_Ping';
                svcList{36} = 'robot_comm/robot_SetAcc';
                svcList{37} = 'robot_comm/robot_SetCartesianA';
                svcList{38} = 'robot_comm/robot_SetCartesianJ';
                svcList{39} = 'robot_comm/robot_SetCartesian';
                svcList{40} = 'robot_comm/robot_SetComm';
                svcList{41} = 'robot_comm/robot_SetDefaults';
                svcList{42} = 'robot_comm/robot_SetInertia';
                svcList{43} = 'robot_comm/robot_SetJoints';
                svcList{44} = 'robot_comm/robot_SetMotionSupervision';
                svcList{45} = 'robot_comm/robot_SetSpeed';
                svcList{46} = 'robot_comm/robot_SetTool';
                svcList{47} = 'robot_comm/robot_SetTrackDist';
                svcList{48} = 'robot_comm/robot_SetWorkObject';
                svcList{49} = 'robot_comm/robot_SetZone';
                svcList{50} = 'robot_comm/robot_Stop';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
