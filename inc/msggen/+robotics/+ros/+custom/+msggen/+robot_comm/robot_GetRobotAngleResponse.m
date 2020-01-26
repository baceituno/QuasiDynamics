classdef robot_GetRobotAngleResponse < robotics.ros.Message
    %robot_GetRobotAngleResponse MATLAB implementation of robot_comm/robot_GetRobotAngleResponse
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'robot_comm/robot_GetRobotAngleResponse' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'fac89e6347e18a1d539561badc84a2ab' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Angle
        Ret
        Msg
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Angle', 'Msg', 'Ret'} % List of non-constant message properties
        ROSPropertyList = {'angle', 'msg', 'ret'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = robot_GetRobotAngleResponse(msg)
            %robot_GetRobotAngleResponse Construct the message object robot_GetRobotAngleResponse
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function angle = get.Angle(obj)
            %get.Angle Get the value for property Angle
            angle = double(obj.JavaMessage.getAngle);
        end
        
        function set.Angle(obj, angle)
            %set.Angle Set the value for property Angle
            validateattributes(angle, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetRobotAngleResponse', 'Angle');
            
            obj.JavaMessage.setAngle(angle);
        end
        
        function ret = get.Ret(obj)
            %get.Ret Get the value for property Ret
            ret = int64(obj.JavaMessage.getRet);
        end
        
        function set.Ret(obj, ret)
            %set.Ret Set the value for property Ret
            validateattributes(ret, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetRobotAngleResponse', 'Ret');
            
            obj.JavaMessage.setRet(ret);
        end
        
        function msg = get.Msg(obj)
            %get.Msg Get the value for property Msg
            msg = char(obj.JavaMessage.getMsg);
        end
        
        function set.Msg(obj, msg)
            %set.Msg Set the value for property Msg
            msg = convertStringsToChars(msg);
            
            validateattributes(msg, {'char', 'string'}, {}, 'robot_GetRobotAngleResponse', 'Msg');
            
            obj.JavaMessage.setMsg(msg);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Angle = obj.Angle;
            cpObj.Ret = obj.Ret;
            cpObj.Msg = obj.Msg;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Angle = strObj.Angle;
            obj.Ret = strObj.Ret;
            obj.Msg = strObj.Msg;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Angle = obj.Angle;
            strObj.Ret = obj.Ret;
            strObj.Msg = obj.Msg;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.robot_comm.robot_GetRobotAngleResponse.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.robot_comm.robot_GetRobotAngleResponse;
            obj.reload(strObj);
        end
    end
end
