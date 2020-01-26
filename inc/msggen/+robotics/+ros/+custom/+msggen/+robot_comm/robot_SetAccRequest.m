classdef robot_SetAccRequest < robotics.ros.Message
    %robot_SetAccRequest MATLAB implementation of robot_comm/robot_SetAccRequest
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'robot_comm/robot_SetAccRequest' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '0bda2379c12ec37c50b08acf18e9ddf6' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        Acc
        Deacc
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Acc', 'Deacc'} % List of non-constant message properties
        ROSPropertyList = {'acc', 'deacc'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = robot_SetAccRequest(msg)
            %robot_SetAccRequest Construct the message object robot_SetAccRequest
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
        
        function acc = get.Acc(obj)
            %get.Acc Get the value for property Acc
            acc = double(obj.JavaMessage.getAcc);
        end
        
        function set.Acc(obj, acc)
            %set.Acc Set the value for property Acc
            validateattributes(acc, {'numeric'}, {'nonempty', 'scalar'}, 'robot_SetAccRequest', 'Acc');
            
            obj.JavaMessage.setAcc(acc);
        end
        
        function deacc = get.Deacc(obj)
            %get.Deacc Get the value for property Deacc
            deacc = double(obj.JavaMessage.getDeacc);
        end
        
        function set.Deacc(obj, deacc)
            %set.Deacc Set the value for property Deacc
            validateattributes(deacc, {'numeric'}, {'nonempty', 'scalar'}, 'robot_SetAccRequest', 'Deacc');
            
            obj.JavaMessage.setDeacc(deacc);
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
            cpObj.Acc = obj.Acc;
            cpObj.Deacc = obj.Deacc;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Acc = strObj.Acc;
            obj.Deacc = strObj.Deacc;
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
            
            strObj.Acc = obj.Acc;
            strObj.Deacc = obj.Deacc;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.robot_comm.robot_SetAccRequest.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.robot_comm.robot_SetAccRequest;
            obj.reload(strObj);
        end
    end
end