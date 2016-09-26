classdef ExampleHelperGazeboSpawnedModel_corrected < handle
    %ExampleHelperGazeboSpawnedModel_corrected - Class controlling user-spawned models in Gazebo
    %   M = ExampleHelperGazeboSpawnedModel_corrected(NAME,WORLD) creates an object that allows for easy
    %   control and interaction with user-spawned models in the Gazebo simulator.
    %   The constructor takes arguments of NAME and WORLD, corresponding to the
    %   name of  model and the Communicator object associated with the
    %   simulation.
    %
    %   ExampleHelperGazeboSpawnedModel_corrected methods:
    %       getState                - Reads the state of the model
    %       getComponents           - Gets components of the model
    %       jointTorque             - Applies effort on joints
    %       applyForce              - Applies force on model
    %       setState                - Sets state of the model
    %       setConfig               - Sets joint configurations of the model
    %
    %   ExampleHelperGazeboSpawnedModel_corrected properties:
    %       Name                    - Name of the Model
    %       Links                   - List of Link Names
    %       Joints                  - List of Joint Names
    %
    %   See also ExampleHelperGazeboCommunicator, ExampleHelperGazeboModel
    
    %   Copyright 2014 The MathWorks, Inc.
    properties (SetAccess = protected)
        Name = [];                % Name of the Model
        Links = [];               % List of Link Names
        Joints = [];              % List of Joint Names
    end
    
    properties (Access = protected)
        % The user can retrieve these properties only using the getState
        % function
        
        Position = [];            % The Model's Position
        Orientation = [];         % The Model's Orientation
        Velocity = [];            % Linear and angular components base velocity of the Model
        
    end
    
    properties (Access = private)
        BodyForceClient = [];     % Service Client for applying force
        JointTorqueClient = [];   % Service Client for applying joint torque
        GetStateClient = [];      % Service Client for getting state of the model
        GetPropClient = [];       % Service Client for getting properties of the model
        SetStateClient = [];      % Service Client for setting the state of the model
        SetConfigClient = [];     % Service Client for setting joint configurations of the model
    end
    
    methods
        
        function obj = ExampleHelperGazeboSpawnedModel_corrected(Name, World)
            %ExampleHelperGazeboSpawnedModel_corrected Constructor
            
            if ~World.IsModelServicesRunning
                startModelServices(World);
                World.IsModelServicesRunning = 1;
            end
            
            % Initialize all Service Clients
            obj.BodyForceClient = World.BodyForceClient;
            obj.JointTorqueClient = World.JointTorqueClient;
            obj.GetStateClient = World.GetModStateClient;
            obj.GetPropClient = World.GetModPropClient;
            obj.SetStateClient = World.SetModStateClient;
            obj.SetConfigClient = World.SetModConfigClient;
            
            obj.Name = Name;
            [obj.Links, obj.Joints] = getComponents(obj);
            [obj.Position, obj.Orientation, obj.Velocity] = getState(obj);
            
        end
        
        
        function [position, orientation, velocity] = getState(obj)
            %GETSTATE - Reads the state of the model
            
            try
                serviceMsg = rosmessage(obj.GetStateClient);
                serviceMsg.ModelName = obj.Name;
                msg = call(obj.GetStateClient,serviceMsg);
                if msg.Success
                    position = msg.Pose.Position;
                    orient = msg.Pose.Orientation;
                    linear = msg.Twist.Linear;
                    angular = msg.Twist.Angular;
                    
                    position = [position.X position.Y position.Z];
                    
                    quat = [orient.W orient.X orient.Y, orient.Z];
                    r = quat2eul(quat);
                    orientation = [r(1) r(2) r(3)];
                    
                    
                    velocity.Linear = [linear.X linear.Y linear.Z];
                    velocity.Angular = [angular.X angular.Y angular.Z];
                    
                    obj.Position = position;
                    obj.Orientation = orientation;
                    obj.Velocity = velocity;
                end
                
            catch
                error('Error during service call, cannot get the state of the model')
            end
            
        end
        
        function [links, joints] = getComponents(obj)
            %GETCOMPONENTS - Gets components of the model
            
            try
                serviceMsg = rosmessage(obj.GetPropClient);
                serviceMsg.ModelName = obj.Name;
                msg = call(obj.GetPropClient,serviceMsg);
                
                if msg.Success
                    links = msg.BodyNames;
                    joints = msg.JointNames;
                end
                obj.Links = links;
                obj.Joints = joints;
            catch
                error('Error during service call. Cannot obtain links and joints')
            end
            
        end
        
        function jointTorque(obj, jointname, duration, effort)
            %JOINTTORQUE - Applies effort on joints
            
            serviceMsg = rosmessage(obj.JointTorqueClient);
            serviceMsg.JointName = [obj.Name '::' jointname];
            
            serviceMsg.Duration.Sec = duration;
            serviceMsg.StartTime.Sec = 0;
            serviceMsg.StartTime.Nsec = 10;
            
            serviceMsg.Effort = effort;
            msg = call(obj.JointTorqueClient,serviceMsg);
            if ~msg.Success
                error('Error during service call. Cannot set joint torque')
            end
        end
        
        function applyForce(obj, linkname, duration, varargin)
            %APPLYFORCE - Applies force on model
            
            if nargin == 3
                force = [1 0 0];
                torque = [0 0 0];
            elseif nargin == 4
                force = varargin{1};
                torque = [0 0 0];
            elseif nargin == 5
                force = varargin{1};
                torque = varargin{2};
            end
            
            serviceMsg = rosmessage(obj.BodyForceClient);
            serviceMsg.BodyName = [obj.Name '::' linkname];
            
            serviceMsg.Duration.Sec = duration;
            serviceMsg.StartTime.Sec = 0;
            serviceMsg.StartTime.Nsec = 0;
            
            
            wrench_msg = rosmessage('geometry_msgs/Wrench');
            vec3_msg = rosmessage('geometry_msgs/Vector3');
            vec3_msg.X = force(1);
            vec3_msg.Y = force(2);
            vec3_msg.Z = force(3);
            wrench_msg.Force = vec3_msg;
            
            
            vec3_msg = rosmessage('geometry_msgs/Vector3');
            vec3_msg.X = torque(1);
            vec3_msg.Y = torque(2);
            vec3_msg.Z = torque(3);
            wrench_msg.Torque = vec3_msg;
            
            serviceMsg.Wrench = wrench_msg;
            call(obj.BodyForceClient,serviceMsg);
        end
        
        function setState(obj, varargin)
            %SETSTATE - Sets state of the model
            
            orientation = obj.Orientation;
            velocity = obj.Velocity;
            position = obj.Position;
            
            options = varargin;
            numOptions = (nargin-1)/2;
            for k = 1:numOptions
                opt = options{2*k-1};
                val = options{2*k};
                if strcmpi(opt,'position')
                    position = val;
                end
                if strcmpi(opt,'orientation')
                    orientation = val;
                end
                if strcmpi(opt,'linvel')
                    velocity.Linear = val;
                end
                if strcmpi(opt,'angvel')
                    velocity.Angular = val;
                end
            end
            
            serviceMsg = rosmessage(obj.SetStateClient);
            
            modelState = serviceMsg.ModelState;
            modelState.ModelName = obj.Name;
            
            pose = rosmessage('geometry_msgs/Pose');
            Point = pose.Position;
            Point.X = position(1);
            Point.Y = position(2);
            Point.Z = position(3);
            
            orient = pose.Orientation;
            orientation = eul2quat([orientation(1),orientation(2),orientation(3)]);
            
            orient.W = orientation(1);
            orient.X = orientation(2);
            orient.Y = orientation(3);
            orient.Z = orientation(4);
            
            pose.Position = Point;
            pose.Orientation = orient;
            
            modelState.Pose = pose;
            
            twist = rosmessage('geometry_msgs/Twist');
            Linear = twist.Linear;
            Linear.X = velocity.Linear(1);
            Linear.Y = velocity.Linear(2);
            Linear.Z = velocity.Linear(3);
            
            Angular = twist.Angular;
            Angular.X = velocity.Angular(1);
            Angular.Y = velocity.Angular(2);
            Angular.Z = velocity.Angular(3);
            
            twist.Linear = Linear;
            twist.Angular = Angular;
            
            modelState.Twist = twist;
            
            serviceMsg.ModelState = modelState;
            call(obj.SetStateClient,serviceMsg);
            
        end
        
        function setConfig(obj, jointnames, jointpositions)
            %SETCONFIG - Sets joint configurations of the model
            
            if ~iscell(jointnames);
                jointnames = {jointnames};
            end
            serviceMsg = rosmessage(obj.SetConfigClient);
            
            if ~isequal(numel(jointnames),numel(jointpositions))
                error('Input length mismatch: The number of joints must agree with the length of the configuration vector')
            end
            
            serviceMsg.ModelName = obj.Name;
            serviceMsg.JointNames = jointnames;
            serviceMsg.JointPositions = jointpositions;
            
            msg = call(obj.SetConfigClient, serviceMsg);
            if ~msg.Success
                error('Configuration not set')
            end
        end
        
        
    end
end

