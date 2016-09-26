classdef ExampleHelperGazeboCommunicator_corrected < handle
    %ExampleHelperGazeboCommunicator_corrected - Class for communicating with Gazebo
    %   G = ExampleHelperGazeboCommunicator_corrected() creates an object that allows for easy
    %   control and interaction with the Gazebo simulator. The
    %   constructor takes no arguments and sets up service clients for easy
    %   communication with Gazebo
    %
    %   ExampleHelperGazeboCommunicator_corrected methods:
    %       pauseSim            - Calls service to pause simulation
    %       resumeSim           - Calls service to resume simulation after a pause
    %       resetSim            - Calls service to reset simulation
    %       resetWorld          - Calls service to reset the world
    %       readPhysics         - Returns struct of simulation physics properties
    %       setPhysics          - Sets simulation physics properties
    %       getSpawnedModels    - Returns a list of all spawned models
    %       spawnModel          - Creates a new model in the simulation
    %       removeModel         - Removes a specified model from the simulation
    %
    %   ExampleHelperGazeboCommunicator_corrected properties:
    %       Physics                     - Physics parameters of the simulation engine
    %       ModelsList                  - List of models in Gazebo world
    %       IsModelServicesRunning      - Flag to check if model services are running
    %       BodyForceClient             - Service Client for applying force
    %       JointTorqueClient           - Service Client for applying joint torque
    %       SetModStateClient           - Service Client for setting model state
    %       GetModPropClient            - Service Client for fetching model properties
    %       GetModStateClient           - Service Client for fetching model state
    %       SetModConfigClient          - Service Client for setting joint configurations
    %
    %   See also ExampleHelperGazeboModel_corrected, ExampleHelperGazeboSpawnedModel_corrected
    
    %   Copyright 2014 The MathWorks, Inc.
    
    properties (Dependent, SetAccess = protected)
        Physics = [];               % Physics parameters of the simulation engine
        ModelsList = [];            % List of models in Gazebo world
    end
    
    properties (Access = ?ExampleHelperGazeboSpawnedModel_corrected)
        IsModelServicesRunning = 0; % Flag to check if model services are running
        BodyForceClient = [];       % Service Client for applying force
        JointTorqueClient = [];     % Service Client for applying joint torque
        SetModStateClient = [];     % Service Client for setting model state
        GetModPropClient = [];      % Service Client for fetching model properties
        GetModStateClient = [];     % Service Client for fetching model state
        SetModConfigClient = [];    % Service Client for setting joint configurations
    end
    
    properties (Access = protected)
        
        TimeSub = [];               % Subscriber for simulation time
        
        PausePhyClient = [];        % Service Client for pausing physics
        ResumePhyClient = [];       % Service Client for resuming physics
        ReadPhyClient = [];         % Service Client for fetching physics properties
        ResetSimClient = [];        % Service Client for resetting simulation
        ResetWorldClient = [];      % Service Client for resetting world
        SpawnClient = [];           % Service Client for spawning a model
        SetPhyClient = [];          % Service Client for setting physics properties
        ModelListClient = [];       % Service Client for listing models
        RemoveModClient = [];       % Service Client for removing models
        
        
    end
    
    methods (Access = public)
        function obj = ExampleHelperGazeboCommunicator_corrected()
            %ExampleHelperGazeboCommunicator_corrected Constructor
            
            % Initialize service clients
            initServiceClients(obj);
            
        end
        
        function pauseSim(obj)
            %pauseSim - Calls service to pause simulation
            
            serviceMsg = rosmessage(obj.PausePhyClient);
            call(obj.PausePhyClient,serviceMsg);
            
        end
        
        function resumeSim(obj)
            %resumeSim - Calls service to resume simulation after a pause
            
            serviceMsg = rosmessage(obj.ResumePhyClient);
            call(obj.ResumePhyClient,serviceMsg);
            
        end
        
        function resetSim(obj)
            %resetSim - Calls service to reset simulation
            
            serviceMsg = rosmessage(obj.ResetSimClient);
            call(obj.ResetSimClient,serviceMsg);
            
        end
        
        function resetWorld(obj)
            % RESETWORLD - Calls service to reset the world
            
            serviceMsg = rosmessage(obj.ResetWorldClient);
            call(obj.ResetWorldClient,serviceMsg);
        end
        
        
        
        function physics = readPhysics(obj)
            %readPhysics - Returns struct of physics properties of the
            % simulation
            %
            % # enable auto disabling of bodies, default false
            % # preconditioning inner iterations when using projected Gauss Seidel
            % # inner iterations when using projected Gauss Seidel
            % # relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
            % # rms error tolerance before stopping inner iterations
            % # contact "dead-band" width
            % # contact maximum correction velocity
            % # global constraint force mixing
            % # global error reduction parameter
            % # maximum contact joints between two geometries
            try
                % Use service to retrieve properties
                serviceMsg = rosmessage(obj.ReadPhyClient);
                msg = call(obj.ReadPhyClient,serviceMsg);
                
                if msg.Success
                    % Set properties in the struct
                    physics.Gravity = [msg.Gravity.X msg.Gravity.Y msg.Gravity.Z];  % Gravity: m/s^2
                    physics.UpdateRate = msg.MaxUpdateRate;   % Update rate: #/second
                    physics.TimeStep = msg.TimeStep;   % Time step: dt in seconds
                    physics.SimulationStatus = msg.Pause;   % Boolean for status: true = paused
                    
                    physics.DisableBodies = msg.OdeConfig.AutoDisableBodies; % Enable automatic disabling of bodies
                    physics.PreconditioningIterations = msg.OdeConfig.SorPgsPreconIters; % Preconditioning inner iterations when using projected Gauss Seidel
                    physics.InnerIterations = msg.OdeConfig.SorPgsIters; % Inner iterations when using Gauss Seidel
                    physics.Relaxation = msg.OdeConfig.SorPgsW;  % Relaxation parameter when using Gauss Seidel (1 means no relaxation)
                    physics.ErrorToleranceRMS = msg.OdeConfig.SorPgsRmsErrorTol;  % RMS error tolerance before stopping inner iterations
                    
                    physics.ContactWidth = msg.OdeConfig.ContactSurfaceLayer;    % Contact dead-band width (m)
                    physics.MaxCorrectingVelocity = msg.OdeConfig.ContactMaxCorrectingVel;    % Contact maximum correction velocity
                    
                    physics.ConstantForceMixing = msg.OdeConfig.Cfm;  % Constant Force Mixing
                    physics.ErrorReductionParameter = msg.OdeConfig.Erp;  % Error Reduction Parameter
                    
                    physics.MaxContacts = msg.OdeConfig.MaxContacts;  % Maximum contact joints between two geometries
                end
            catch
                error('Error reading the physics properties')
            end
        end
        
        function setPhysics(obj,phy)
            %setPhysics - Sets simulation physics using the struct format
            % from GETPHYSICS
            %
            % # enable auto disabling of bodies, default false
            % # preconditioning inner iterations when using projected Gauss Seidel
            % # inner iterations when using projected Gauss Seidel
            % # relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
            % # rms error tolerance before stopping inner iterations
            % # contact "dead-band" width
            % # contact maximum correction velocity
            % # global constraint force mixing
            % # global error reduction parameter
            % # maximum contact joints between two geometries
            
            % Unpack phys struct and set message values accordingly
            serviceMsg = rosmessage(obj.SetPhyClient);
            
            vec3_msg = rosmessage('geometry_msgs/Vector3');
            vec3_msg.X = phy.Gravity(1);
            vec3_msg.Y = phy.Gravity(2);
            vec3_msg.Z = phy.Gravity(3);
            
            serviceMsg.Gravity = vec3_msg;
            
            serviceMsg.MaxUpdateRate = phy.UpdateRate;
            serviceMsg.TimeStep = phy.TimeStep;
            
            odeConfig_msg = rosmessage('gazebo_msgs/ODEPhysics');
            odeConfig_msg.AutoDisableBodies = phy.DisableBodies;
            odeConfig_msg.SorPgsPreconIters = phy.PreconditioningIterations;
            odeConfig_msg.SorPgsIters = phy.InnerIterations;
            odeConfig_msg.SorPgsW = phy.Relaxation;
            odeConfig_msg.SorPgsRmsErrorTol = phy.ErrorToleranceRMS;
            
            odeConfig_msg.ContactSurfaceLayer = phy.ContactWidth;
            odeConfig_msg.ContactMaxCorrectingVel = phy.MaxCorrectingVelocity;
            
            odeConfig_msg.Cfm = phy.ConstantForceMixing;
            odeConfig_msg.Erp = phy.ErrorReductionParameter;
            odeConfig_msg.MaxContacts = phy.MaxContacts;
            
            serviceMsg.OdeConfig = odeConfig_msg;
            call(obj.SetPhyClient,serviceMsg);
        end
        
        
        function out = getSpawnedModels(obj)
            %getSpawnedModels - Returns a list of all spawned models
            %   All models that are currently present in the Gazebo
            %   simulation will be returned.
            
            serviceMsg = rosmessage('gazebo_msgs/GetWorldProperties');
            msg = call(obj.ModelListClient, serviceMsg);
            
            out = msg.ModelNames;
        end
        
        function varargout = spawnModel(obj, buildModel, varargin)
            %spawnModel - Creates a new model in the simulation
            
            % Set model name and type
            serviceMsg = rosmessage(obj.SpawnClient);
            s = getModelXml(buildModel);
            serviceMsg.ModelXml = s;
            try
                model = buildModel.ModelObj.getElementsByTagName('model');
                modName = char(model.item(0).getAttribute('name'));
                
            catch
                modName = 'sample';
            end
            
            list = getSpawnedModels(obj);
            
            try
                if isequal(class(list{2}),'double')
                    list(2) = [];
                end
            catch
            end
            
            objnum = 0;
            modName = varargin{3};
%             while(1)
%                 
%                 
%                 ind = find(ismember(list,modName));
%                 
%                 if (ind)
%                     modName = [Name '_' num2str(objnum)];
%                     objnum = objnum+1;
%                 else
%                     break
%                 end
%             end
            
            serviceMsg.ModelName = modName;
            
            % Default values for position and orientation
            orientation = [1 0 0 0];
            position = [0 0 2];
            
            % Set position and orientation
            %if nargin == 3;
                position = varargin{1};
            %end
            
            %if nargin ==4
                if ~isempty(varargin{1})
                    position = varargin{1};
                end
                orientation = varargin{2};
                orientation = eul2quat([orientation(3),orientation(2),orientation(1)]);
           % end
            
            pose = rosmessage('geometry_msgs/Pose');
            point = pose.Position;
            point.X = position(1);
            point.Y = position(2);
            point.Z = position(3);
            
            orient = pose.Orientation;
            
            orient.W = orientation(1);
            orient.X = orientation(2);
            orient.Y = orientation(3);
            orient.Z = orientation(4);
            
            pose.Position = point;
            pose.Orientation = orient;
            
            serviceMsg.InitialPose = pose;
            
            msg = call(obj.SpawnClient, serviceMsg);
            
            if ~obj.IsModelServicesRunning
                startModelServices(obj);
                obj.IsModelServicesRunning = 1;
            end
            if ~msg.Success
                error('Error in spawning model')
            end
            
            % Get handle if assigned an output
            if nargout ==1
                varargout{1} = ExampleHelperGazeboSpawnedModel(modName,obj);
            end
        end
        
        function removeModel(obj, modName)
            %removeModel - Removes a specified model from the simulation
            
            serviceMsg = rosmessage(obj.RemoveModClient);
            serviceMsg.ModelName = modName;
            msg = call(obj.RemoveModClient,serviceMsg);
            if ~msg.Success
                error('Service call to remove model failed')
            end
        end
    end
    
    methods
        % Dependent properties getter and setter methods
        function out = get.Physics(obj)
            out = readPhysics(obj);
        end
        
        function out = get.ModelsList(obj)
            out = getSpawnedModels(obj);
        end
    end
    
    methods (Access = ?ExampleHelperGazeboSpawnedModel_corrected)
        
        function startModelServices(obj)
            %startModelServices - Starts all model-specific Service Clients
            
            obj.BodyForceClient = rossvcclient('gazebo/apply_body_wrench');
            obj.JointTorqueClient = rossvcclient('gazebo/apply_joint_effort');
            obj.GetModPropClient = rossvcclient('gazebo/get_model_properties');
            obj.GetModStateClient = rossvcclient('gazebo/get_model_state');
            obj.SetModStateClient = rossvcclient('gazebo/set_model_state');
            obj.SetModConfigClient = rossvcclient('gazebo/set_model_configuration');
            
        end
    end
    
    methods (Access = private)
        
        function initServiceClients(obj)
            %initServiceClients - Starts all gazebo world specific Service Clients
            
            obj.PausePhyClient = rossvcclient('gazebo/pause_physics');
            obj.ResumePhyClient = rossvcclient('gazebo/unpause_physics');
            obj.ResetSimClient = rossvcclient('gazebo/reset_simulation');
            obj.ResetWorldClient = rossvcclient('gazebo/reset_world');
            obj.ReadPhyClient = rossvcclient('gazebo/get_physics_properties');
            obj.SpawnClient = rossvcclient('gazebo/spawn_sdf_model');
            obj.SetPhyClient = rossvcclient('gazebo/set_physics_properties');
            obj.ModelListClient = rossvcclient('gazebo/get_world_properties');
            obj.RemoveModClient = rossvcclient('gazebo/delete_model');
        end
        
    end
    
end
