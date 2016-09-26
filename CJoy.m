classdef CJoy<handle
    %CCAMERA Summary of this class goes here
    %   Detailed explanation goes here
    properties
        % physical properties
        X_VEL_AXIS      = 5;
        Y_VEL_AXIS      = 4;
        Z_VEL_AXIS      = 2;
        YAW_SPEED_AXIS  = 1;
        START_BUTTON    = 1;
        LAND_BUTTON     = 2;
        AUTO_BUTTON     = 4;
        MANUAL_BUTTON   = 3;
        START_BUTTOM    = 8
        
        Joy_sub
        
        
        vel = [0, 0, 0]';
        
        yawSpeed = 0;
        land = 0
        start = 0
        auto = 0
        manual = 1
        mpc = 0
    end
    
    methods
        
        function obj=CJoy()
            obj.Joy_sub = rossubscriber('/joy','sensor_msgs/Joy',@obj.Joycallback);
            
            
        end
        
        function Joycallback(obj,data,data1)
            obj.vel = [data1.Axes(obj.X_VEL_AXIS);
                data1.Axes(obj.Y_VEL_AXIS);
                data1.Axes(obj.Z_VEL_AXIS)];
            obj.yawSpeed = data1.Axes(obj.YAW_SPEED_AXIS);
            if data1.Buttons(obj.START_BUTTON) == 1
                obj.start = 1;
                obj.land = 0;
                obj.auto = 0;
                obj.manual = 1;
                obj.mpc = 0;
            end
            
            if data1.Buttons(obj.LAND_BUTTON) == 1
                obj.start = 0;
                obj.land = 1;
                obj.auto = 0;
                obj.manual = 1;
                 obj.mpc = 0;
            end
            
            
            if data1.Buttons(obj.AUTO_BUTTON) == 1
                obj.auto = 1;
                obj.manual = 0;
                 obj.mpc = 0;
            end
            
            if data1.Buttons(obj.MANUAL_BUTTON) == 1
                obj.auto = 0;
                obj.manual = 1;
                 obj.mpc = 0;
            end
            
             if data1.Buttons(obj.START_BUTTOM) == 1
                obj.auto = 0;
                obj.manual = 0;
                obj.mpc = 1;
            end
            
            % if data1
            
        end
        
        function [vel,yawSpeed] = giveVel(obj)
            vel = obj.vel;
            yawSpeed = obj.yawSpeed;
        end
        
        function setDroneState(obj,Drone)
            if obj.start == 1
                if Drone.m_flying == 0
                    Drone.takeoff();
                    Drone.m_flying = 1;
                    Drone.m_auto = 0;
                    Drone.m_manual = 1;
                    Drone.m_mpc = 0;
                end
                obj.start = 0;
            end
            
            if obj.land == 1
                if Drone.m_flying == 1
                    Drone.land();
                    Drone.m_flying = 0;
                    Drone.m_auto = 0;
                    Drone.m_manual = 1;
                    Drone.m_mpc = 0;
                end
                obj.land = 0;
            end
            
            if obj.auto == 1
                Drone.m_auto = 1;
                Drone.m_manual = 0;
                Drone.m_mpc = 0;
            end
            
            if obj.manual == 1
                Drone.m_auto = 0;
                Drone.m_manual = 1;
                Drone.m_mpc = 0;
            end
             if obj.mpc == 1
                Drone.m_auto = 0;
                Drone.m_manual = 0;
                Drone.m_mpc = 1;
             end
   
            
        end
    end
end
