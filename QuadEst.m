%% This simulator is based on the formulas written in the paper: Minimum Snap Trajectory Generation and Controll for Quadrotors.
% Based on structural code from Tobias Naegeli.
classdef QuadEst<handle
    %QUADSIM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %% physical properties
        pos
        vel
        acc
        rate
        R_BW
        mass
        dt
        air_res
        quad_arm_length
        J_inertia
        pos_indices = 1:3;
        vel_indices = 4:6;
        acc_indices = 7:9;
        angular_vel_indices = 10:12;
        nStates = 12;
        rotorSpeeds
        state =[0;0;0;0;0;0;0;0;0];
        handle
        arm=0;
        % camera
        camera
        
        %% graphical properties
        visQuad
        txtMode
    end
    
    methods
        function obj= QuadEst(handle)
            function points= plotCircle3D(handle,center,normal,radius)
                
                theta=0:0.1:2*pi;
                v=null(normal);
                points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
                
                % plot3(points(1,:),points(2,:),points(3,:),'r-');
                
            end
            % constructor
            obj.handle = handle;
            camera = [];
            state0 = [];
            
            R_BW0 = [];
            mass=0.5; %mass
            dt=1/200; %sample time
            air_res=0.05*0;
            quad_arm_length=0.175;
            
            
            %d=0.175;
            d_accu=0.005;
            radius_accu=0.003;
            mass=0.443;
            m_accu=0.167;
            m_mot=0.03;
            J_11=4*m_mot*(sin(pi/4)*quad_arm_length)^2+m_accu*d_accu^2;
            J_22=4*m_mot*(sin(pi/4)*quad_arm_length)^2+m_accu*d_accu^2;
            J_33=4*m_mot*quad_arm_length^2+2/3*m_accu*radius_accu^2;
            J_inertia=[ J_11,0,0;
                0,J_22,0;
                0,0,J_33];
            
            options = struct();
            
            plot_quadrotor = true;
            
            
            obj.mass=mass; %mass
            obj.dt=dt; %sample time
            obj.air_res=air_res;
            obj.quad_arm_length=quad_arm_length;
            obj.J_inertia=J_inertia;
            
            obj.set_state(state0, R_BW0);
            
            % camera
            obj.camera=camera;
            
            %% plot quadrotor
            if plot_quadrotor
                
                rotors=4;
                %% plotting the quadrotor with 4 rotors
                diameter=0.25;
                obj.visQuad.diameter=diameter;  % diameter of the quadrotor beams
                rotCenter(1,:)=diameter*[-1,1,0];
                rotCenter(2,:)=diameter*[1,1,0];
                rotCenter(3,:)=diameter*[1,-1,0];
                rotCenter(4,:)=diameter*[-1,-1,0];
                obj.visQuad.beam_1=line( obj.handle,[rotCenter(1,1),rotCenter(3,1)],[rotCenter(1,2),rotCenter(3,2)],[rotCenter(1,3),rotCenter(3,3)]);
                obj.visQuad.beam_2=line( obj.handle,[rotCenter(2,1),rotCenter(4,1)],[rotCenter(2,2),rotCenter(4,2)],[rotCenter(2,3),rotCenter(4,3)]);
                hold on
                for i=1:rotors
                    rotor_points= plotCircle3D( obj.handle,rotCenter(i,:),[0,0,1],0.1);
                    obj.visQuad.propeller(i)=plot3( obj.handle,rotor_points(1,:),rotor_points(2,:),rotor_points(3,:),'r-');
                end
                
                obj.txtMode=text( obj.handle,0,0,1,'Armed');
            end
            
        end
        
        function set_state(obj, state0, R_BW0)
            if nargin < 2 || isempty(state0)
                % [p;v;a;w] = initial state
                tmppos=[0;0;10]; tmpvel=[0;0;0]; tmpacc=[0;0;0]; tmprate=[0;0;0];
            end
            if nargin < 3 || isempty(R_BW0)
                R_BW0 = eye(3);
            end
            obj.pos = tmppos;
            obj.vel = tmpvel;
            obj.acc = tmpacc;
            obj.rate = tmprate; % body angular rate
            obj.R_BW = R_BW0;
        end
        
        function [state, R_BW] = get_state(obj)
            state.pos=obj.pos;
            state.vel=obj.vel;
            state.acc=obj.acc;
            state.rate=obj.rate;
            R_BW = obj.R_BW;
        end
        
        function  step(obj, u_mot)
            
            c_T=1/4;
            c_Q=1/4;
            
            Phi=[0;pi/2;pi;3*pi/2];
            %Moments acting to the frame: (phi is the angle between the propeller arm
            %and the body fixed x axis)
            
            %input: u_mot is the input of all 4 motors
            Gamma=[ c_T,        c_T,        c_T,                c_T;
                0,         c_T*obj.quad_arm_length,      0,          -c_T*obj.quad_arm_length;
                -c_T*obj.quad_arm_length,    0,          c_T*obj.quad_arm_length,      0;
                -c_Q,      c_Q,        -c_Q,       c_Q];
            
            
            ForceTorque=Gamma*u_mot.^2;
            
            Thrust=ForceTorque(1);
            Tau=ForceTorque(2:4);
            ang_a=obj.J_inertia\(-cross(obj.rate,obj.J_inertia*obj.rate)+Tau);    %angular acceleration
            
            % rotation velocity integration
            obj.rate=obj.rate+ang_a*obj.dt;
            
            % rotation integration
            dR = expm(-0.5 * skew(obj.rate));
            obj.R_BW = obj.R_BW * dR;
            
            %Position
            obj.acc=obj.R_BW*[0;0;Thrust/obj.mass]+[0;0;-9.81]+obj.air_res*obj.vel.^2;
            obj.vel = obj.vel + obj.acc*obj.dt;
            obj.pos = obj.pos + obj.vel*obj.dt + 1/2*obj.acc*obj.dt^2;
            
            % obj.R_BW = R_new;
            
        end
        
        
        %% controlling

        function velocityControl(obj,vel_d)
            F_des = 0.5*(vel_d - obj.vel);
            R_B=R_cw;
            phi=atan2(R_B(2,3),R_B(3,3));
            theta=-asin(R_B(1,3));
            psi=atan2(R_B(1,2),R_B(1,1));
            R_c = angle2dcm(0, 0, psi, 'XYZ');
            F_des_f=R_c'*F_des_c;
            [rates_des,thrust] = attitude_controller(obj,R_B,yaw,F_des,Kp,kumar_variant)
        end
        % first the Attitude Control
        %This controller is on the Quadrotor
        function [rates_des,thrust] = attitude_controller(obj,R_B,yaw,F_des,Kp,kumar_variant)
            
            R_B=R_B';
            
            R_C = angle2dcm(0, 0, yaw, 'XYZ');
            x_B = R_B(1, :);
            x_C = R_C(1, :);
            z_B_des = F_des / norm(F_des);
            x_C_des = [cos(yaw); sin(yaw); 0];
            y_C_des = [cos(yaw + pi/2); sin(yaw + pi/2); 0];
            
            
            if kumar_variant
                y_B_des = cross(z_B_des, x_C_des) / norm(cross(z_B_des, x_C_des));
                x_B_des = cross(y_B_des, z_B_des);
            else
                x_B_des = cross(y_C_des, z_B_des) / norm(cross(y_C_des, z_B_des));
                y_B_des = cross(z_B_des, x_B_des) / norm(cross(z_B_des, x_B_des));
            end
            
            
            R_des1 = [x_B_des, y_B_des, z_B_des];
            r1 = vrrotmat2vec(R_des1' * R_B);
            
            R_des2 = [-x_B_des, -y_B_des, z_B_des];
            r2 = vrrotmat2vec(R_des2' * R_B);
            
            angle_err1 = r1(4);
            angle_err2 = r2(4);
            if angle_err1 <= angle_err2
                R_des = R_des1;
            else
                R_des = R_des2;
            end
            e_R = 0.5 * (R_des' * R_B - R_B' * R_des);
            
            
            
            rates_des=Kp*[e_R(3,2);e_R(1,3);e_R(2,1)];
            
            
            thrust = norm(F_des);
            
        end
        
        function [u_mot] = rate_controller_pid(obj,rates_des,ang_v,K_rate,d,Thrust)
            %% PID controller
            % second the body angular Rate Control
            %This controller is on the Quadrotor
            c_T=1/4;
            
            c_Q=1/4;
            
            Phi=[0;pi/2;pi;3*pi/2];
            %ang_v=state(7:9);
            e_rate=rates_des-ang_v;
            ForceTorque(1)=Thrust;
            ForceTorque(2:4)=K_rate*e_rate;
            
            %% Mixe
            Gamma=[ c_T,        c_T,        c_T,                c_T;
                0,         c_T*d,      0,          -c_T*d;
                -c_T*d,    0,          c_T*d,      0;
                -c_Q,      c_Q,        -c_Q,       c_Q];
            u_mot=sqrt(Gamma\ForceTorque');
            
        end
        
        %% Statemachine Stuff
         function receive(obj,msg)
             if msg.arm==0
                 obj.arm=0;
                obj.rotorSpeeds=[0;0;0;0];
             else
                 obj.arm=1;
             end
             
        end
        %% visualization
        function updateVis(obj,R,pos)
            if nargin<2
                R=obj.R_BW;
                pos=obj.pos;
            end
            if obj.arm==1
            set(obj.txtMode,'position',pos+[0.1;0.1;0.5]);
            set(obj.txtMode,'String','armed');
            else
            set(obj.txtMode,'position',pos+[0.1;0.1;0.5]);  
            set(obj.txtMode,'String','disarmed');
            end
            function points= plotCircle3D(center,normal,radius)
                theta=0:0.1:2*pi;
                v=null(normal);
                points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
            end
            diameter=obj.visQuad.diameter;
            rotCenter(1,:)=pos+R*diameter*[-1,1,0]';
            rotCenter(2,:)=pos+R*diameter*[1,1,0]';
            rotCenter(3,:)=pos+R*diameter*[1,-1,0]';
            rotCenter(4,:)=pos+R*diameter*[-1,-1,0]';
            set(obj.visQuad.beam_1,'XData',[rotCenter(1,1),rotCenter(3,1)],'YData',[rotCenter(1,2),rotCenter(3,2)],'ZData',[rotCenter(1,3),rotCenter(3,3)])
            set(obj.visQuad.beam_2,'XData',[rotCenter(2,1),rotCenter(4,1)],'YData',[rotCenter(2,2),rotCenter(4,2)],'ZData',[rotCenter(2,3),rotCenter(4,3)])
            for i=1:4
                rotor_points= plotCircle3D(rotCenter(i,:),R(:,3)',0.1);
                set(obj.visQuad.propeller(i),'XData',rotor_points(1,:),'YData',rotor_points(2,:),'ZData',rotor_points(3,:))
            end
        end
        
    end
    
end