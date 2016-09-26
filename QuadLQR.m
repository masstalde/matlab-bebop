classdef QuadLQR<handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        K_lqr
        K_lqri
        i_e
        
    end
    
    methods
        function obj = QuadLQR(dt)
            m = 0.7;
            qx_p = 1;
            qx_v = 0.0001;
            
            qz_p = 1;
            qz_v = 0.3;
            
            rx = 5;
            rz = 5;
            
            %% the system dynamics
            A=[0,0,0,1,0,0;
                0,0,0,0,1,0;
                0,0,0,0,0,1;
                0,0,0,0,0,0;
                0,0,0,0,0,0
                0,0,0,0,0,0];
            
            B= [0,0,0;
                0,0,0;
                0,0,0;
                1/m,0,0;
                0,1/m,0;
                0,0,1/m];
            
            C=[1,0,0,0,0,0;
                0,1,0,0,0,0;
                0,0,1,0,0,0];
            D=[];
            
            %% add the integral model
            Ai=[A,zeros(6,3);C,zeros(3)];
            Bi=[B;zeros(3)];
            Ci=[C,zeros(3)];
            sys=ss(A,B,C,D);
            sysd=c2d(sys,dt);
            sys_i=ss(Ai,Bi,Ci,D);
            sysd_i=c2d(sys,dt);
            
            Ad_i=sysd_i.A;
            Bd_i=sysd_i.B;
            
            Ad=sysd.A;
            Bd=sysd.B;
            
            Qp=[qx_p,0,0,0,0,0;
                0,qx_p,0,0,0,0;
                0,0,qz_p,0,0,0;
                0,0,0,qx_v,0,0;
                0,0,0,0,qx_v,0;
                0,0,0,0,0,qz_v];
            
            Ri=[rx,0,0;
                0,rx,0;
                0,0,rz];
            
          
            
            
            % Rn=kron(zeros(3),Rp);
            
           % Qi=blkdiag(Qp,diag([qx_i,qy_i,qz_i]));
            
           % [obj.K_lqri,~,~] = dlqr(Ad_i,Bd_i,Qi,Ri);
            [obj.K_lqr,~,~] = dlqr(Ad,Bd,Qp,Ri);
            
            
            
        end
       function [vel_d] = step(obj,pos,vel,pos_d)
         
            %% error in pos       
            e_pos_earth = (pos - pos_d);
            e_vel_eartk = (vel - [0;0;0]);
            res = [e_pos_earth;e_vel_eartk*0.5];
            vel_d = -obj.K_lqr*res;
            
%             R_bw = RotFromQuatJ(q);
%             R_updown = diag([1,1,1]);
%             phi=atan2(R_bw(2,3),R_bw(3,3));
%             theta=-asin(R_bw(1,3));
%             psi=atan2(R_bw(1,2),R_bw(1,1));
%             R_c = angle2dcm(0, 0, psi, 'XYZ');
    

          

            if abs(vel_d(1))>=2
                vel_d(1)=sign(vel_d(1))*2;
            end
            if abs(vel_d(2))>=2
                vel_d(2)=sign(vel_d(2))*2;
            end
            if abs(vel_d(3))>=2
                vel_d(3)=sign(vel_d(3))*2;
            end
           
        end
        function [pos,vel] = step_int(obj,z)
            obj.sendState();
            Kp = K_LQRp(:,1:3);
            Kd = K_LQRp(:,4:6);
            Ki = K_LQRp(:,7:9);
            %% error in pos       
            pos_d = obj.pos_Setpoint;
            e_earth = (pos_d-pos);
            
            F_des = Kp*e_earth+Kd*([0;0;0]-vel);
            
            R_bw = RotFromQuatJ(q);
            R_updown = diag([1,1,1]);
            phi=atan2(R_bw(2,3),R_bw(3,3));
            theta=-asin(R_bw(1,3));
            psi=atan2(R_bw(1,2),R_bw(1,1));
            R_c = angle2dcm(0, 0, psi, 'XYZ');
            
            if pos(3)<0.3
                intReset = 1;
            else
                intReset = 0;
            end
            IntLim = [10;10;10];
            %% integrator
            dt = 1/50;
             e_body = R_c*e_earth;
            ie = obj.ie
            
            if intReset==0
                if abs(ie(1))<IntLim(1)
                    ie(1)=ie(1)+e_body(1)*dt;
                else
                    ie(1)=ie(1);
                end
                if abs(ie(2))<IntLim(2)
                    ie(2)=ie(2)+e_body(2)*dt;
                else
                    ie(2)=ie(2);
                end
                if abs(ie(3))<IntLim(3)
                    ie(3)=ie(3)+e_body(3)*dt;
                else
                    ie(3)=ie(3);
                end
                
            else
                % If below height limit keep integrals cosntant
                ie=zeros(3,1);
            end
           obj.ie = ie;
            Fp=R_updown*R_c*F_des+[0;0;thrust] + Ki*ie*0 + Ki*F_ff;

            if abs(Fp(1))>=2
                Fp(1)=sign(Fp(1))*2;
            end
            if abs(Fp(2))>=2
                Fp(2)=sign(Fp(2))*2;
            end
            if abs(Fp(3))>=2
                Fp(3)=sign(Fp(3))*2;
            end
            obj.SendAttitude(Fp(2),Fp(1),yaw,Fp(3))
        end
        
       
    end
    
end

