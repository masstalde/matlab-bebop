classdef CDroneRT<handle
    %CCAMERA Summary of this class goes here
    %   Detailed explanation goes here
    properties
        % physical properties
        pos_d
        vel_d
        
        id
        vel
        pos
        q
        R_bw
        
        %
        Target
        
        % visualization
        
        simulation
        m_flying = 0;
        m_flying_last = 0;
        m_takeoff
        
        
        m_mpc = 0;
        m_auto = 0;
        m_manual =0;
        
        takeoff_pub
        takeoff_msg
        
        land_pub
        land_msg
        
        reset_pub
        reset_msg
        
        setp_vel_pub
        setp_vel_msg
        
        setp_pos_pub
        setp_pos_msg
        
        setp_mot_pub
        setp_mot_msg
        
        gimbal_pub
        gimbal_msg
        
        img_sub
        img_info_sub
        
        pos_sub
        pos_msg
        
        h_img
        h_bb
        
        lifestream
        
        tf
        
        estimator
        controller
    end
    
    methods
        
        function obj=CDroneRT(quadID,h_img,lifestream,simulation)
            obj.id=quadID;
            
            obj.simulation=simulation;
            obj.m_auto=0;
            obj.m_flying=0;
            obj.m_takeoff=0;
            obj.m_mpc = 0;
            
            obj.vel_d = [0;0;0];
            
            obj.lifestream=lifestream;
            obj.h_img= h_img;
            
            if obj.simulation==1
                %% define all the ros subscribers and publishers
                obj.takeoff_pub = rospublisher(['/q',num2str(quadID),'/bebop/takeoff'], 'std_msgs/Empty','IsLatching', false);
                obj.takeoff_msg = rosmessage(obj.takeoff_pub);
                
                obj.land_pub = rospublisher(['/q',num2str(quadID),'/bebop/land'], 'std_msgs/Empty','IsLatching', false);
                obj.land_msg = rosmessage(obj.land_pub);
                
                obj.setp_vel_pub = rospublisher(['/q',num2str(quadID),'/cmd_vel'], 'geometry_msgs/Twist','IsLatching', false);
                obj.setp_vel_msg = rosmessage(rostype.geometry_msgs_Twist);
                
                obj.setp_pos_pub = rospublisher(['/q',num2str(quadID),'/bebop/cmd_pos'], 'nav_msgs/Odometry','IsLatching', false);
                obj.setp_pos_msg = rosmessage(rostype.nav_msgs_Odometry);
                
                obj.gimbal_pub = rospublisher(['/q',num2str(quadID),'/bebop/camera_control'], 'geometry_msgs/Twist','IsLatching', false);
                obj.gimbal_msg = rosmessage(rostype.geometry_msgs_Twist);
                
                %obj.tf = RosTransformListener('world','q1\base_link');
                if obj.lifestream==1
                    %obj.img_sub = rossubscriber(['/q',num2str(quadID),'/front_cam/camera/image/compressed'],'sensor_msgs/CompressedImage', 'BufferSize', 1);
                    %obj.img_sub = rossubscriber(['/q',num2str(quadID),'/front_cam/camera/image'],'sensor_msgs/Image',@(h,e)obj.Image_callback2(), 'BufferSize', 1);
                     obj.img_sub = rossubscriber(['/q',num2str(quadID),'/front_cam/camera/image'],'sensor_msgs/Image', 'BufferSize', 1);
                    obj.img_info_sub = rossubscriber(['/q',num2str(quadID),'/real/camera_info'],'sensor_msgs/CameraInfo', 'BufferSize', 1);
                    I=readImage(obj.img_sub.LatestMessage);
                    h_img =  subimage(I);
                    uistack(h_img, 'bottom')
                    obj.h_img= h_img;
                end
                obj.pos_sub = rossubscriber(['/q',num2str(quadID),'/ground_truth/state'],'nav_msgs/Odometry', 'BufferSize', 1);
                
                
                
            else
                %% define all the ros subscribers and publishers
                obj.takeoff_pub = rospublisher(['/q',num2str(quadID),'/real/takeoff'], 'std_msgs/Empty','IsLatching', false);
                obj.takeoff_msg = rosmessage(obj.takeoff_pub);
                
                obj.land_pub = rospublisher(['/q',num2str(quadID),'/real/land'], 'std_msgs/Empty','IsLatching', false);
                obj.land_msg = rosmessage(obj.land_pub);
                
                obj.setp_vel_pub = rospublisher(['/q',num2str(quadID),'/real/cmd_vel'], 'geometry_msgs/Twist','IsLatching', false);
                obj.setp_vel_msg = rosmessage(rostype.geometry_msgs_Twist);
                
                obj.setp_pos_pub = rospublisher(['/q',num2str(quadID),'/real/cmd_pos'], 'nav_msgs/Odometry','IsLatching', false);
                obj.setp_pos_msg = rosmessage(rostype.nav_msgs_Odometry);
                
                obj.gimbal_pub = rospublisher(['/q',num2str(quadID),'/real/camera_control'], 'geometry_msgs/Twist','IsLatching', false);
                obj.gimbal_msg = rosmessage(rostype.geometry_msgs_Twist);
                
                %obj.tf = RosTransformListener('world','q1\base_link');
                if obj.lifestream==1
                    %obj.img_sub = rossubscriber(['/q',num2str(quadID),'/front_cam/camera/image/compressed'],'sensor_msgs/CompressedImage', 'BufferSize', 1);
                   % obj.img_sub = rossubscriber(['/q',num2str(quadID),'/real/image_raw'],'sensor_msgs/Image',@(h,e)obj.Image_callback2(), 'BufferSize', 1);
                    %obj.img_sub = rossubscriber(['/q',num2str(quadID),'/real/image_raw/compressed'],'sensor_msgs/CompressedImage',@(h,e)obj.Image_callback2(), 'BufferSize', 1);
                     obj.img_sub = rossubscriber(['/q',num2str(quadID),'/real/image_raw'],'sensor_msgs/Image', 'BufferSize', 1);
                    obj.img_info_sub = rossubscriber(['/q',num2str(quadID),'/real/front_cam/camera/camera_info'],'sensor_msgs/CameraInfo', 'BufferSize', 1);
                   try
                    I=readImage(obj.img_sub.LatestMessage);
                    pause(2)
                    h_img =  subimage(I);
                    uistack(h_img, 'bottom')
                    obj.h_img= h_img;
                   catch
                   end
                end
                name = 'vicon/Bebop2_Lukas/Bebop2_Lukas';
                obj.tf = RosTransformListener('world', 'vicon/Bebop2_Lukas/Bebop2_Lukas');
                obj.pos_sub = rossubscriber(name,'geometry_msgs/TransformStamped');
                %
            end
            
            
        end
        
        %         function SetVeld(obj,vel_d_linear, vel_d_angular)
        %
        %
        %             obj.setp_vel_msg.Twist.Linear.X=vel_d_linear(1);
        %             obj.setp_vel_msg.Twist.Linear.Y=vel_d_linear(2);
        %             obj.setp_vel_msg.Twist.Linear.Z=vel_d_linear(3);
        %             obj.setp_vel_msg.Twist.Angular.Z=vel_d_angular(3);
        %             obj.setp_vel_pub.send(obj.setp_vel_msg);
        %         end
        function step(obj,Joy,vel_MPC,yawspeed)
            
            [pos_filt,vel_filt,q]=obj.getPoseAndVel();
            obj.q = q;
            if obj.m_auto
                vel_d = obj.controller.step(pos_filt,vel_filt,[0;0;1]);
                vel_d(1:2) = -vel_d(1:2)
                velYaw = Joy.yawSpeed;
            end
            if obj.m_manual
                vel_d = Joy.vel
               velYaw= Joy.yawSpeed;
            end
            
            if obj.m_mpc
                vel_d = vel_MPC ;
                velYaw = yawspeed;
            end
            
            obj.SetVelEarth(vel_d, [0;0;velYaw])
            
            
          
        end
        function SetVel(obj,vel_d_linear, vel_d_angular)
            
            
            obj.setp_vel_msg.Linear.X=vel_d_linear(1);
            obj.setp_vel_msg.Linear.Y=vel_d_linear(2);
            obj.setp_vel_msg.Linear.Z=vel_d_linear(3);
            obj.setp_vel_msg.Angular.Z=vel_d_angular(3);
            obj.setp_vel_pub.send(obj.setp_vel_msg);
                       
        end
         
        
        function SetVelEarth(obj,vel_d_linear, vel_d_angular)
            [~,obj.q]=getPose(obj);
            R_cw_real = RotFromQuatJ(obj.q);
            %msg=Joy_sub.LatestMessage;
            
            yaw_real        = atan2(R_cw_real(2,1),R_cw_real(1,1));
            
            quaternion_tmp  = angle2quat(-yaw_real,0,0);
            quat            = [quaternion_tmp(2:4),quaternion_tmp(1)]';
            R_test          = RotFromQuatJ(quat);
            
            velRotated = R_test*vel_d_linear;
            obj.setp_vel_msg.Linear.X=velRotated(1);
            obj.setp_vel_msg.Linear.Y=velRotated(2);
            obj.setp_vel_msg.Linear.Z=velRotated(3);
            obj.setp_vel_msg.Angular.Z=vel_d_angular(3);
            obj.setp_vel_pub.send(obj.setp_vel_msg);
        end
        
        function SetPosAndVel(obj,pos_d,vel_d_linear, vel_d_angular)
            obj.SetPos(pos_d);
            obj.SetVel_stamped(vel_d_linear, vel_d_angular);
        end
        
        
        
        
        function SetPos(obj,pos_d)
            
            obj.setp_pos_msg.Header.Stamp=rostime('now');
            obj.setp_pos_msg.Pose.Pose.Position.X=pos_d(1);
            obj.setp_pos_msg.Pose.Pose.Position.Y=pos_d(2);
            obj.setp_pos_msg.Pose.Pose.Position.Z=pos_d(3);
            
            obj.setp_pos_pub.send(obj.setp_pos_msg);
        end
        
        function SetGimbal(obj,pitch,yaw)
            
            
            obj.gimbal_msg.Angular.Y = pitch;
            obj.gimbal_msg.Angular.Z = yaw;
            
            obj.gimbal_pub.send(obj.gimbal_msg);
        end
        
        
        function takeoff(obj)
            if obj.simulation==0
                obj.takeoff_pub.send(obj.takeoff_msg);
                 obj.m_flying = 1;
            else
                while obj.m_flying==0
                    [pos,~]=obj.getPose();
                    hight = pos(3);
                    if hight<=1
                        vel_d_linear(1)=0;
                        vel_d_linear(2)=0;
                        vel_d_linear(3)=1;
                        
                        vel_d_angular(1)=0;
                        vel_d_angular(2)=0;
                        vel_d_angular(3)=0;
                    else
                        obj.m_flying=1;
                        vel_d_linear(1)=0;
                        vel_d_linear(2)=0;
                        vel_d_linear(3)=0;
                        
                        vel_d_angular(1)=0;
                        vel_d_angular(2)=0;
                        vel_d_angular(3)=0;
                    end
                    obj.SetVel(vel_d_linear,vel_d_angular)
                end
            end
        end
        
        
        function land(obj)
            if obj.simulation==0
                obj.land_pub.send(obj.land_msg);
                  obj.m_flying = 0;
                
            else
                while obj.m_flying==1
                    [pos,~]=obj.getPose();
                    hight = pos(3);
                    if hight>0.2
                        vel_d_linear(1)=0;
                        vel_d_linear(2)=0;
                        vel_d_linear(3)=-1;
                        
                        vel_d_angular(1)=0;
                        vel_d_angular(2)=0;
                        vel_d_angular(3)=0;
                        
                    else
                        pause(0.1)
                        
                        obj.m_flying=0;
                        
                        vel_d_linear(1)=0;
                        vel_d_linear(2)=0;
                        vel_d_linear(3)=0;
                        
                        vel_d_angular(1)=0;
                        vel_d_angular(2)=0;
                        vel_d_angular(3)=0;
                        obj.SetVel(vel_d_linear,vel_d_angular)
                    end
                    obj.SetVel(vel_d_linear,vel_d_angular)
                    
                end
            end
        end
        function [pos,q]=getPose(obj)
            if obj.simulation==0
                [pos, q, time] = obj.tf.getPose();
            else
                msg=obj.pos_sub.LatestMessage;
                
                pos=[0;0;0];
                pos(1)=msg.Pose.Pose.Position.X;
                pos(2)=msg.Pose.Pose.Position.Y;
                pos(3)=msg.Pose.Pose.Position.Z;
                
                
                qx=msg.Pose.Pose.Orientation.X;
                qy=msg.Pose.Pose.Orientation.Y;
                qz=msg.Pose.Pose.Orientation.Z;
                qw=msg.Pose.Pose.Orientation.W;
                q=[qx;qy;qz;qw];
            end
            obj.pos = pos;
            
        end
        
        function [pos_filt,vel_filt,q]=getPoseAndVel(obj)
            if obj.simulation==0
                %[pos, q, time] = obj.tf.getPose();
                msg = obj.pos_sub.LatestMessage;
                pos = [ msg.Transform.Translation.X;
                    msg.Transform.Translation.Y;
                    msg.Transform.Translation.Z];
                
                    q = [ msg.Transform.Rotation.X;
                    msg.Transform.Rotation.Y;
                    msg.Transform.Rotation.Z;
                    msg.Transform.Rotation.W];
            else
                msg=obj.pos_sub.LatestMessage;
                
                pos=[0;0;0];
                pos(1)=msg.Pose.Pose.Position.X;
                pos(2)=msg.Pose.Pose.Position.Y;
                pos(3)=msg.Pose.Pose.Position.Z;
                
                
                qx=msg.Pose.Pose.Orientation.X;
                qy=msg.Pose.Pose.Orientation.Y;
                qz=msg.Pose.Pose.Orientation.Z;
                qw=msg.Pose.Pose.Orientation.W;
                q=[qx;qy;qz;qw];
            end
            [pos_filt,vel_filt] = obj.estimator.step(pos);
            obj.pos = pos_filt;
            obj.vel = vel_filt;
        end
        
        
        
        function I=getFrontImage(obj)
            
            I=readImage(obj.img_sub.LatestMessage);
            
        end
        
        function I=plotFrontImage(obj)
            imgmsg = obj.img_sub.LatestMessage;
            
                try
                I=readImage(imgmsg);
                obj.h_img.CData=I;
                
            catch
                
            end
        end
        
        
        
        function setAutoMode(obj)
            obj.m_auto=1;
        end
        
        
        function setManualMode(obj)
            obj.m_auto=0;
        end
        
        function  Image_callback(obj)
            
            I=obj.getFrontImage();
            % img = readImage(callbackdata);
            
            try
                
                obj.h_img.CData=I;
                
            catch
                
            end
            
        end
        
        function  Image_callback2(u,~)
            
            I=u.getFrontImage();
            % img = readImage(callbackdata);
            
            try
                
                u.h_img.CData=I;
                
            catch
                
            end
            
        end
        
        function K = getCameraParameter(obj)
            
            msg=obj.img_info_sub.LatestMessage;
            
            K=reshape(msg.K,[],3)';
            
            
            
        end
        
        
        
    end
end
