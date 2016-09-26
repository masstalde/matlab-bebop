



ROS_MASTER_IP   = 'localhost';
ROS_IP          = 'localhost';                %'192.168.1.3';
setenv('ROS_MASTER_URI', ['http://',ROS_MASTER_IP,':11311']);
setenv('ROS_IP', ROS_IP);
try
    rosinit;
    
catch
    display('Ros is already started');
    rosshutdown
    rosinit;
end
dt = 1/20;
Joy = CJoy();

h2d_subplot=subplot(1,1,1);

    Quadrotor = CDroneRT(1,h2d_subplot,0,0);
    Quadrotor.estimator = kalmanTarget(dt);
    Quadrotor.controller = QuadLQR(dt);


%Joy_sub = rossubscriber('/joy','sensor_msgs/Joy', 'BufferSize', 1);

%Quadrotor.takeoff();
%Quadrotor(2).takeoff();
%pause(1)

%Quadrotor.land();
%Quadrotor(2).land();

vel_log=zeros(10,10000);
    while(true)
        Joy.setDroneState(Quadrotor);
        Quadrotor.step(Joy,[0;0;0]);
%        Quadrotor.m_auto
%        
%        vel_log(:,i)=[Joy.vel;Quadrotor.vel;Quadrotor.q];
%        Quadrotor.SetVelEarth(Joy.vel,[0;0;Joy.yawSpeed]);
%        Quadrotor.SetVel(Joy.vel,[0;0;Joy.yawSpeed]);
        pause(0.05)
        display('running')
    end