



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

    Quadrotor = CDroneRT(1,'vicon/Bebop_Manuel_1/Bebop_Manuel_1', 0);
    Quadrotor.estimator = kalmanTarget(dt);
    Quadrotor.controller = QuadLQR(dt);

vel_log=zeros(10,10000);
    while(true)
        Joy.setDroneState(Quadrotor);
        Quadrotor.step(Joy,[0;0;0]);
%        
%        vel_log(:,i)=[Joy.vel;Quadrotor.vel;Quadrotor.q];
%        Quadrotor.SetVelEarth(Joy.vel,[0;0;Joy.yawSpeed]);
%        Quadrotor.SetVel(Joy.vel,[0;0;Joy.yawSpeed]);
        pause(0.05)
        display('running')
    end