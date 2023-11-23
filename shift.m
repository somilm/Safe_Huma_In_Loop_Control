% function [t0, x0, u0] = shift(T, t0, x0, u,f,execution_time, xs, xo)
function [t0, x0, u0, stored_values] = shift(T, t0, x0, u, f, execution_time, xs, xo, stored_values)
st = x0;
con = u(1,:)';
pause(execution_time);
robotCmd = rospublisher("/cmd_vel","DataFormat","struct") ;
humanInputSubscriber = rossubscriber('/human_input', 'geometry_msgs/Twist');
humanInputMsg = receive(humanInputSubscriber, 1);  % Receive the latest message
linearVelocity = humanInputMsg.Linear.X;  % Access linear X component
angularVelocity = humanInputMsg.Angular.Z;  % Access angular Z component
% disp(linearVelocity);
% disp(angularVelocity);
stored_values.linearVelocity = [stored_values.linearVelocity, linearVelocity];
stored_values.angularVelocity = [stored_values.angularVelocity, angularVelocity];
stored_values.x0 = [stored_values.x0, x0];
stored_values.con = [stored_values.con, con];
%%%%%%%%%%%%%%%%%%5
% [linear_x, linear_z, U, nr, Vr] = Potencial_Field(xo, x0, xs);
%     
% stored_values.linear_x = [stored_values.linear_x, linear_x];
% stored_values.linear_z = [stored_values.linear_z, linear_z];
% stored_values.U = [stored_values.U, U];
% stored_values.nr = [stored_values.nr, nr];
% stored_values.Vr = [stored_values.Vr, Vr];
%%%%%%%%%%%%%%%%%%%%%%%%%%55
velMsg = rosmessage(robotCmd);
linear_x = 0;
linear_z = 0;
lamda  = 0.5;
beta_1 = 0.8;
beta_2 = 0.8;

if (x0(1) > -0.5 && x0(1) < 0.5) && (x0(2) > -0.5 && x0(2) < 0.5)
    
%     [linear_x, linear_z] = Potencial_Field (xo,x0,xs);
    [linear_x, linear_z, U, nr, Vr] = Potencial_Field(xo, x0, xs);
    
    stored_values.linear_x = [stored_values.linear_x, linear_x];
    stored_values.linear_z = [stored_values.linear_z, linear_z];
    stored_values.U = [stored_values.U, U];
    stored_values.nr = [stored_values.nr, nr];
    stored_values.Vr = [stored_values.Vr, Vr];
    velMsg.Linear.X = beta_1*linearVelocity +(1-beta_1)*linear_x;
    velMsg.Angular.Z = beta_1*angularVelocity +(1-beta_1)*linear_z;
    
else 
    velMsg.Linear.X = lamda*con(1) + (1-lamda)*linearVelocity;
    velMsg.Angular.Z = lamda*con(2) + (1-lamda)*angularVelocity;
end


send(robotCmd,velMsg)
disp("MPC");
disp(con);
disp("Human")
disp(linearVelocity);
disp(angularVelocity);
disp("Potencial_Field")
disp(linear_x);
disp(linear_z);
disp("final");
disp(velMsg.Linear.X);
disp(velMsg.Angular.Z);
%
SubPose1 = rossubscriber("/vicon/TB4/TB4");
PoseData1 = receive(SubPose1);
pose_T1 = PoseData1.Transform.Translation;
pose_R1 = PoseData1.Transform.Rotation;

% disp('recieved from vicon')
x1 = pose_T1.X;          % X position of the Robot
y1 = pose_T1.Y;          % Y position of the Robot
z1 = pose_T1.Z;

quat1 = pose_R1;      % Orientation of the Robot
angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]);  % Conversion from Quatarnian to Euler Angle
theta = angles1(1); % Rotation about Z axis in radian
% f_value = f(st,con);
% st = st+ (T*f_value);
x0 = [x1;y1;theta];

t0 = t0 + T;
u0 = [u(2:size(u,1),:);u(size(u,1),:)];
if isempty(u0)
    u0 = zeros(size(u));
end
end