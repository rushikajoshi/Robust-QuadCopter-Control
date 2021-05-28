function [ u ] = pd_controller(~,s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
%

%Tuning parameters
kp=200;
kd=20;

%Initial Condition
e=s_des-s;
acc=0;

%Thrust control
u=params.mass*(params.gravity+acc+kp*e(1)+kd*e(2));
acc=(u/params.mass)+params.gravity;

%Boundary Condition
if(u>params.u_max)
    u=params.u_max;
end

if(u<params.u_min)
    u=params.u_min;
end



end