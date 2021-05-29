function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


%Tuning parameters
kpy=100;
kpz=200;
kdy=50;
kdz=10;
kp_phi=600;
kd_phi=80;

%Initialization
y_ddot=des_state.acc(1);
z_ddot=des_state.acc(2);
ep=des_state.pos-state.pos;
ev=des_state.vel-state.vel;
roll_ddot=0;
roll_dot=0;

%control
roll=-(y_ddot+kpy*ep(1)+kdy*ev(1))/params.gravity;
u1=params.mass*(params.gravity+z_ddot+kpz*ep(2)+kdz*ev(2));
u2=params.Ixx*(roll_ddot+kp_phi*(roll-state.rot)+kd_phi*(roll_dot-state.omega(1)));

%Boundary condition
if(u1<params.minF)
    u1=params.minF;
end

if(u1>params.maxF)
    u1=params.maxF;
end




end

