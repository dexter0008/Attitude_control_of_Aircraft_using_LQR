
clear all;
close all;

% A and B matrices for Longitudinal Dynamics

AL=[-0.83705 1.7696 -0.35236 0; -5.9575 -21.766 0.0056738 0.8717; 0 0 0 1; 14.891 -47.637 -0.015802 -7.9269];
BL=[0 3.9397; -0.91092 0; 0 0; -30.902 -6.9048];
CL=[0 1 0 0;0 0 1 0];
DL=zeros(4,2);

%Eigen value of AL
disp('Eigen Values of AL are')
eig(AL)

%Controllability and Observability of a matrix
TL=ctrb(AL,BL);  %Controllability matrix

if rank(AL)==rank(TL)
    disp('System is Controllable')
else
    disp('System is not Controllable')
end

SL=obsv(AL,CL);  %Observability matrix

if rank(AL)==rank(SL)
    disp('System is Observable')
else
    disp('System is not Observable')
end


%LQR for augmented state feedback
AL_hat=[AL zeros(4,2);-CL zeros(2,2)];
BL_hat=[BL;zeros(2,2)];

QL=[1000 0 0 0 0 0; 0 1000 0 0 0 0; 0 0 1000 0 0 0; 0 0 0 50 0 0; 0 0 0 0 2050 0;0 0 0 0 0 35000 ];
RL=[0.07 0; 0 0.07];

[KL_hat,PL,CLP]=lqr(AL_hat,BL_hat,QL,RL)

KL=[KL_hat(:,1) KL_hat(:,2) KL_hat(:,3) KL_hat(:,4)];
kli=[KL_hat(:,5) KL_hat(:,6)];

