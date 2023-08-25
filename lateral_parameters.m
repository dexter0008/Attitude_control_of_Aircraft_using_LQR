clc;
clear all;
close all;

%A and B matrices for Lateral Dyamics

A=[-2.3817 0 -1.0019 2.1827; -21.063 -16.055 0.87229 0; 24.512 -16.651 -3.5379 0; 0 1.0026 -0.029766 0];
B=[0 -0.24719; -36.263 -688.44; -0.6725 -67.983; 0 0];
B1=[0.0869 0;4.424 1.184; 0 -1; -2.148 0.021]; %input vector for disturbance
C=[1 0 0 0; 0 0 0 1];
D=zeros(4,2);


%Eigen values of A
disp('Eigen Values of A are:');
eig(A)


%Controllability and Observability of a matrix
P=ctrb(A,B);  %Controllability matrix

if rank(A)==rank(P)
    disp('System is Controllable')
else
    disp('System is not Controllable')
end

S=obsv(A,C);  %Observability matrix

if rank(A)==rank(S)
    disp('System is Observable')
else
    disp('System is not Observable')
end


%LQR for augmented state feedback
A_hat=[A zeros(4,2);-C zeros(2,2)];
B_hat=[B;zeros(2,2)];

Q=[0.01 0 0 0 0 0; 0 0.001 0 0 0 0; 0 0 0.001 0 0 0; 0 0 0 70 0 0; 0 0 0 0 10 0;0 0 0 0 0 7000 ];
R=[.02 0; 0 .02];

[K_hat,P,CLP]=lqr(A_hat,B_hat,Q,R)

K=[K_hat(:,1) K_hat(:,2) K_hat(:,3) K_hat(:,4)];
ki=[K_hat(:,5) K_hat(:,6)];
