function [dy] = DiffDrive(~,y,u,p)
    %DIFFDRIVE Differential Drive Robot Dynamics
    %   Detailed explanation goes here
    r=p(1); b=p(2); %Radious and 
    theta=y(3);


    dy = A_mat(y)*S_mat(r,b)*[u(1);...
              u(2)];
end

function A=A_mat(y)
    %theta==y(3)
    A=[cos(y(3)) 0;...
        sin(y(3)) 0;...
        0         1];
end
function S=S_mat(r,b)
    S=[r/2      r/2 ;...
       r/(2*b) -r/(2*b) ];
end

