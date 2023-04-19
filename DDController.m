function deu = DDController(t,eu,y,p) %#codegen
    %DDTRAJECTORY Summary of this function goes here
    %   Detailed explanation goes here
    e=eu(1:3);
    %r=p(1); b=p(2); %Radious and Axis
    % e_v=e(1);e_x=e(2);e_y=e(3);
    k=[10,80,90,0.5]';%Control Coeff
    %k(4)=1 feed forward component;
    de=depsilon(t,e,y,k);

    u=input(t,e,y,p,k);

    deu=[de;u];
end
function A=A_mat(y,e)
    %theta==y(3)
    A=[cos(y(3)) -e(1)*sin(y(3));...
        sin(y(3)) e(1)*cos(y(3))];
end
function S=S_mat(r,b)
    S=[r/2      r/2 ;...
        r/(2*b) -r/(2*b) ];
end
function [eta,rd]=eta_fun(t,e,y,k)

    kv=k(1);kp=k(2);ki=k(3);
    kf=k(4);
    e_v=e(1);% e_x=e(2);e_y=e(3);
    [rd,drd,ddrd]=Trajectory(t);

    dre=[cos(y(3))*e_v;sin(y(3))*e_v,];
    eta=kf*ddrd+kv*(drd-dre)+kp*(rd-y(1:2))+ki*e(2:3);

end
function de=depsilon(t,e,y,k)
    [eta,rd]=eta_fun(t,e,y,k);
    de=[ [1 0]*pinv(A_mat(y,e))*eta;...
        rd-y(1:2)];
end
function u=input(t,e,y,p,k)
    r=p(1); b=p(2);
    [eta,~]=eta_fun(t,e,y,k);
    h_mat=[e(1) ;[0 1]*pinv(A_mat(y,e))*eta];
    u=pinv(S_mat(r,b))*h_mat;
end
