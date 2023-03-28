function [r,dr,ddr] = Trajectory(t,scale,y)
    %TRAJECTORY Summary of this function goes here
    %   Detailed explanation goes here
    if ~exist('scale','var')
        scale=1/200;
    end
    if ~exist('y','var')
        y=[1;1;1];
    end


    r=zeros(2,1);dr=r;ddr=dr;%#ok

     w=t*scale;
    % r=[w;w];
    % dr=[1+0*w;1+0*w]*scale;
    % ddr=[0+0*w;0+0*w]*scale*scale;


    % forward parking
%x_i=0; y_i=0; theta_i=0;     % initial configuration
x_f=4; y_f=2; theta_f=-pi/4;  % final configuration
% % 
% parallel parking
%x_i=y(1); y_i=y(2); theta_i=y(3);     % initial configuration
%x_f=0; y_f=4; theta_f=0;  % final configuration
% % 
% pure reorientation
x_i=y(1); y_i=y(2); theta_i=y(3);     % initial configuration
%x_f=0; y_f=0; theta_f=pi/2;  % final configuration
% % 
% forward or backward motion (k>0 or <0)
k=1;   % k=ki=kf same value for initial and final geometric velocity 

%%%%%%%%%%%%%%%
% computation %
%%%%%%%%%%%%%%%

% parameters of the cubic polynomial
% % 
alfa=[k*cos(theta_f)-3*x_f; k*sin(theta_f)-3*y_f];
alfa_x=alfa(1);
alfa_y=alfa(2);
% % 
beta=[k*cos(theta_i)+3*x_i;k*sin(theta_i)+3*y_i];
beta_x=beta(1);
beta_y=beta(2);
% % 
% vector for trajectory parametrization
% % 
s=w;

% these are the interpolating polynomials
% x(s),y(s) and their first and second derivative wrt s
% % 
x=-(s-1).^3*x_i+s.^3*x_f+alfa_x*(s.^2).*(s-1)+beta_x*s.*((s-1).^2);
y=-(s-1).^3*y_i+s.^3*y_f+alfa_y*(s.^2).*(s-1)+beta_y*s.*((s-1).^2);
% % 
xp= -3*(s-1).^2 *x_i+3*s.^2 *x_f+alfa_x*(3*s.^2-2*s)+beta_x*(3*s.^2-4*s +1); 
yp= -3*(s-1).^2 *y_i+3*s.^2 *y_f+alfa_y*(3*s.^2-2*s)+beta_y*(3*s.^2-4*s +1); 
% % 
xpp= -6*(s-1)*x_i+6*s*x_f+alfa_x*(6*s-2)+beta_x*(6*s-4); 
ypp= -6*(s-1)*y_i+6*s*y_f+alfa_y*(6*s-2)+beta_y*(6*s-4); 
 
    
r=[x;y];
dr=[xp;yp];
ddr=[xpp;ypp];

end

