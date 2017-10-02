function Vdot=diff_drive_dyn_neural(x)


persistent w1
persistent w2
persistent vv
persistent w3
persistent flag

if isempty(flag)
    flag=1;
    vv=rand(6,25);
    w1=zeros(25,25);
    w2=zeros(25,25);
    w3=zeros(25,2);
end
    

alpha1=0.01;
alpha2=0.01;
alpha3=0.01;
B_n=10*ones(25,2);

%Robot parameters
mT=10;  %Total mass
mW=2;   %Mass of 1 wheel
r=0.05; %Wheel radius
b=0.4;  %half the robot width
d=0.1;  %CG offset from rear axle
Iyy=1;  %Wheel moment of Iertia
IT=5;   %Platform total moment of inertia
Fv=.5;  %Coefficient of Fiscous friction
Fd=.8;  %Coefficient of Colomb friction

%%

%%
%Control Gains
k1=10;
k2=5;
k3=4;

K=[20 0;
   0 20];
%%
%Robot States
xc=x(1);
yc=x(2);
th=x(3);

v=x(4);
w=x(5);
V=[v;w];

%%
%Reference Robot
xd=x(6);
yd=x(7);
thd=x(8);

%Reference Velocity
vd=x(9);
wd=x(10);

vddot=x(11);
wddot=x(12);
%%
e=[cos(th) sin(th) 0;
  -sin(th)  cos(th) 0;
   0          0    1]*[xd-xc; yd-yc; thd-th];

Vd=[vd;wd];
Vddot=[vddot;wddot];

Vc=[vd*cos(e(3))+k1*e(1); wd+k2*vd*e(2)+k3*vd*sin(e(3))];

edot=[0 w 0;-w 0 0;0 0 0]*e+[vd*cos(e(3));vd*sin(e(3));wd]-[1 0;0 0;0 1]*V;

Vcdot=[vddot*cos(e(3))+vd*(-sin(e(3)))*edot(3)+k1*edot(1);
    wddot+k2*vddot*e(2)+k2*vd*edot(2)+k3*vddot*sin(e(3))+k3*vd*cos(e(3))*edot(3)];
%%
%Robot Kinematics
S=[cos(th) -d*sin(th);
   sin(th)  d*cos(th);
   0        1;
   1/r      b/r;
   1/r     -b/r];


xdot(1:5)=S*V;

phRdot=xdot(4);
phLdot=xdot(5);
%%

%dynamics
M=[mT+2*Iyy/(r^2)                   0;
    0           mT*d^2+IT+2*Iyy*(b^2)/(r^2)-4*mW*d^2];

Vm=[0       -d*w*(mT-2*mW);
    d*w*(mT-2*mW)   0];

F=(1/r)*[Fv*(phRdot+phLdot)+Fd*(sign(phRdot)+sign(phLdot));
      b*(Fv*(phRdot-phLdot)+Fd*(sign(phRdot)-sign(phLdot)))];

G=[0;0];

td=[0;0];

B=[1/r  1/r;
   b/r -b/r];


hc=[-e(1);sin(e(3))/k2];
f=-M^(-1)*(Vm*V+F+G);
%%
%Part a
% tau=B^(-1)*(-K*(V-Vc));
%%
%Part b
tau=B^(-1)*M*(-K*(V-Vc)+Vcdot+hc-f);

V_neural= [V;Vc;Vcdot];

 phi1=tanh(vv'*V_neural);
 phi2=tanh(w1'*phi1);
 phi3=phi2; % tanh(w2'*phi2);
   
tau_hat= w3'*phi3;

w_e=tau_hat-tau;
% hold on;
% plot(tau,'-');
% plot(tau_hat,'g');

    w1=w1-alpha1*phi1*(w1'*phi1+B_n*w_e)';
%     whos;
%      w2=w2-alpha2*phi2*(w2'*phi2+B_n*w_e)';
    w3=w3-alpha3*phi3*w_e';

%%
Vdot=M^(-1)*(B*tau_hat-td-G-F-Vm*V);
