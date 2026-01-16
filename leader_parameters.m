clear;
close all;


%%  System, Constraints and Disrturbances
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%if these are changed, then all the offline
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%parts must be re-computed

%% Robot modeling
r=0.04445;        % radius od wheels [m]
d= 0.393;      % distance between the two wheels [m]

wrlmax=9.0;   % max angular velocity [rad/s] %can be increased??
Ts= 1/60;       % sampling time [s]
b = 0.17;          % distance point b [m]

%Online Optimization parameters
R=0*eye(2);
R_d=0*eye(2);
u_pre=[0;0];


parameters=[r d wrlmax Ts b];

A = eye(2);
Bu = eye(2)*Ts;
Bd = eye(2)*Ts;
C = eye(2);
D = [0; 0];

%Differential-drive wheels' angular velocities limits



%% TRAJECTORY PARAMETERS
%Initial conditions
%scaling factor of the trajectory
eta=1.5;
alpha=6.5;
k=0:Ts:2*pi*alpha*2-Ts;
xr=eta*sin(k/alpha);
yr=eta*sin(k/(2*alpha));

%Velocity trajectory
xpr=eta*cos(k/alpha)*(1/alpha);
ypr=eta*cos(k/(2*alpha))*(1/(2*alpha));

%Acceleration trajectory
xppr=-eta*sin(k/alpha)*(1/alpha)*(1/alpha);
yppr=-eta*sin(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));

%Driving velocity reference
vr=sqrt(xpr.^2+ypr.^2);
wr=(yppr.*xpr-xppr.*ypr)./(xpr.^2+ypr.^2);


% Orientation reference
thetar=atan2(ypr,xpr);

%Adjusting Orientation ( a part of tetha(k) is out of phase
thetar_diff=diff(thetar);
for i=1:length(thetar_diff)
    if thetar_diff(i)<-6
        i1=i+1;
    elseif thetar_diff(i)>6
        i2=i;
    end
end
thetar(i1:i2)=thetar(i1:i2)+2*pi;

vicon_client=tcpclient("192.168.0.5",18100);
vicon_data=read(vicon_client,14,"double");
flush(vicon_client);
clear vicon_client;
X0 = [vicon_data(1); vicon_data(2); vicon_data(6)];




%%Bound on the disturbance
disturbance=zeros(2,length(xr));
disturbance_norm=zeros(1,length(xr));
for i=1:length(xr)
    T_FL_i=[cos(thetar(i)) sin(thetar(i)); -sin(thetar(i))/b cos(thetar(i))/b];
    disturbance(1:2,i)=inv(T_FL_i)*[vr(i);wr(i)];
    
    
    disturbance_norm(i)=disturbance(:,i)'*disturbance(:,i);
end


rd=sqrt((max(disturbance_norm)));

Qd=rd^2*eye(2);% disturbance Shaping matrix




figure
plot(disturbance(1,:),disturbance(2,:))
hold on
ell_d=ellipsoid(Qd);
plot(ell_d)
hold off




T=[[r/2 r/2];[r/d -r/d]];
T_inv=inv(T);
Hd=[-1/wrlmax 0 ; 0 -1/wrlmax ; 1/wrlmax 0 ; 0 1/wrlmax];  %shaping matrix of box-like constraint set for differential-drive


U_dd= Polyhedron('lb',-[wrlmax;wrlmax],'ub',[wrlmax;wrlmax]); %Differential-drive input constraint set
U_uni=T*U_dd; %Unicycle input constraint set


ru=(2*wrlmax*r*b)/(sqrt(4*b^2+d^2));
Qu=ru^2*eye(2);% disturbance Shaping matrix



Nsets=10000;
Q0=Bd'*Qd*Bd;

plot_sets=false;
if plot_sets
    ell_0=ellipsoid(Q0);
    figure
    plot(ell_0)
    grid
    hold on
    axis([-0.8 0.8 -0.8 0.8])
end



Qcurr=Q0;
Q_k=Q0;

for i=1:Nsets
    
    Qi=one_step_ellipsoidal_reachable_set(A,Bu,Bd,Qcurr,Qu,Qd);
    
    if plot_sets
        ell_i=ellipsoid(Qi);
        plot(ell_i,'b')
    end
    
    
    pause(0.00001)
    Q_k=[Q_k Qi];
    Qcurr=Qi;
    
end



