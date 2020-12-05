clc
clear
close all
%% Input parameters

% Influence coefficient for Attractive Field
zita1 = 1; zita2 = 1; zita3 = 3; zitaE = 1;


% point obstacle       radius of influence    Influence of repulsive force

b1 = [1;1.5;0];          ro1 = 0.2;            eta1 = 1;
b2 = [0;1.3;0];          ro2 = 0.2;            eta2 = 1;
b3 = [-1;0.9;0];         ro3 = 0.3;            eta3 = 1;




P = [1.866; 1.366; 0]; fi = 0;   % Initil position and orientation of end effector

P_goal = [-2.081; 0.621; 0]; fi_goal = 140; % Goal position and orientation of end effector



%% Inverse and forward Kinematics



[thetaf] = P2_InvKin(P_goal,fi_goal);
[Of,~,n] = P2_forKin(thetaf);



[theta] = P2_InvKin(P,fi);


%%
i = 3; alpha =0.5;
while  norm(theta - thetaf)>1
    N = norm(theta - thetaf)
    [Oi0,zi0,n] = P2_forKin(theta);
    
    [torq] = P2_Att_field(Oi0,Of,zi0,n,zita1,zita2,zita3,zitaE,eta1,eta2,eta3,b1,b2,b3,ro1,ro2,ro3);
    
    thetaNew = theta + (alpha*torq)/norm(torq);
    
    
    
    th(:,i) = theta;
    N2 = norm(th(:,i) - th(:,i-2))
    
    if norm(th(:,i) - th(:,i-2))<0.01
        thetaNew= [theta(1)-randi([5 10]); theta(2)-randi([10 20]); theta(3)-randi([10 30])];
    end
    
    
    
    
    
    X = Oi0(1,:);
    Y = Oi0(2,:);
    Z = Oi0(3,:);
    
    
    
%     plot3(X,Y,Z,b1(1),b1(2),0,'*',b2(1),b2(2),0,'*',b3(1),b3(2),0,'*')
    
    
    plot(X,Y,b1(1),b1(2),'*',b2(1),b2(2),'*',b3(1),b3(2),'*',[-0.5 0.5],[0 0])
    text(P_goal(1), P_goal(2), 'Goal')
    
    axis([-2.5 2.5 -1.5 2.5])
    xlabel(["X-axis";"Presented By Group-5"],'color','k')
    ylabel("Y-axis",'color','k')
    zlabel("Z-axis",'color','k')
    title(["Path Planning of RRR Robot";"Using Potential Field"])
    pause(0.01)
    
    theta = thetaNew;
    
    i=i+1;
end