function [Oi0,zi0,n] = P2_forKin(theta)

n=3;                       % Numbers of links
l1 = 1; l2 = 1; l3 =0.5;   % length of links

% DH parameters
alpha = [0 0 0];
ai    = [0  l1 l2];
di    = [0 0 0];



den=[l3;0; 0];
Temp = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

for i=1:n
    T_i_i_1= [  cosd(theta(i))                           -sind(theta(i))                         0                          ai(i)
                sind(theta(i))*cos(alpha(i))             cosd(theta(i))*cosd(alpha(i))         -sind(alpha(i))        -sind(alpha(i))*di(i)
                sind(theta(i))*sin(alpha(i))             cosd(theta(i))*sind(alpha(i))          cosd(alpha(i))         cosd(alpha(i))*di(i)
                0                                           0                                    0                           1                  ];      
    
    Ti0(:,:,i)=Temp*T_i_i_1;
    Temp=Ti0(:,:,i);
    
    Oi0(1:3,i)=Ti0(1:3,4,i);
    zi0(1:3,i)=Ti0(1:3,3,i);
    
    
    
    
end
Pe0=Oi0(:,n) + Ti0(1:3,1:3,n)*(den);
Oi0(:,(n+1))=Pe0;


% defining folating points (one per link) middle point of each link
fp1 = (Oi0(:,2)+Oi0(:,1))/2;
fp2 = (Oi0(:,3)+Oi0(:,2))/2;
fp3 = (Oi0(:,4)+Oi0(:,3))/2;

Oi0 =    [Oi0(:,1) fp1 Oi0(:,2) fp2 Oi0(:,3) fp3 Oi0(:,4)];
%position -  1      2      3     4     5      6       7



end