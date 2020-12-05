function [theta] = P2_InvKin(P,fi)

px = P(1);
py = P(2);

l1 = 1;
l2 = 1;
l3 = 0.5;

wx = px - l3*cosd(fi);
wy = py - l3*sind(fi);
D = (wx^2 + wy^2 - l1^2 - l2^2)/(2*l1*l2);

% first solution
theta2_1 = atan2d(sqrt(1-D^2),D);
theta1_1 = atan2d(wy,wx) - atan2d(l2*sind(theta2_1),(l1 + l2*cosd(theta2_1)));
theta3_1 = fi - theta1_1 - theta2_1;

% second solution
theta2_2 = atand(-sqrt(1-D^2)/D);
theta1_2 = atan2d(wy,wx) - atan2d(l2*sind(theta2_2),(l1 + l2*cosd(theta2_2)));
theta3_2 = fi - theta1_2 - theta2_2;


%only one solution has been taken according to project
theta = [theta1_1;theta2_1;theta3_1]; 

end