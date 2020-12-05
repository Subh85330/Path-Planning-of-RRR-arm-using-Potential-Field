
function [torq] = P2_Att_field(Oi0,Of,zi0,n,zita1,zita2,zita3,zitaE,eta1,eta2,eta3,b1,b2,b3,ro1,ro2,ro3)



% Jacobian for origion 2, 3 and Oe
Jo3 = [cross(zi0(:,1),(Oi0(:,3)-Oi0(:,1)))         [ 0 0 0]'                             [ 0 0 0]' ];
Jo5 = [cross(zi0(:,1),(Oi0(:,5)-Oi0(:,1)))         cross(zi0(:,2),(Oi0(:,5)-Oi0(:,3)))   [ 0 0 0]'];
JoE = [cross(zi0(:,1),(Oi0(:,7)-Oi0(:,1)))         cross(zi0(:,2),(Oi0(:,7)-Oi0(:,3)))   cross(zi0(:,3),(Oi0(:,7)-Oi0(:,5)))];


% Jacobian for floating points 1, 2, and 3 on links 1, 2 and 3
J_fp1 = [cross(zi0(:,1),(Oi0(:,2)-Oi0(:,1)))         [ 0 0 0]'                             [ 0 0 0]' ];
J_fp2 = [cross(zi0(:,1),(Oi0(:,4)-Oi0(:,1)))         cross(zi0(:,2),(Oi0(:,4)-Oi0(:,3)))   [ 0 0 0]' ];
J_fp3 = [cross(zi0(:,1),(Oi0(:,6)-Oi0(:,1)))         cross(zi0(:,2),(Oi0(:,6)-Oi0(:,3)))   cross(zi0(:,3),(Oi0(:,6)-Oi0(:,5)))];






for i=1:(n+1+3)
    
    % Attractive force
    zita = [zita1;0;zita2;0;zita3;0;zitaE];
    f_att(:,i) = -zita(i)*(Oi0(:,i) - Of(:,i));
    
    
    
    
    
    % Repulsive force
    
    % constraint motion below horizontal level only for right side
    ro = 0.1; % radius of influence
    x = [1;0;0];
    eta = 1;
    dist_vec = Oi0(:,i)-x;
    
    if norm(dist_vec)<=ro
        f_repC(:,i) = eta*(1/norm(dist_vec) - 1/ro)*(1/(norm(dist_vec)^2))*(dist_vec/norm(dist_vec));
        
    elseif norm(dist_vec)>ro
        f_repC(:,i) =[0;0;0];
        
    end
    
    
    % Three point obstacles
    ro = [ro1;ro2;ro3];
    b = [b1 b2 b3];
    eta = [eta1;eta2;eta3];
    for j=1:3
        dist_vec = Oi0(:,i)-b(:,j);
        
        if norm(dist_vec)<=ro(j)
            f_rep(:,i,j) = eta(j)*(1/norm(dist_vec) - 1/ro(j))*(1/(norm(dist_vec)^2))*(dist_vec/norm(dist_vec));
            
        elseif norm(dist_vec)>ro(j)
            f_rep(:,i,j) =[0;0;0];
            
        end
        
    end
    
    
    
   
    
    
    %self-collision avoidance
    
    for k=1:2
        ro = [0.1;0.1]; % radius of influence for collision avoidance
        a1 = Oi0(:,1); a2 = Oi0(:,4);; % point obstacle origion which can collide with other origions
        a = [a1 a2];
        eta = [1;1];
        dist_vec = Oi0(:,i)-a(:,k);
        
        if norm(dist_vec)<=ro(k)
            f_rep_SA(:,i,k) = eta(k)*(1/norm(dist_vec) - 1/ro(k))*(1/(norm(dist_vec)^2))*(dist_vec/norm(dist_vec));
            
        elseif norm(dist_vec)>ro(k)
            f_rep_SA(:,i,k) =[0;0;0];
            
        end
        
    end
    
    
    
    
    
    
end





% Torque on origions
torq_O2   =   (Jo3)'*( f_att(:,3) +  f_repC(:,3) + f_rep(:,3,1) + f_rep(:,3,2) + f_rep(:,3,3)   );
torq_O3   =   (Jo5)'*( f_att(:,5) +  f_repC(:,5) + f_rep(:,5,1) + f_rep(:,5,2) + f_rep(:,5,3) + f_rep_SA(:,5,1)     );
torq_OE   =   (JoE)'*( f_att(:,7) +  f_repC(:,7) + f_rep(:,7,1) + f_rep(:,7,2) + f_rep(:,7,3) + f_rep_SA(:,7,1) + f_rep_SA(:,7,2)    );

% Torque on floating points
torq_fp1  = (J_fp1)'*( f_att(:,2) +  f_repC(:,2) + f_rep(:,2,1) + f_rep(:,2,2) + f_rep(:,2,3)       );
torq_fp2  = (J_fp2)'*( f_att(:,4) +  f_repC(:,4) + f_rep(:,4,1) + f_rep(:,4,2) + f_rep(:,4,3)       );
torq_fp3  = (J_fp3)'*( f_att(:,6) +  f_repC(:,6) + f_rep(:,6,1) + f_rep(:,6,2) + f_rep(:,6,3) + f_rep_SA(:,6,1) + f_rep_SA(:,6,2)     );




% Combined torque
torq  =  torq_O2 +  torq_O3 + torq_OE + torq_fp1 + torq_fp2 + torq_fp3;





end


