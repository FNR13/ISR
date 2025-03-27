P = F_full*P0*F_full' + Q
K = (P*H'*(H*P*H' + R)^-1)
P=(eye(5)-(K*H))*P

P = F_full*P*F_full' + Q
K = (P*H'*(H*P*H' + R)^-1)
P=(eye(5)-(K)*H)*P

P = F_full*P*F_full' + Q
K = (P*H'*(H*P*H' + R)^-1)

%  P = F_full*P*F_full' + Q
%    P=(eye(5)-(P*H'*(H*P*H' + R)^-1)*H)*P
% 
% K = (P*H'*(H*P*H' + R)^-1)