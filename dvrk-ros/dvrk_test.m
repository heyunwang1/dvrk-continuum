clc
clear
q = [0,0,0,0,0,0.01]'
g0 = forward(q)
qdesird = [1,0,0,0,1,0.01]'
gdesired = forward(qdesird)
T = 0.01;
K = 0.8;
% q = ur5.get_current_joints();
% Get the current pose
g = forward(q);
xi = getXi(gdesired \ g);
v = xi(1:3);
w = xi(4:6);
%Set gripper frame
%Define inital time factor
m = 35;
n = 1;
qk =q;
while or(norm(v) >0.005, norm(w) > pi/180)%Condition of stop
%     qk = ur5.get_current_joints();
    gk = forward(qk);
    J = jacob(qk);
    xi = getXi(gdesired \ gk);
    v = xi(1:3)
    w = xi(4:6);
    dq = K*T*(J\xi)
    %Increase the angle each movement step
    qk = qk - dq
    pause(1);
%     break
end
%%
function skew = hat(v)

skew = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];

end
function xi = getXi(g)
% Purpose: Take a homogenous transformation matrix and extract the unscaled twist
% Input: g: a homogeneous transformation
% Output: xi: the (un-normalized) twist in 6 ¡Á 1 vector or twist coordinate form such that
% g = exp(?¦Î)

[rows, cols] = size(g);
if ((rows ~= 4) | (cols ~= 4))
  error('getXi requires a 4x4 matrix argument. Check your dimensions.');
end

d = det(g);
if ((d <0.999) | (d >1.001))
  fprintf(1,'Error in getXi: the argument is not a rotation. Determinent is %f, should be 1.0\n',d);
  error('aborting');
end

xi_hat = logm(g);
v = xi_hat(1:3,4);
w_hat = xi_hat(1:3,1:3);
w = [w_hat(3,2);w_hat(1,3);w_hat(2,1)];
xi = [v;w];
end