function C_matrix = C_func(in1,in2,in3)
%C_func
%    C_matrix = C_func(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    2025-06-13 14:14:58

L = in1(3,:);
dL = in2(3,:);
dqx1 = in2(6,:);
dqx2 = in2(7,:);
dqx3 = in2(8,:);
dqy1 = in2(9,:);
dqy2 = in2(10,:);
dqy3 = in2(11,:);
dtheta_x = in2(4,:);
dtheta_y = in2(5,:);
dx = in2(1,:);
dy = in2(2,:);
m = in3(3,:);
qx1 = in1(6,:);
qx2 = in1(7,:);
qx3 = in1(8,:);
qy1 = in1(9,:);
qy2 = in1(10,:);
qy3 = in1(11,:);
rho = in3(4,:);
theta_x = in1(4,:);
theta_y = in1(5,:);
t2 = cos(theta_x);
t3 = cos(theta_y);
t4 = sin(theta_x);
t5 = sin(theta_y);
t6 = L.*rho;
t7 = L.^2;
t8 = pi.^2;
t9 = m.*2.0;
t10 = m.*3.0;
t11 = qy2.*3.0;
t12 = qy3.*2.0;
t13 = qy1.*6.0;
t14 = qx1.^2;
t15 = qx2.^2;
t16 = qx3.^2;
t17 = qy1.^2;
t18 = qy2.^2;
t19 = qy3.^2;
t20 = theta_x.*2.0;
t23 = 1.0./L;
t24 = 1.0./pi;
t25 = qx1.*3.6e+1;
t27 = qy1.*3.6e+1;
t28 = qx3.*1.0e+2;
t29 = qx3.*1.35e+2;
t30 = qx2.*2.25e+2;
t31 = qx2.*3.2e+2;
t32 = qx1.*3.75e+2;
t33 = qy3.*1.0e+2;
t34 = qy3.*1.35e+2;
t35 = qy2.*2.25e+2;
t36 = qy2.*3.2e+2;
t37 = qy1.*3.75e+2;
t39 = (dL.*rho)./2.0;
t40 = (dL.*rho)./4.0;
t41 = dL.*rho.*(2.0./3.0);
t42 = dL.*rho.*(6.0./5.0);
t49 = qx1.*2.0e+3;
t50 = qx2.*2.496e+3;
t51 = qx3.*5.616e+3;
t54 = qy1.*2.0e+3;
t55 = qy2.*2.496e+3;
t56 = qy3.*5.616e+3;
t59 = dL.*rho.*(3.0./8.0);
t21 = t2.^2;
t22 = sin(t20);
t26 = -t11;
t38 = t6./2.0;
t43 = dtheta_x.*t2.*t5;
t44 = dtheta_x.*t3.*t4;
t45 = dtheta_y.*t2.*t5;
t46 = dtheta_y.*t3.*t4;
t47 = -t30;
t48 = -t31;
t52 = -t35;
t53 = -t36;
t57 = -t41;
t58 = -t42;
t60 = t6+t9;
t61 = t6+t10;
t62 = qx1.*t8.*2.4e+1;
t63 = qy1.*t8.*2.4e+1;
t64 = -t50;
t65 = -t55;
t66 = -t59;
t67 = dL.*rho.*t24.*2.0;
t69 = qx2.*t8.*6.0e+2;
t70 = qx3.*t8.*6.0e+2;
t71 = qy2.*t8.*6.0e+2;
t72 = qy3.*t8.*6.0e+2;
t75 = t24.*t41;
t68 = m+t38;
t73 = -t69;
t74 = -t71;
t76 = dy.*t2.*t3.*t38;
t77 = t12+t13+t26;
t78 = t43+t46;
t79 = t44+t45;
t80 = (dtheta_y.*t7.*t22.*t61)./6.0;
t81 = t25+t29+t48+t62;
t82 = t27+t34+t53+t63;
t83 = t28+t32+t64+t70;
t84 = t33+t37+t65+t72;
t85 = t47+t49+t51+t73;
t86 = t52+t54+t56+t74;
t87 = (dL.*rho.*t23.*t81)./1.44e+2;
t88 = (dL.*rho.*t23.*t82)./1.44e+2;
t89 = (dL.*rho.*t23.*t83)./4.0e+2;
t90 = (dL.*rho.*t23.*t84)./4.0e+2;
t91 = (dL.*rho.*t23.*t85)./9.0e+2;
t92 = (dL.*rho.*t23.*t86)./9.0e+2;
mt1 = [t39,0.0,(dx.*rho)./2.0+dqx1.*rho.*t24.*2.0+dqx3.*rho.*t24.*(2.0./3.0)+dtheta_x.*t2.*t68,t68.*(dL.*t2-L.*dtheta_x.*t4),0.0,t67,0.0,t75,0.0,0.0,0.0,0.0,t39,(dy.*rho)./2.0+dqy1.*rho.*t24.*2.0+dqy3.*rho.*t24.*(2.0./3.0)+dtheta_y.*t2.*t3.*t68-dtheta_x.*t4.*t5.*t68,-t68.*(L.*t43+L.*t46+dL.*t4.*t5),-t68.*(L.*t44+L.*t45-dL.*t2.*t3),0.0,0.0,0.0,t67,0.0,t75,rho.*(dx+L.*dtheta_x.*t2).*(-1.0./2.0),rho.*(dy+L.*dtheta_y.*t2.*t3-L.*dtheta_x.*t4.*t5).*(-1.0./2.0)];
mt2 = [(dqx1.*rho.*t23.*t81)./1.44e+2+(dqx3.*rho.*t23.*t83)./4.0e+2-(dqx2.*rho.*t23.*t85)./9.0e+2+(dqy1.*rho.*t23.*t82)./1.44e+2+(dqy3.*rho.*t23.*t84)./4.0e+2-(dqy2.*rho.*t23.*t86)./9.0e+2-(dL.*rho.*(t14.*4.5e+2+t15.*4.5e+2+t16.*4.5e+2+t17.*4.5e+2+t18.*4.5e+2+t19.*4.5e+2-qx1.*qx2.*8.0e+3+qx1.*qx3.*3.375e+3-qx2.*qx3.*2.2464e+4-qy1.*qy2.*8.0e+3+qy1.*qy3.*3.375e+3-qy2.*qy3.*2.2464e+4+t8.*t14.*3.0e+2+t8.*t15.*1.2e+3+t8.*t16.*2.7e+3+t8.*t17.*3.0e+2+t8.*t18.*1.2e+3+t8.*t19.*2.7e+3))./(t7.*3.6e+3)];
mt3 = [dx.*t2.*t6.*(-1.0./2.0)+dy.*t4.*t5.*t38-(t6.*t24.*t46.*t77)./3.0-(L.*dtheta_x.*t24.*(m.*pi.*6.0+t6.*pi.*3.0+qx1.*rho.*t4.*1.2e+1-qx2.*rho.*t4.*6.0+qx3.*rho.*t4.*4.0+qy1.*rho.*t2.*t5.*1.2e+1-qy2.*rho.*t2.*t5.*6.0+qy3.*rho.*t2.*t5.*4.0))./6.0,-dtheta_y.*((L.*t6.*t21)./6.0+(L.*t21.*t61)./3.0+(t2.*t5.*t6.*t24.*t77)./3.0)-(dy.*t2.*t3.*t6)./2.0-(t6.*t24.*t44.*t77)./3.0,t87,-t91,t89,t88,-t92,t90,dL.*t2.*t38,dL.*t4.*t5.*t6.*(-1.0./2.0)];
mt4 = [(t24.*(dqx1.*t2.*t6.*1.2e+1-dqx2.*t2.*t6.*6.0+dqx3.*t2.*t6.*4.0+dL.*qx1.*rho.*t2.*1.2e+1-dL.*qx2.*rho.*t2.*6.0+dL.*qx3.*rho.*t2.*4.0-dqy1.*t4.*t5.*t6.*1.2e+1+dqy2.*t4.*t5.*t6.*6.0-dqy3.*t4.*t5.*t6.*4.0+L.*dtheta_x.*m.*pi.*6.0+L.*dtheta_x.*t6.*pi.*3.0+dx.*t2.*t6.*pi.*3.0-dy.*t4.*t5.*t6.*pi.*3.0-dL.*qy1.*rho.*t4.*t5.*1.2e+1+dL.*qy2.*rho.*t4.*t5.*6.0-dL.*qy3.*rho.*t4.*t5.*4.0))./6.0,(L.*dL.*t60)./2.0,t80,dL.*t2.*t6.*t24.*2.0,-dL.*t2.*t6.*t24,dL.*t2.*t6.*t24.*(2.0./3.0),dL.*t4.*t5.*t6.*t24.*-2.0,dL.*t4.*t5.*t6.*t24,dL.*t4.*t5.*t6.*t24.*(-2.0./3.0),0.0,dL.*t2.*t3.*t38];
mt5 = [t76+(L.*dtheta_y.*t21.*t60)./2.0+dqy1.*t2.*t3.*t6.*t24.*2.0-dqy2.*t2.*t3.*t6.*t24+dqy3.*t2.*t3.*t6.*t24.*(2.0./3.0)+(dL.*rho.*t2.*t3.*t24.*t77)./3.0,-t80,dtheta_x.*t7.*t22.*t61.*(-1.0./6.0)+(L.*dL.*t21.*t60)./2.0,0.0,0.0,0.0,dL.*t2.*t3.*t6.*t24.*2.0,-dL.*t2.*t3.*t6.*t24,dL.*t2.*t3.*t6.*t24.*(2.0./3.0),0.0,0.0,-t87+(dqx1.*rho)./4.0+dqx2.*rho.*(2.0./3.0)-dqx3.*rho.*(3.0./8.0),-L.*dtheta_x.*t4.*t6.*t24,0.0,t40,t41,t66,0.0,0.0,0.0,0.0,0.0,t91-dqx1.*rho.*(2.0./3.0)+(dqx2.*rho)./4.0+dqx3.*rho.*(6.0./5.0),L.*dtheta_x.*t4.*t24.*t38,0.0,t57,t40,t42,0.0,0.0,0.0,0.0,0.0,-t89+dqx1.*rho.*(3.0./8.0)-dqx2.*rho.*(6.0./5.0)+(dqx3.*rho)./4.0];
mt6 = [L.*dtheta_x.*t4.*t6.*t24.*(-1.0./3.0),0.0,t59,t58,t40,0.0,0.0,0.0,0.0,0.0,-t88+(dqy1.*rho)./4.0+dqy2.*rho.*(2.0./3.0)-dqy3.*rho.*(3.0./8.0),-L.*t6.*t24.*t78,-L.*t6.*t24.*t79,0.0,0.0,0.0,t40,t41,t66,0.0,0.0,t92-dqy1.*rho.*(2.0./3.0)+(dqy2.*rho)./4.0+dqy3.*rho.*(6.0./5.0),L.*t24.*t38.*t78,L.*t24.*t38.*t79,0.0,0.0,0.0,t57,t40,t42,0.0,0.0,-t90+dqy1.*rho.*(3.0./8.0)-dqy2.*rho.*(6.0./5.0)+(dqy3.*rho)./4.0,L.*t6.*t24.*t78.*(-1.0./3.0),L.*t6.*t24.*t79.*(-1.0./3.0),0.0,0.0,0.0,t59,t58,t40];
C_matrix = reshape([mt1,mt2,mt3,mt4,mt5,mt6],11,11);
end
