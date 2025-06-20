function M_matrix = M_func(in1,in2)
%M_func
%    M_matrix = M_func(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    2025-06-13 14:14:57

L = in1(3,:);
m = in2(3,:);
m_x = in2(1,:);
m_y = in2(2,:);
qx1 = in1(6,:);
qx2 = in1(7,:);
qx3 = in1(8,:);
qy1 = in1(9,:);
qy2 = in1(10,:);
qy3 = in1(11,:);
rho = in2(4,:);
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
t10 = qx1.*6.0;
t11 = qy2.*3.0;
t12 = qy3.*2.0;
t13 = qy1.*6.0;
t14 = qx1.^2;
t15 = qx2.^2;
t16 = qx3.^2;
t17 = qy1.^2;
t18 = qy2.^2;
t19 = qy3.^2;
t20 = qx3.*rho.*2.0;
t24 = 1.0./pi;
t25 = qx3.*9.0;
t26 = qx3.*1.0e+1;
t27 = qx1.*1.5e+1;
t28 = qx2.*1.5e+1;
t29 = qx2.*1.6e+1;
t30 = qx1.*4.0e+1;
t31 = qx2.*4.8e+1;
t33 = qy3.*9.0;
t34 = qy3.*1.0e+1;
t35 = qy1.*1.5e+1;
t36 = qy2.*1.5e+1;
t37 = qy2.*1.6e+1;
t38 = qy1.*4.0e+1;
t39 = qy2.*4.8e+1;
t46 = qx3.*7.2e+1;
t50 = qy3.*7.2e+1;
t21 = rho.*t10;
t22 = rho.*t12;
t23 = rho.*t13;
t32 = -t11;
t40 = qx2.*t2.*3.0;
t41 = qx3.*t2.*2.0;
t42 = t2.*t10;
t43 = -t25;
t44 = -t30;
t45 = -t31;
t47 = -t33;
t48 = -t38;
t49 = -t39;
t51 = m.*t4.*pi.*3.0;
t52 = t6./2.0;
t54 = t6+t9;
t56 = t4.*t5.*t12;
t57 = t4.*t5.*t13;
t60 = t6.*t24.*2.0;
t61 = m.*t2.*t5.*pi.*3.0;
t62 = t6.*t24.*(2.0./3.0);
t63 = L.*t2.*t6.*t24;
t68 = L.*t4.*t5.*t6.*t24;
t64 = t12+t13+t32;
t65 = t10+t29+t43;
t66 = t13+t37+t47;
t67 = t3.*t63;
t70 = t63./3.0;
t71 = (L.*t2.*t54)./2.0;
t72 = t26+t27+t45;
t73 = t34+t35+t49;
t74 = t63.*(-1.0./2.0);
t75 = t28+t44+t46;
t76 = t36+t48+t50;
t77 = t20+t21+t51;
t78 = -t68;
t80 = (L.*t4.*t5.*t54)./2.0;
t83 = L.*t4.*t5.*t24.*t52;
t84 = t68./3.0;
t85 = t22+t23+t61;
t99 = t6.*t24.*(t40-t41-t42+t56+t57-qy2.*t4.*t5.*3.0).*(-1.0./3.0);
t79 = t3.*t71;
t82 = t67./3.0;
t86 = -t80;
t87 = t67.*(-1.0./2.0);
t88 = -t84;
t89 = (rho.*t65)./2.4e+1;
t90 = (rho.*t66)./2.4e+1;
t91 = (rho.*t72)./4.0e+1;
t92 = (rho.*t73)./4.0e+1;
t93 = (rho.*t75)./6.0e+1;
t94 = (rho.*t76)./6.0e+1;
t95 = (t24.*t77)./3.0;
t96 = (t2.*t3.*t6.*t24.*t64)./3.0;
t97 = (t24.*t85)./3.0;
mt1 = [m+m_x+t6,0.0,t95,t71,0.0,t60,0.0,t62,0.0,0.0,0.0,0.0,m+m_y+t6,t97,t86,t79,0.0,0.0,0.0,t60,0.0,t62,t95,t97,(L.*m.*1.8e+3+rho.*t14.*4.5e+2+rho.*t15.*4.5e+2+rho.*t16.*4.5e+2+rho.*t17.*4.5e+2+rho.*t18.*4.5e+2+rho.*t19.*4.5e+2-qx1.*qx2.*rho.*8.0e+3+qx1.*qx3.*rho.*3.375e+3-qx2.*qx3.*rho.*2.2464e+4-qy1.*qy2.*rho.*8.0e+3+qy1.*qy3.*rho.*3.375e+3-qy2.*qy3.*rho.*2.2464e+4+rho.*t8.*t14.*3.0e+2+rho.*t8.*t15.*1.2e+3+rho.*t8.*t16.*2.7e+3+rho.*t8.*t17.*3.0e+2+rho.*t8.*t18.*1.2e+3+rho.*t8.*t19.*2.7e+3)./(L.*1.8e+3),t99,t96,t89,t93,t91,t90,t94,t92,t71,t86,t99];
mt2 = [m.*t7+(t6.*t7)./3.0,0.0,t63,t74,t70,t78,t83,t88,0.0,t79,t96,0.0,(t2.^2.*t7.*(m.*3.0+t6))./3.0,0.0,0.0,0.0,t67,t87,t82,t60,0.0,t89,t63,0.0,t52,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t93,t74,0.0,0.0,t52,0.0,0.0,0.0,0.0,t62,0.0,t91,t70,0.0,0.0,0.0,t52,0.0,0.0,0.0,0.0,t60,t90,t78,t67,0.0,0.0,0.0,t52,0.0,0.0,0.0,0.0,t94,t83,t87,0.0,0.0,0.0,0.0,t52,0.0,0.0,t62,t92,t88,t82,0.0,0.0,0.0,0.0,0.0,t52];
M_matrix = reshape([mt1,mt2],11,11);
end
