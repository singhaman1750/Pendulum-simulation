function dx_dt = DP_ode(t,x,g,m1,m2,l1,l2)
   A = ((m1+m2)*l1*l1)/2;
   B = (m2*l2*l2)/2;
   C = m2*l1*l2;
   D = (m1+m2)*g*l1;
   E = m2*g*l2;

   Theta_1 = x(1);
   Theta1_dot = x(2);
   Theta_2 = x(3);
   Theta2_dot = x(4);
   dx1_dt = Theta1_dot;
   dx2_dt = (2*B/(4*A*B-C*C*(cos(Theta_2)^2))) * ...
      (E*sin(Theta_1 + Theta_2)*((C*cos(Theta_2))/(2*B) - 1) + C*sin(Theta_2)*(Theta2_dot)^2 - D*sin(Theta_1));
   dx3_dt = Theta2_dot;
   dx4_dt = (1/(2*B)) * (-C*cos(Theta_2)*dx2_dt - E*sin(Theta_1 + Theta_2));
   dx_dt = [dx1_dt;dx2_dt;dx3_dt;dx4_dt];
end