function dx_dt = SP_ode(t,x,g,m,l,b)
   x_1 = x(1);
   x_2 = x(2);
   dx1_dt = x_2;
   dx2_dt = -(g/l)*sin(x_1) - (b/(m*l*l))*x_2;
   dx_dt = [dx1_dt;dx2_dt];
end