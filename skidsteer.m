function wheelrot = skidsteer(V,w)

d=0.53; %Inter Wheel Distance
            r=0.115; %Wheel Radius
            wb=0.202; %wheel base

            TurnRadius = abs(V/w);

            R_in=TurnRadius-(d/2);
            R_out=TurnRadius+(d/2);

            theta_inner=atan2(wb,R_in);
            theta_outer=atan2(wb,R_out);

            V_star_in=(V/TurnRadius)*(sqrt(R_in^2 +wb^2));
            V_star_out=(V/TurnRadius)*(sqrt(R_out^2 +wb^2));

            V_in_wheel=V_star_in*cos(theta_inner);
            V_out_wheel=V_star_out*cos(theta_outer);
            

            w_outer=0.5*(V_out_wheel/r);
            w_inner=0.5*(V_in_wheel/r);

            wheelrot = [w_outer;w_inner];
end
            
            