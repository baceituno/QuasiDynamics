function [c,dc]=LinearMomentumConstraint(obj,x)


theta_h    =x(obj.LCtheta_h);
omega_h    =x(obj.LComega_h);
alpha_h    =x(obj.LCalpha_h);

theta_b    =x(obj.LCtheta_b);
omega_b    =x(obj.LComega_b);
alpha_b    =x(obj.LCalpha_b);

CPxh_h      =x(obj.LCPxh_h);
CPyh_h      =x(obj.LCPyh_h);
Vx_h        =x(obj.LVx_h);
Vx_h        =x(obj.LVy_h);
Ax_h        =x(obj.LAx_h);
Ax_h        =x(obj.LAy_h);


CPxh_b      =x(obj.LCPxh_b);
CPyh_b      =x(obj.LCPyh_b);
Vx_b        =x(obj.LVx_b);
Vx_b        =x(obj.LVy_b);
Ax_b        =x(obj.LAx_b);
Ax_b        =x(obj.LAy_b);

dtInv       =x(obj.LdtInv);

Fx_h        =x(obj.LFx_h);
Fy_h        =x(obj.LFy_h);

Fx_e        =x(obj.LFx_e);
Fy_e        =x(obj.LFy_e);


% M*Ax = dPxdT = sum(F_x)
% M*Ay = dPydT = sum(F_y)
for count=1:length(Ax_h)

    constraint_x(count,1)=sum(Fx_h(count,:))+sum(Fx_e(count,:))-obj.M_b*Ax_b(count);
    constraint_y(count,1)=sum(Fy_h(count,:))+sum(Fy_e(count,:))-obj.M_b*Ay_b(count);

end

constraint_out=[constraint_x;constraint_y];



end
