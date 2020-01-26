function [c,dc]=AngularMomentumConstraint(obj,x)

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


% I*alpha=sum(tau)=sum(r x F)
for count=1:length(Ax_h)
    constraint_out(count,1)=sum(ry_h.*Fx_h(count,:))-sum(rx_h.*Fy_h(count,:))+sum(ry_x.*Fx_e(count,:))-sum(rx_e.*Fx_e(count,:))-obj.I_b*alpha_b(count);
end


end
