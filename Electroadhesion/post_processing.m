x=-1.5:0.01:1.5;
y=-1:0.01:1;

uxy=tri2grid(p,t,u,x,y);
[FU, FV] = gradient(uxy);
quiver(x,y,FU,FV)
strength=vecnorm([FU(3:199,150),FV(3:199,150)]');
save('ratio change/width02.mat','strength');

plot(strength,y(3:199))
xlabel('E Strength')
ylabel('z Depth')
title('width 0.2 E vs z')

% [ux,uy] = pdegrad(p,t,u);
% h=pdeplot(p,e,t,'XYData',u,'FlowData',[ux;uy])
% mesh(x,y,uxy)