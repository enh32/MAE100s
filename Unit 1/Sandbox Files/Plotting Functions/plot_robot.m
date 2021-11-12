function plot_robot(u_r, u_l, V,omega,L,r, plot_title)

    title(plot_title+': Robot Velocities')

    hold on
    grid on
    axis equal
    th2 = linspace(pi/2,pi/2+omega,50);
    rad = L/4;
    time = 1;
    [t,y] = ode45(@(t,y) diff_drive(t,y,V,omega),[0 time],[0 0 pi/2]);
    lim = 0.5;
    robotRad = L/2;
    axis([-lim lim -lim lim])
    arrowx = [robotRad, -robotRad, 0];
    arrowy = [0,0,0];
    arrowlenx = [0,0,0];
    arrowleny = [u_r*r,u_l*r,V];
    quiver(arrowx,arrowy,arrowlenx,arrowleny,0,'filled')
    th = linspace(0,2*pi,30);
    plot(robotRad*cos(th),robotRad*sin(th),'k-')
    plot([robotRad,0], ...
        [0,0],'k-')
    plot(rad*cos(th2),rad*sin(th2),'b-')
    plot([rad*cos(th2(end)),rad*.8*cos(th2(end-10))],[rad*sin(th2(end)),rad*.8*sin(th2(end-10))],'b-')
    plot([rad*cos(th2(end)),rad*1.2*cos(th2(end-10))],[rad*sin(th2(end)),rad*1.2*sin(th2(end-10))],'b-')
    rectangle('Position',[robotRad-r/4,-r,r/2,2*r],'EdgeColor','k')
    rectangle('Position',[-robotRad-r/4,-r,r/2,2*r],'EdgeColor','k')
    plot(y(:,1),y(:,2),'r-')
end