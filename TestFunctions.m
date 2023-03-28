function TestFunctions(string)
    %TESTFUNCTION TestSingle functions

    tspan=linspace(0,200,10000);
    y_0=[1;1;1]+randn(3,1)/100;
    eu_0=[0.1;0.1;0.1;0;0];
    yeu_0=[y_0(:);eu_0(:)];
    u=[-0.3;0.6];
    p_0=[0.1;0.2];
    t_0=0;

    %DiffrDrive
    if string=="DiffDrive"
        [t,y]=ode45(@(t,y)DiffDrive(t,y,u,p_0),tspan,y_0);
        plot(t,y)
    end

    %DDController
    if string=="DDController"
        deu=DDController(t_0,eu_0,y_0,p_0);
        disp(["y=",y_0(:)'])
        disp(["u=",deu(4:5)'])
        disp(["de=",deu(1:3)'])
        deu=DDController(t_0+0.1,eu_0+deu*0.1,y_0,p_0);
        disp(["y=",y_0(:)'])
        disp(["u=",deu(4:5)'])
        disp(["de=",deu(1:3)'])
        
    end

    if string=="Trajectory"

        [r,dr,ddr]=Trajectory(tspan);
        plot(r(1,:),r(2,:));
        hold on
        plot(dr(1,:),dr(2,:));
        hold on
        plot(ddr(1,:),ddr(2,:));
        hold off

    end

    if string=="Evolution"
        p=p_0;
        [t,yeu]=ode45(@(t,yeu)Evolution(t,yeu,p,p_0),tspan,yeu_0);
        [rd,~,~]=Trajectory(t');
        rd=rd';
        size(t)
        size(yeu)
        size(rd)
        plot(rd(:,1),rd(:,2),yeu(:,1),yeu(:,2))
        %plot(t,[arrayfun(@(x,y)norm([x,y]),yeu(:,1),yeu(:,2)),arrayfun(@(x,y)norm([x,y]),rd(:,1),rd(:,2))]);
        legend("rd","y")
    end

end




function dyeu=Evolution(t,yeu,p,p_0)
    y=yeu(1:3);
    eu=yeu(4:8);
    deu=DDController(t,eu,y,p);
    u=deu(4:5);
    dy=DiffDrive(t,y,u,p_0);
    dyeu=[dy(:);deu(:)];

end

