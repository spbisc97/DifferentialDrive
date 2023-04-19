function TestFunctions(string) %#codegen
    %TESTFUNCTION TestSingle functions

    tspan=linspace(0,200,100000);
    y_0=[1;1;pi];%+randn(3,1);
    eu_0=[0.1;0.1;0.1;0;0];
    yeu_0=[y_0(:);eu_0(:)];
    u=[0.2;0.2];
    p_0=[0.1;0.2];
    t_0=0;

    %DiffrDrive
    if string=="DiffDrive"
        tic
        [~,y]=ode45(@(t,y)DiffDrive(t,y,u,p_0),tspan,y_0);
        plot(y(:,1),y(:,2))
        toc
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
        %p=p+(rand(1)-0.5)*0.2;
        [t,yeu]=ode45(@(t,yeu)Evolution(t,yeu,p,p_0),tspan,yeu_0);
        [rd,~,~]=Trajectory(t');
        rd=rd';
        size(t)
        size(yeu)
        size(rd)
        plot(rd(:,1),rd(:,2),yeu(:,1),yeu(:,2))
        legend("rd","y")
    end

end




function dyeu=Evolution(t,yeu,p,p_0) %#codegen
    y=yeu(1:3);
    eu=yeu(4:8);
    %noise=(randn(3,1))/1000;
    deu=DDController(t,eu,y,p_0);
    u=deu(4:5);
    dy=DiffDrive(t,y,u,p);
    dyeu=[dy(:);deu(:)];

end

