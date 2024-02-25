%Código principal del robot
%COBOT-19

clc, clear
disp('---------------------------------')
disp('           COBOT-19              ')
disp('---------------------------------')

%----------------------Parámetros de DH----------------------
dh = [
    0.000  0.250  0.300  0.000 0;
    0.000  0.050  0.250  pi 0;
    0.000  0.030  0.000  0.000 0;
    0.000  0.000  0.000  0.000 1];

R = SerialLink(dh,'name','Cobot-19');
q = [0,0,0,0];

%----------------------Límites artículares----------------------
R.qlim(1,1:2) = [-90 ,90]*pi/180;
R.qlim(2,1:2) = [-120,120]*pi/180;
R.qlim(3,1:2) = [-200,200]*pi/180;
R.qlim(4,1:2) = [0, 0.25];
rqlim=R.qlim;

%----------------------Herramienta----------------------
%R.base = transl(0,0,0); 
l=0.40;
%R.tool = transl(l,0,0);
%R.offset = [0 0 0 0];

disp('--------------------------------------------------------')
fprintf('Presione 0 para ver el espacio del trabajo del robot\n')
fprintf('Presione 1 para ver los parámetros de DH y algunas posturas del robot\n')
fprintf('Presione 2 para comprobar la Cinemática Directa del robot\n')
fprintf('Presione 3 para comprobar la Cinemática Indirecta del robot\n')
fprintf('Presione 4 para comprobar la Jacobiano del robot\n')
fprintf('Presione 5 para comprobar la Trayectorias del robot\n')
fprintf('Presione 6 para finalizar\n')
disp('--------------------------------------------------------')
flag=1;
while((flag==1))
    
    OPT=input('Elija una opción\n');
    if (OPT==0)
        q = [0,0,0,0];
        %----------------------2D plano X-Y----------------------        
        theta1 = rqlim(1,1):0.1:rqlim(1,2); 
        theta2 = rqlim(2,1):0.1:rqlim(2,2);
        theta3 = rqlim(3,1):0.1:rqlim(3,2);       
        [THETA1,THETA2,THETA3] = meshgrid(theta1,theta2, theta3);
        
        l=0;
        X = dh(1,3) * cos(THETA1) + dh(2,3) * cos(THETA1 + THETA2)+ l * cos(THETA1 + THETA2 + THETA3);
        Y = dh(1,3) * sin(THETA1) + dh(2,3) * sin(THETA1 + THETA2)+ l * sin(THETA1 + THETA2 + THETA3); 
        figure
        hold on
        plot(X(:),Y(:),'r.'); 
        %axis equal;
        R.plot(q,'view','top','workspace',[-2 2 -2 2 -2 2])
        title('Sin herramienta')
        hold off
        
        l=0.40;
        X = dh(1,3) * cos(THETA1) + dh(2,3) * cos(THETA1 + THETA2)+ l * cos(THETA1 + THETA2 + THETA3);
        Y = dh(1,3) * sin(THETA1) + dh(2,3) * sin(THETA1 + THETA2)+ l * sin(THETA1 + THETA2 + THETA3); 
        figure
        hold on
        plot(X(:),Y(:),'r.'); 
        %axis equal;
        R.plot(q,'view','top','workspace',[-2 2 -2 2 -2 2])
        title('Con herramienta')
        hold off
        %*-------------------2D plano Y-Z------------------------
        lbrazo = -(dh(1,3)+dh(2,3)):0.01:(dh(1,3)+dh(2,3)); 
        profundidad = 0.3:-0.002:0.05; 
        [LBRAZO, PROFUNDIDAD] = meshgrid(lbrazo,profundidad);
        figure
        hold on
        plot(LBRAZO(:),PROFUNDIDAD(:),'b.');
        xlabel('Y')
        ylabel('Z')
        title('Sin herramienta')
        hold off
        
        lbrazo = -(dh(1,3)+dh(2,3)+l):0.01:(dh(1,3)+dh(2,3)+l); 
        profundidad = 0.3:-0.002:0.05; 
        [LBRAZO, PROFUNDIDAD] = meshgrid(lbrazo,profundidad);
        figure
        hold on
        plot(LBRAZO(:),PROFUNDIDAD(:),'b.');
        xlabel('Y')
        ylabel('Z')
        title('Con herramienta')
        hold off
    end
    if (OPT==1)
        fprintf('Parámetros de DH\n')
        display(R)
        fprintf('Límites artículares\n')
        disp(rqlim);
        fprintf('La longitud de la herramienta es: %f\n', l )
        q0=[pi/4 -pi/2 -3*pi/4 0]; % posicion articular inicial y final del robot 
        qt=[0 -pi/2 -3*pi/4 0.19]; % posicion articular para toma de temp
        qr=[5*pi/12 -pi/6 0 0.1]; % posicion articular para toma de freq respi
        figure
        R.plot(q0,'workspace',[-1 1 -1 1 -1 1])
        figure
        R.plot(qt,'workspace',[-1 1 -1 1 -1 1])
        figure
        R.plot(qr,'workspace',[-1 1 -1 1 -1 1])
        fprintf('\n')
    end
    if (OPT==2)
        fprintf('El vector articular de ejemplo es: \n')
        qf=[5*pi/12 -pi/6 0 0.1]
        fprintf('La matriz obtenida con fkine es: \n')
        Tf=R.fkine(qf)
        fprintf('La matriz obtenida con nuestra función de CD sin la herramienta es: \n')
        [T,dentro] = CD_funcion(qf,dh,rqlim)
        if dentro==0
           disp('Está FUERA del espacio de trabajo'); 
        end
        if dentro==1
           disp('Está DENTRO del espacio de trabajo'); 
        end
        fprintf('La matriz obtenida con nuestra función de CD con la herramienta es: \n')
        T=transl(0,0,0) * T * transl(l,0,0)
        fprintf('\n')
    end
    if (OPT==3)
        fprintf('El vector articular de ejemplo es: \n')
        qf=[5*pi/12 -pi/6 0 0.1]
        TT=R.fkine(qf);
        Tf=TT.double;
        q0= [0 0 0 0];
        fprintf('El vector articular obtenido con ikine es: \n')
        qi=R.ikine(Tf,qf,'mask',[1 1 1 1 0 0]);
        disp(qi)
        fprintf('El vector articular obtenido con función de CI es: \n')
        [q,qb,qsol,ET] = CI_funcion(Tf,dh,rqlim,qf);
        fprintf('Las posibles soluciones son: \n');
        disp(q)
        disp(qb)
        if q==qsol
            fprintf('==> La solución más cercana es q: ')
            disp(qsol)
        else
            fprintf('==> La solución más cercana es qb: ')
        end
        if ET==0
            disp('Esta solución está FUERA del espacio de trabajo')
        else
            disp('Esta solución está DENTRO del espacio de trabajo')
        end
        fprintf('\n')
    end
    if (OPT==4)
        qz=[5*pi/12 -pi/6 0 0.1]; %posición articular característica
        [DJ2r]= JACOB_funcion(R,dh,rqlim,qz)
        fprintf('\n')
    end
    if (OPT==5)
        q0=[pi/4 -pi/2 -3*pi/4 0]; % posicion articular inicial y final del robot 
        qt=[0 -pi/2 -3*pi/4 0.19]; % posicion articular para toma de temp
        qr=[5*pi/12 -pi/6 0 0.1]; % posicion articular para toma de freq respi
        disp('Posicion articular inicial y final del robot'), q0
        disp('Poosicion articular para toma de temperatura'), qt
        disp('Posicion articular para toma de frequencia respiratoria'), qr
        TRAY_funcion(R,dh,rqlim,q0,qt,qr);
        fprintf('\n')
    end
    if (OPT==6)
        disp('Ha finalizado la prueba');
        flag=0;
        fprintf('\n')
    end
    disp('--------------------------------------------------------')
    fprintf('\n')
end