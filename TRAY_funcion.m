%Trayectorias - COBOT-19
%Le pasamos el robot instanciado, los parámetros de DH, los límites
%articulares, tres vectores articulares
%No nos devuelve nada
function TRAY_funcion(R,dh,rqlim,q0,qt,qr)
%% DEF DIFERNETES PUNTOS DEL END EFECTOR 
[TT0,dentro]= CD_funcion(q0,dh,rqlim);
[TTt,dentro]= CD_funcion(qt,dh,rqlim);
[TTr,dentro]= CD_funcion(qr,dh,rqlim);

%% LLAMADA DE LA FUNCION DE CINEMATICA INVERSA PARA TRANSFORMAR DEL ESP CARTESIANO A ARTICULAR

TT=TTt;
qstart=q0;
[q,qb,qsol,ET] = CI_funcion(TT,dh,rqlim,qstart);

qsolT=qb;

TT=TTr;
qstart=qsolT;
[q,qb,qsol,ET] = CI_funcion(TT,dh,rqlim,qstart);
qsolR=qb;

TT=TT0;
qstart=qsolR;
[q,qb,qsol,ET] = CI_funcion(TT,dh,rqlim,qstart);
qsolF=qb;

%% INTERPOLACION ARTICULAR

[tgT,tgTd,tgTdd]=jtraj(q0,qsolT,25);
[tgR,tgRd,tgRdd]=jtraj(qsolT,qsolR,25);
[tgF,tgFd,tgFdd]=jtraj(qsolR,qsolF,25);

tg=[tgT;tgR;tgF];
tgd=diff(tg);
tgd=[tgd;tgd(74,:)];
tgdd=diff(tgd);
tgdd=[tgdd;tgdd(74,:)];


figure('name','Usando jtraj','NumberTitle','off','windowstate','maximized'); 
trplot(TT0,'color','g','length',0.25)
hold on
trplot(TTt,'color','r','length',0.25)
hold on
trplot(TTr,'color','b','length',0.25)
hold on
R.plot(tg,'trail','k')


%% INTERPOLACION CARTESIANA

T2=ctraj(TT0,TTt,25);
qsolT2=zeros(25,4);
for i=1:25
    [q,qb,qsol,ET] = CI_funcion(T2(:,:,i),dh,rqlim,qstart);
    qsolT2(i,:)=qb;
    qstart=qb;
end

R2=ctraj(TTt,TTr,25);
qsolR2=zeros(25,4);
for i=1:25
    [q,qb,qsol,ET] = CI_funcion(R2(:,:,i),dh,rqlim,qstart);
    qsolR2(i,:)=qb;
    qstart=qb;
end

F2=ctraj(TTr,TT0,25);
qsolF2=zeros(25,4);
for i=1:25
    [q,qb,qsol,ET] = CI_funcion(F2(:,:,i),dh,rqlim,qstart);
    qsolF2(i,:)=qb;
    qstart=qb;
end
qT=[T2;R2;F2];
qsol=[qsolT2;qsolR2;qsolF2];
qsold=diff(qsol);
qsold=[qsold;qsold(74,:)];
qsoldd=diff(qsold);
qsoldd=[qsoldd;qsoldd(74,:)];


figure('name','Usando ctraj','NumberTitle','off','windowstate','maximized'); 
trplot(TT0,'color','g','length',0.25)
hold on
trplot(TTt,'color','r','length',0.25)
hold on
trplot(TTr,'color','b','length',0.25)
hold on
R.plot(qsol,'trail','k')


%% INTERPOLACION CON MTRAJ
[Tmt,Tmtd,Tmtdd]= mtraj(@lspb, q0, qt, 25);
[Rmt,Rmtd,Rmtdd]= mtraj(@lspb, qt, qr, 25);
[Fmt,Fmtd,Fmtdd]= mtraj(@lspb, qr, q0, 25);

mt=[Tmt;Rmt;Fmt];
mtd=[Tmtd;Rmtd;Fmtd];
mtdd=[Tmtdd;Rmtdd;Fmtdd];


figure('name','Usando mtraj','NumberTitle','off','windowstate','maximized'); 
trplot(TT0,'color','g','length',0.25)
hold on
trplot(TTt,'color','r','length',0.25)
hold on
trplot(TTr,'color','b','length',0.25)
hold on
R.plot(mt,'delay',0.001,'trail','k')


%%  GRÁFICAS
t=zeros(1,75);
for i=1:75
    t(1,i)=i;
end

figure;
subplot(1,3,1)
hold on
title('Usando jtraj');
xlabel('Tiempo');
ylabel('Desplazamiento angular');
axis ([ 0 75 -2.5 2]);
plot(t(1,:),tg(:,1))
plot(t(1,:),tg(:,2))
plot(t(1,:),tg(:,3))
plot(t(1,:),tg(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off

subplot(1,3,2)
hold on
title('Usando ctraj');
xlabel('Tiempo');
ylabel('Desplazamiento angular');
axis ([ 0 75 -2.5 2]);
plot(t(1,:),qsol(:,1))
plot(t(1,:),qsol(:,2))
plot(t(1,:),qsol(:,3))
plot(t(1,:),qsol(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off

subplot(1,3,3)
hold on
title('Usando mtraj');
xlabel('Tiempo');
ylabel('Desplazamiento angular');
axis ([ 0 75 -2.5 2]);
plot(t(1,:),mt(:,1))
plot(t(1,:),mt(:,2))
plot(t(1,:),mt(:,3))
plot(t(1,:),mt(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off


figure;
subplot(1,3,1)
hold on
title('Usando jtraj');
xlabel('Tiempo');
ylabel('Velocidad angular');
axis ([ 0 75 -0.2 0.2]);
plot(t(1,:),tgd(:,1))
plot(t(1,:),tgd(:,2))
plot(t(1,:),tgd(:,3))
plot(t(1,:),tgd(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off

subplot(1,3,2)
hold on
title('Usando ctraj');
xlabel('Tiempo');
ylabel('Velocidad angular');
axis ([ 0 75 -0.2 0.2]);
plot(t(1,:),qsold(:,1))
plot(t(1,:),qsold(:,2))
plot(t(1,:),qsold(:,3))
plot(t(1,:),qsold(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off

subplot(1,3,3)
hold on
title('Usando mtraj');
xlabel('Tiempo');
ylabel('Velocidad angular');
axis ([ 0 75 -0.2 0.2]);
plot(t(1,:),mtd(:,1))
plot(t(1,:),mtd(:,2))
plot(t(1,:),mtd(:,3))
plot(t(1,:),mtd(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off


figure;
subplot(1,3,1)
hold on
title('Usando jtraj');
xlabel('Tiempo');
ylabel('Aceleración angular');
axis ([ 0 75 -0.06 0.04]);
plot(t(1,:),tgdd(:,1))
plot(t(1,:),tgdd(:,2))
plot(t(1,:),tgdd(:,3))
plot(t(1,:),tgdd(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off

subplot(1,3,2)
hold on
title('Usando ctraj');
xlabel('Tiempo');
ylabel('Aceleración angular');
axis ([ 0 75 -0.06 0.04]);
plot(t(1,:),qsoldd(:,1))
plot(t(1,:),qsoldd(:,2))
plot(t(1,:),qsoldd(:,3))
plot(t(1,:),qsoldd(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off

subplot(1,3,3)
hold on
title('Usando mtraj');
xlabel('Tiempo');
ylabel('Aceleración angular');
axis ([ 0 75 -0.06 0.04]);
plot(t(1,:),mtdd(:,1))
plot(t(1,:),mtdd(:,2))
plot(t(1,:),mtdd(:,3))
plot(t(1,:),mtdd(:,4))
legend({'q1','q2','q3','q4'},'FontSize',8);
grid on
hold off

%------------------Desplazamiento------------------------
Ttgj=zeros(4,4,75);
for i=1:75
    [Ttgj(:,:,i),dentro]= CD_funcion(tg(i,:),dh,rqlim);
end

xj=zeros(75,1);
yj=zeros(75,1);
zj=zeros(75,1);
for i=1:75
    xj(i,1)=Ttgj(1,4,i);
    yj(i,1)=Ttgj(2,4,i);
    zj(i,1)=Ttgj(3,4,i);
end
figure;
hold on
subplot(3,3,1)
plot(t(1,:),xj(:,1))
xlabel('Tiempo');
ylabel('X');
title('Usando jtraj')
grid on
subplot(3,3,4)
plot(t(1,:),yj(:,1))
xlabel('Tiempo');
ylabel('Y');
grid on
subplot(3,3,7) 
plot(t(1,:),zj(:,1))
xlabel('Tiempo');
ylabel('Z');
grid on
hold off


xc=zeros(75,1);
yc=zeros(75,1);
zc=zeros(75,1);
for i=1:25
    xc(i,1)=qT(1,4,i);
    yc(i,1)=qT(2,4,i);
    zc(i,1)=qT(3,4,i);
end
for i=1:25
    xc(i+25,1)=qT(5,4,i);
    yc(i+25,1)=qT(6,4,i);
    zc(i+25,1)=qT(7,4,i);
end
for i=1:25
    xc(i+50,1)=qT(9,4,i);
    yc(i+50,1)=qT(10,4,i);
    zc(i+50,1)=qT(11,4,i);
end

hold on
subplot(3,3,2)
plot(t(1,:),xc(:,1))
xlabel('Tiempo');
ylabel('X');
title('Usando ctraj')
grid on
subplot(3,3,5)
plot(t(1,:),yc(:,1))
xlabel('Tiempo');
ylabel('Y');
grid on
subplot(3,3,8) 
plot(t(1,:),zc(:,1))
xlabel('Tiempo');
ylabel('Z');
grid on
hold off


Tmt=zeros(4,4,75);
for i=1:75
    [Tmt(:,:,i),dentro]= CD_funcion(mt(i,:),dh,rqlim);
end

xm=zeros(75,1);
ym=zeros(75,1);
zm=zeros(75,1);
for i=1:75
    xm(i,1)=Tmt(1,4,i);
    ym(i,1)=Tmt(2,4,i);
    zm(i,1)=Tmt(3,4,i);
end

hold on
subplot(3,3,3)
plot(t(1,:),xm(:,1))
xlabel('Tiempo');
ylabel('X');
title('Usando mtraj')
grid on
subplot(3,3,6)
plot(t(1,:),ym(:,1))
xlabel('Tiempo');
ylabel('Y');
grid on
subplot(3,3,9) 
plot(t(1,:),zm(:,1))
xlabel('Tiempo');
ylabel('Z');
grid on
hold off

%-------------Velocidades----------------
vxj=zeros(75,1);
vyj=zeros(75,1);
vzj=zeros(75,1);
axj=zeros(75,1);
ayj=zeros(75,1);
azj=zeros(75,1);

vxj=diff(xj);
vxj=[vxj;vxj(74,:)];
axj=diff(vxj);
axj=[axj;axj(74,:)];
vyj=diff(yj);
vyj=[vyj;vyj(74,:)];
ayj=diff(vyj);
ayj=[ayj;ayj(74,:)];
vzj=diff(zj);
vzj=[vzj;vzj(74,:)];
azj=diff(vzj);
azj=[azj;azj(74,:)];

vxc=zeros(75,1);
vyc=zeros(75,1);
vzc=zeros(75,1);
axc=zeros(75,1);
ayc=zeros(75,1);
azc=zeros(75,1);

vxc=diff(xc);
vxc=[vxc;vxc(74,:)];
axc=diff(vxc);
axc=[axc;axc(74,:)];
vyc=diff(yc);
vyc=[vyc;vyc(74,:)];
ayc=diff(vyc);
ayc=[ayc;ayc(74,:)];
vzc=diff(zc);
vzc=[vzc;vzc(74,:)];
azc=diff(vzc);
azc=[azc;azc(74,:)];

vxm=zeros(75,1);
vym=zeros(75,1);
vzm=zeros(75,1);
axm=zeros(75,1);
aym=zeros(75,1);
azm=zeros(75,1);

vxm=diff(xm);
vxm=[vxm;vxm(74,:)];
axm=diff(vxm);
axm=[axm;axm(74,:)];
vym=diff(ym);
vym=[vym;vym(74,:)];
aym=diff(vym);
aym=[aym;aym(74,:)];
vzm=diff(zm);
vzm=[vzm;vzm(74,:)];
azm=diff(vzm);
azm=[azm;azm(74,:)];


figure;
hold on
subplot(3,3,1)
plot(t(1,:),vxj(:,1))
xlabel('Tiempo');
ylabel('Vx');
title('Usando jtraj')
grid on
subplot(3,3,4)
plot(t(1,:),vyj(:,1))
xlabel('Tiempo');
ylabel('Vy');
grid on
subplot(3,3,7) 
plot(t(1,:),vzj(:,1))
xlabel('Tiempo');
ylabel('Vz');
grid on
hold off

hold on
subplot(3,3,2)
plot(t(1,:),vxc(:,1))
xlabel('Tiempo');
ylabel('Vx');
title('Usando ctraj')
grid on
subplot(3,3,5)
plot(t(1,:),vyc(:,1))
xlabel('Tiempo');
ylabel('Vy');
grid on
subplot(3,3,8) 
plot(t(1,:),vzc(:,1))
xlabel('Tiempo');
ylabel('Vz');
grid on
hold off

hold on
subplot(3,3,3)
plot(t(1,:),vxm(:,1))
xlabel('Tiempo');
ylabel('Vx');
title('Usando mtraj')
grid on
subplot(3,3,6)
plot(t(1,:),vym(:,1))
xlabel('Tiempo');
ylabel('Vy');
grid on
subplot(3,3,9) 
plot(t(1,:),vzm(:,1))
xlabel('Tiempo');
ylabel('Vz');
grid on
hold off

%--------------Aceleraciones-------------
figure;
hold on
subplot(3,3,1)
plot(t(1,:),axj(:,1))
xlabel('Tiempo');
ylabel('Ax');
title('Usando jtraj')
grid on
subplot(3,3,4)
plot(t(1,:),ayj(:,1))
xlabel('Tiempo');
ylabel('Ay');
grid on
subplot(3,3,7) 
plot(t(1,:),azj(:,1))
xlabel('Tiempo');
ylabel('Az');
grid on
hold off

hold on
subplot(3,3,2)
plot(t(1,:),axc(:,1))
xlabel('Tiempo');
ylabel('Ax');
title('Usando ctraj')
grid on
subplot(3,3,5)
plot(t(1,:),ayc(:,1))
xlabel('Tiempo');
ylabel('Ay');
grid on
subplot(3,3,8) 
plot(t(1,:),azc(:,1))
xlabel('Tiempo');
ylabel('Az');
grid on
hold off

hold on
subplot(3,3,3)
plot(t(1,:),axm(:,1))
xlabel('Tiempo');
ylabel('Ax');
title('Usando mtraj')
grid on
subplot(3,3,6)
plot(t(1,:),aym(:,1))
xlabel('Tiempo');
ylabel('Ay');
grid on
subplot(3,3,9) 
plot(t(1,:),azm(:,1))
xlabel('Tiempo');
ylabel('Az');
grid on
hold off

