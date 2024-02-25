%Cinemática Inversa - COBOT-19
%Le pasamos una matriz, los parámetros de DH, los límites articulares y el
%vector articular semilla.
%Nos devuelve tres vectores articulares, dos de ellos la soluciones y el tercero es 
%la solución más cercana y si está dentro del espacio de trabajo
function [q,qb,qsol,ET] = CI_funcion(TT,dh,rqlim,qstart)

%FUNCION CINEMATICA INVERSA
% EN EL ESPACIO DE TRABAJO

x=TT(1,4); 
y=TT(2,4); 
z=TT(3,4);
roll=atan2(TT(2,1),TT(1,1));

l=x;
h=y;
if x<0
    if y>0
        l=y;
    elseif y<0
        l=-y;
    end
    h=-x;
end

gamma=atan2(h,l);
beta=real(acos((l^2+h^2+dh(1,3)^2+dh(2,3)^2)/(2*dh(1,3)*sqrt(l^2+h^2))));
alpha=real(acos((dh(1,3)^2+dh(2,3)^2-l^2-h^2)/(2*dh(1,3)*dh(2,3))));
omega=real(acos((l^2+h^2+dh(2,3)^2-dh(1,3)^2)/(2*dh(2,3)*sqrt(l^2+h^2))));

%   Def q2, q2b
q2=acos((l^2+h^2-dh(1,3)^2-dh(2,3)^2)/(2*dh(1,3)*dh(2,3)));
q2=real(q2);

q2b=-acos((l^2+h^2-dh(1,3)^2-dh(2,3)^2)/(2*dh(1,3)*dh(2,3)));
q2b=real(q2b);

%   Def q1,q1b
if q2>=0
    q1=atan(h/l)-atan((dh(2,3)*sin(q2))/(dh(1,3)+dh(2,3)*cos(q2)));
elseif q2<0
    q1=atan(h/l)+atan((dh(2,3)*sin(q2))/(dh(1,3)+dh(2,3)*cos(q2)));
end

if q2b>=0
    q1b=atan(h/l)-atan((dh(2,3)*sin(q2))/(dh(1,3)+dh(2,3)*cos(q2)));
elseif q2b<0
    q1b=atan(h/l)+atan((dh(2,3)*sin(q2))/(dh(1,3)+dh(2,3)*cos(q2)));
end

if x<0
    if y>0
        q1=q1+pi/2;
        q1b=q1b+pi/2;
    elseif y<0
        q1=q1-pi/2;
        q1b=q1b-pi/2;;
    end
end

%   Def q3,q3b
q3=q1+q2-roll;
q3b=q3-2*omega;

%   Def q4,q4b
q4=dh(1,2)+dh(2,2)-dh(3,2)-z;
q4b=q4;

%% FUERA DEL ESPACIO DE TRABAJO
ETa=1;
ETb=1;

% Def q1,q1b 
if q1>rqlim(1,2)
    q1=rqlim(1,2);
    ETa=0;
elseif q1<rqlim(1,1)
    q1=rqlim(1,1);
    ETa=0;
end
    
if q1==rqlim(1,2)
    q1b=rqlim(1,2);
    ETb=0;
elseif q1==rqlim(1,1)
    q1b=rqlim(1,1);
    ETb=0;
end
    
% Def q2,q2b
if q2>rqlim(2,2)
    q2=rqlim(2,2);
    ETa=0;
elseif q2<rqlim(2,1)
    ETa=0;
    q2=rqlim(2,1);
end
   
if q2b>rqlim(2,2)
    q2b=rqlim(2,2);
    ETb=0;
elseif q2b<rqlim(2,1)
    q2b=rqlim(2,1);
    ETb=0;
end

% Def q3,q3b
if q3>rqlim(3,2)
    q3=rqlim(3,2);
    ETa=0;
elseif q3<rqlim(3,1)
    q3=rqlim(3,1);
    ETa=0;
end
   
if q3b>rqlim(3,2)
    q3b=rqlim(3,2);
    ETb=0;
elseif q3b<rqlim(3,1)
    q3b=rqlim(3,1);
    ETb=0;
end
   
% Def q4,q4b
if q4>rqlim(4,2)
    q4=rqlim(4,2);
    ETa=0;
elseif q4<rqlim(4,1)
    q4=rqlim(4,1);
    ETa=0;
end
   
if q4b>rqlim(4,2)
    q4b=rqlim(4,2);
    ETb=0;
elseif q4b<rqlim(4,1)
    q4b=rqlim(4,1);
    ETb=0;
end   
   
%% ELECCIÃ“N DE LA MÃ?S CERCANA (!solamente funciona por q0=[0 0 0 0]!)

q=[q1 q2 q3 q4];
qb=[q1b q2b q3b q4b];

contador_q=0;
contador_qb=0;
for i=1:1:3
    contador_q=contador_q+(abs(qstart(i))-abs(q(i)));
    contador_qb=contador_qb+(abs(qstart(i))-abs(qb(i)));
end

if contador_q<=contador_qb
    qsol=q;
    ET=ETa;
else
    qsol=qb;
    ET=ETb;
end
