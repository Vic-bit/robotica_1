%Cinemática Directa - COBOT-19
%Le pasamos un vector articular, los parámetros de DH y los límites articulares
%Nos devuelve una matriz de transformación homogénea y un boolean que 
%usaremos para saber si está dentro del espacio de trabajo
function [T,dentro]= CD_funcion(q,dh,rqlim)

it=0;
dentro=1;
while it<1000
    if q(1)>rqlim(1,2)
        dentro=0;
        q(1)=q(1)-0.05;
    end
    if q(1)<rqlim(1,1)
        dentro=0;
        q(1)=q(1)+0.05;
    end
    if q(2)>rqlim(2,2)
        dentro=0;
        q(2)=q(2)-0.05;
    end
    if q(2)<rqlim(2,1) 
        dentro=0;
        q(2)=q(2)+0.05;
    end
    if q(3)>rqlim(3,2)
        dentro=0;
        q(3)=q(3)-0.05;
    end
    if q(3)<rqlim(3,1)
        dentro=0;
        q(3)=q(3)+0.05;
    end
    if q(4)>rqlim(4,2)
        dentro=0;
        q(4)=q(4)-0.01;
    end
    if q(4)<rqlim(4,1) 
        dentro=0;
        q(4)=q(4)+0.01;
    end
    if dentro==1
        break
    end
    it=it+1;
end

T = trotz(q(1)) * transl(0,0,dh(1,2)) * transl(dh(1,3),0,0)  * trotz(q(2)) * transl(0,0,dh(2,2)) * transl(dh(2,3),0,0) * trotx(pi) *trotz(q(3))* transl(0,0,dh(3,2))*transl(0,0,q(4));


