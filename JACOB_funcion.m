%Jacobiano - COBOT-19
%Le pasamos el robot instanciado, los parámetros de DH, los límites articulares
%y un vector articular
%Nos devuelve el valor del determinante de la matriz jacobiana para ese vector
%articular
function [DJ2r]= JACOB_funcion(R,dh,rqlim,qz)
%% JACOBIANO Y DETERMINANTE SIMBOLICO
disp('Jacobiano y determinante simbólico')

syms q1 q2 q3 q4 a1 a2 d1 d2 d3 reals x y z

dhs = [0,d1,a1,0,0;
      0,d2,a2,pi,0;
      0,d3,0,0,0;
      0, 0,0,0,1];

r = SerialLink(dhs,'name','Cobot19')

q = [q1 q2 q3 q4];

J = simplify(r.jacob0(q));
fprintf('J =\n')
disp(J)

f1 = J(1,:);
f2 = J(2,:);
f3 = J(3,:);
f6 = J(6,:);
Jr = [f1;f2;f3;f6]
DJr = (det(Jr));
fprintf('det(J) = ')
disp(DJr) 
DJrs= simplify((det(Jr)))

q2_singular=solve(DJrs==0,q2);
fprintf('Se detecta una singularidad cuando q2 es igual a :  ')
disp(q2_singular) 


%% EJ DE UNA POSTURA CARACTERISTICA DE NUESTRA APLICACION
disp('Aplicación del Jacobiano y determinante')

R2 = SerialLink(dh)

J2 = R2.jacob0(qz);
fprintf('J2 =\n')
disp(J2)

f1 = J2(1,:);
f2 = J2(2,:);
f3 = J2(3,:);
f6 = J2(6,:);
J2r = [f1;f2;f3;f6]
DJ2r = det(J2r);
fprintf('det(J2) = ')
disp(DJ2r)

%% VELOCITY ELIPSE OF COBOT19
figure; 
R.plot(qz,'view','top','workspace',[-1 1 -1 1 -1 1])
R.teach(qz,'callback',@(R,q) R.vellipse(q));