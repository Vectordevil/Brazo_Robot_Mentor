% CINDIR4 Cinemática directa.
% A05 = mentor_dir(q) devuelve la matriz de transformación del
% primer sistema de coordenadas al último en función del vector q
% de variables articulares.
%
% See also DENAVIT.
function A05 = mentor_dir(q,formato) %q=[tita1 d2 d3 tita4]


a1=(255-128)/(105-0); % Calculamos todos los a y b
b1=128-a1*0;
a2=(155-20)/(0+90);
b2=20-a2*(-90);
a3=(255-128)/(115-0);
b3=128-a3*0;
a4=(255-128)/(160-0);
b4=128-a4*0;
a5=(255-128)/(160-0);
b5=128-a5*0;

if formato == 'g' % los datos se han introducido en grados
    qg=q % como los datos se han introducido en grados, qg=q
    qr=qg*pi/180 % muestra el valor de qr en el command window
    qw(1)=qg(1)*a1+b1; % Las constantes a1 y b1 hay que calcularlas con
    % los datos dados en la práctica
    qw(2)=qg(2)*a2+b2; % Las constantes a2 y b2 hay que calcularlas con
    % los datos dados en la práctica
    qw(3)=qg(3)*a3+b3; % Las constantes a3 y b3 hay que calcularlas con
    % los datos dados en la práctica
    qw(4)=qg(4)*a4+b4; % Las constantes a4 y b4 hay que calcularlas con
    % los datos dados en la práctica
    qw(5)=qg(5)*a5+b5; % Las constantes a5 y b5 hay que calcularlas con
    % los datos dados en la práctica
    qw=round(qw) % redondea y lo muestra en el command window
end
if formato == 'r' % Los datos son introducidos en radianes 
    qr=q %Asignamos q a qr
    qg=qr*180/pi %pasamos de radianes a grados y de grados a sistema walli
    qw(1)=qg(1)*a1+b1; % Las constantes a1 y b1 hay que calcularlas con
    
    qw(2)=qg(2)*a2+b2; % Las constantes a2 y b2 hay que calcularlas con
    
    qw(3)=qg(3)*a3+b3; % Las constantes a3 y b3 hay que calcularlas con
    
    qw(4)=qg(4)*a4+b4; % Las constantes a4 y b4 hay que calcularlas con
    
    qw(5)=qg(5)*a5+b5; % Las constantes a5 y b5 hay que calcularlas con
    
    qw=round(qw) % redondea y lo muestra en el command window
 
end
if formato == 'w'
    qw=q
    
    qg(1)=(qw(1)-b1)/a1;
    qg(2)=(qw(2)-b2)/a2;
    qg(3)=(qw(3)-b3)/a3;
    qg(4)=(qw(4)-b4)/a4;
    qg(5)=(qw(5)-b5)/a5;
    qg
    
    qr=qg*pi/180
 
end


% Parámetros Denavit-Hartenberg del robot
l1=185;
l2=165;
l3=150;
l4=195;
teta = [pi/2-qr(1) pi/2+qr(2) qr(3) qr(4)-pi/2 qr(5)-pi/2];
d = [l1 0 0 0 l4];
a = [0 l2 l3 0 0];
alfa = [pi/2 0 0 -pi/2 0];
% Matrices de transformación homogénea entre sistemas de coordenadas
A01 = denavit(teta(1), d(1), a(1), alfa(1));
A12 = denavit(teta(2), d(2), a(2), alfa(2));
A23 = denavit(teta(3), d(3), a(3), alfa(3));
A34 = denavit(teta(4), d(4), a(4), alfa(4));
A45 = denavit(teta(5), d(5), a(5), alfa(5));
% Matriz de transformación del primer al último sistema de coordenadas
A05 = A01 * A12 * A23 * A34 * A45;