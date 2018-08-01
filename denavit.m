% DENAVIT Matriz de transformación homogénea.
% DH = DENAVIT(TETA, D, A, ALFA) devuelve la matriz de transformación
% homogénea 4 x 4 a partir de los parámetros de Denavit-Hartemberg
% D, ALFA, A y TETA.
function dh=denavit(teta, d, a, alfa)

dh=[cos(teta) -cos(alfa)*sin(teta) sin(alfa)*sin(teta) a*cos(teta);
    sin(teta) cos(alfa)*cos(teta) -sin(alfa)*cos(teta) a*sin(teta);
    0 sin(alfa) cos(alfa) d;
    0 0 0 1];
