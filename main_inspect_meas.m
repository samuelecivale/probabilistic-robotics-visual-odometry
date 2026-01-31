clear; close all; clc;
addpath(genpath("src"));

fn = "data/meas-00000.dat";

% Leggi come testo per capire struttura
fid = fopen(fn, "r");
if fid < 0, error("Non riesco ad aprire %s", fn); endif

printf("Prime 10 righe di %s:\n\n", fn);
for i=1:10
  line = fgetl(fid);
  if ~ischar(line), break; endif
  printf("%s\n", line);
endfor
fclose(fid);

% Prova a leggere numerico ignorando eventuali righe non numeriche
M = read_numeric_matrix(fn);

printf("\nLetto come matrice numerica: %dx%d\n", rows(M), columns(M));
if rows(M) > 0
  printf("Prima riga (numerica):\n");
  disp(M(1, :));
endif

