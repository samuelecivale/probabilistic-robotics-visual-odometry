function M = read_numeric_matrix(filename)
  % Legge un file di numeri anche se ci sono spazi multipli o righe vuote.
  % Se ci sono header testuali, prova a scartarli.

  fid = fopen(filename, "r");
  if fid < 0, error("Cannot open %s", filename); endif

  rows_cell = {};
  while true
    line = fgetl(fid);
    if ~ischar(line), break; endif
    line = strtrim(line);
    if isempty(line), continue; endif

    % prova a parsare la riga come numeri
    nums = sscanf(line, "%f")';
    if isempty(nums)
      % riga non numerica -> skip
      continue;
    endif
    rows_cell{end+1,1} = nums; %#ok<AGROW>
  endwhile
  fclose(fid);

  if isempty(rows_cell)
    M = zeros(0,0);
    return;
  endif

  % Alcune righe potrebbero avere lunghezze diverse: tagliamo al min comune
  lens = cellfun(@numel, rows_cell);
  m = min(lens);
  if m == 0
    M = zeros(0,0);
    return;
  endif

  M = zeros(numel(rows_cell), m);
  for i=1:numel(rows_cell)
    M(i,:) = rows_cell{i}(1:m);
  endfor
endfunction

