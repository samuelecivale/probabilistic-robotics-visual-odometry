function files = list_measurements(data_dir)
  d = dir(fullfile(data_dir, "meas-*.dat"));
  if isempty(d)
    error("Nessun file meas-*.dat trovato in: %s", data_dir);
  end

  names = {d.name};
  [~, idx] = sort(names);
  d = d(idx);

  files = cell(numel(d), 1);
  for i = 1:numel(d)
    files{i} = fullfile(data_dir, d(i).name);
  endfor
endfunction

