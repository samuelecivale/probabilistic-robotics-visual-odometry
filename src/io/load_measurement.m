function meas = load_measurement(filename)
  fid = fopen(filename, "r");
  if fid < 0
    error("Non riesco ad aprire: %s", filename);
  endif

  meas.seq = NaN;
  meas.gt_pose = [NaN; NaN; NaN];
  meas.odom_pose = [NaN; NaN; NaN];

  local_id = [];
  true_id  = [];
  uv = [];
  app = [];

  while true
    line = fgetl(fid);
    if ~ischar(line), break; endif
    line = strtrim(line);
    if isempty(line), continue; endif

    if startsWith(line, "seq:")
      nums = parse_numbers(line);
      if ~isempty(nums), meas.seq = nums(1); endif

    elseif startsWith(line, "gt_pose:")
      nums = parse_numbers(line);
      if numel(nums) >= 3, meas.gt_pose = nums(1:3)(:); endif

    elseif startsWith(line, "odom_pose:")
      nums = parse_numbers(line);
      if numel(nums) >= 3, meas.odom_pose = nums(1:3)(:); endif

    elseif startsWith(line, "point")
      nums = parse_numbers(line);
      % Atteso: local_id, true_id, u, v, app(10)  => totale 14 numeri
      if numel(nums) < 4
        continue;
      endif

      local_id(end+1,1) = nums(1);
      true_id(end+1,1)  = nums(2);
      uv(end+1,1:2)      = nums(3:4);

      if numel(nums) >= 14
        app(end+1,1:10) = nums(5:14);
      else
        % se appearance non completa, metti NaN (così lo vedi subito)
        tmp = NaN(1,10);
        if numel(nums) > 4
          tmp(1:numel(nums)-4) = nums(5:end);
        endif
        app(end+1,1:10) = tmp;
      endif
    endif
  endwhile

  fclose(fid);

  meas.local_id = local_id;
  meas.true_id  = true_id;    % NON usarlo per VO (solo debug/eval)
  meas.uv       = uv;         % [col row]
  meas.app      = app;        % Nx10
endfunction


function nums = parse_numbers(line)
  % Estrae tutti i numeri (int/float/scientific) presenti in una stringa
  tokens = regexp(line, '[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', 'match');
  if isempty(tokens)
    nums = [];
  else
    nums = str2double(tokens);
  endif
endfunction

