function cam = load_camera(filename)
  fid = fopen(filename, "r");
  if fid < 0, error("Non riesco ad aprire: %s", filename); endif

  cam.K = [];
  cam.T_cr = eye(4);     % cam_transform (camera wrt robot), ignorabile per VO
  cam.z_near = NaN;
  cam.z_far  = NaN;
  cam.width  = NaN;
  cam.height = NaN;

  mode = "";  % "", "K", "T"
  K_rows = [];
  T_rows = [];

  while true
    line = fgetl(fid);
    if ~ischar(line), break; endif
    line = strtrim(line);
    if isempty(line), continue; endif

    if startsWith(line, "camera matrix")
      mode = "K"; K_rows = [];
      continue;
    elseif startsWith(line, "cam_transform")
      mode = "T"; T_rows = [];
      continue;
    elseif startsWith(line, "z_near:")
      cam.z_near = str2double(strtrim(strrep(line, "z_near:", "")));
      mode = "";
      continue;
    elseif startsWith(line, "z_far:")
      cam.z_far = str2double(strtrim(strrep(line, "z_far:", "")));
      mode = "";
      continue;
    elseif startsWith(line, "width:")
      cam.width = str2double(strtrim(strrep(line, "width:", "")));
      mode = "";
      continue;
    elseif startsWith(line, "height:")
      cam.height = str2double(strtrim(strrep(line, "height:", "")));
      mode = "";
      continue;
    endif

    nums = sscanf(line, "%f")';
    if isempty(nums), continue; endif

    if strcmp(mode, "K")
      if numel(nums) >= 3
        K_rows(end+1, 1:3) = nums(1:3); %#ok<AGROW>
      endif
    elseif strcmp(mode, "T")
      if numel(nums) >= 4
        T_rows(end+1, 1:4) = nums(1:4); %#ok<AGROW>
      endif
    endif
  endwhile
  fclose(fid);

  if rows(K_rows) ~= 3 || columns(K_rows) ~= 3
    error("camera.dat: camera matrix non letta correttamente");
  endif
  cam.K = K_rows;

  if rows(T_rows) == 4 && columns(T_rows) == 4
    cam.T_cr = T_rows;
  endif
endfunction

