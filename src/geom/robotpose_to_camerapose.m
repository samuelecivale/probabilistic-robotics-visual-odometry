function T_wc = robotpose_to_camerapose(T_wr, T_rc)
  % T_wr: world <- robot
  % T_rc: robot <- camera   (cam_transform da camera.dat)
  % T_wc: world <- camera
  T_wc = T_wr * T_rc;
endfunction

