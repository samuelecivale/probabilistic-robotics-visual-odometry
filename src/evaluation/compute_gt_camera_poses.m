function T_wc_gt = compute_gt_camera_poses(camera, traj)
    n = size(traj.gt_pose, 1);
    T_wc_gt = zeros(4, 4, n);

    for k = 1:n
        T_wr = pose2d_to_se3(traj.gt_pose(k, :));
        T_wc_gt(:, :, k) = T_wr * camera.cam_transform;
    end
end
