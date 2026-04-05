function traj = load_trajectory(filename)
    data = load(filename);
    assert(size(data, 2) == 7, 'trajectory.dat must have 7 columns');

    traj = struct();
    traj.pose_id = data(:, 1);
    traj.odom_pose = data(:, 2:4);
    traj.gt_pose = data(:, 5:7);
end
