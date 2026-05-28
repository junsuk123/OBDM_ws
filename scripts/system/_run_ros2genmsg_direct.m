repoRoot = fileparts(fileparts(mfilename('fullpath')));
pkgPath = fullfile(repoRoot, 'ardu_ws', 'src', 'ros_gz', 'ros_gz_interfaces');
disp(['Attempting ros2genmsg for: ' pkgPath]);
ros2genmsg(pkgPath);
disp('ros2genmsg finished');
