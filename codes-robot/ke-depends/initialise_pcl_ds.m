%initialises pcl data structures

pcl.head.x = nan(nFrames,size(robot(HEAD).Patch.Vertices,1));
pcl.head.y = nan(nFrames,size(robot(HEAD).Patch.Vertices,1));
pcl.head.z = nan(nFrames,size(robot(HEAD).Patch.Vertices,1));
pcl.w1.x = nan(nFrames,size(robot(W1).Patch.Vertices,1));
pcl.w1.y = nan(nFrames,size(robot(W1).Patch.Vertices,1));
pcl.w1.z = nan(nFrames,size(robot(W1).Patch.Vertices,1));
pcl.w2.x = nan(nFrames,size(robot(W2).Patch.Vertices,1));
pcl.w2.y = nan(nFrames,size(robot(W2).Patch.Vertices,1));
pcl.w2.z = nan(nFrames,size(robot(W2).Patch.Vertices,1));

if(exist('t_output','var'))
    pcl.v_world.head.x = nan(length(t_output),size(robot(HEAD).Patch.Vertices,1));
    pcl.v_world.head.y = nan(length(t_output),size(robot(HEAD).Patch.Vertices,1));
    pcl.v_world.head.z = nan(length(t_output),size(robot(HEAD).Patch.Vertices,1));
    pcl.v_world.w1.x = nan(length(t_output),size(robot(W1).Patch.Vertices,1));
    pcl.v_world.w1.y = nan(length(t_output),size(robot(W1).Patch.Vertices,1));
    pcl.v_world.w1.z = nan(length(t_output),size(robot(W1).Patch.Vertices,1));
    pcl.v_world.w2.x = nan(length(t_output),size(robot(W2).Patch.Vertices,1));
    pcl.v_world.w2.y = nan(length(t_output),size(robot(W2).Patch.Vertices,1));
    pcl.v_world.w2.z = nan(length(t_output),size(robot(W2).Patch.Vertices,1));
    pcl.v_body.head.x = nan(length(t_output),size(robot(HEAD).Patch.Vertices,1));
    pcl.v_body.head.y = nan(length(t_output),size(robot(HEAD).Patch.Vertices,1));
    pcl.v_body.head.z = nan(length(t_output),size(robot(HEAD).Patch.Vertices,1));
    pcl.v_body.w1.x = nan(length(t_output),size(robot(W1).Patch.Vertices,1));
    pcl.v_body.w1.y = nan(length(t_output),size(robot(W1).Patch.Vertices,1));
    pcl.v_body.w1.z = nan(length(t_output),size(robot(W1).Patch.Vertices,1));
    pcl.v_body.w2.x = nan(length(t_output),size(robot(W2).Patch.Vertices,1));
    pcl.v_body.w2.y = nan(length(t_output),size(robot(W2).Patch.Vertices,1));
    pcl.v_body.w2.z = nan(length(t_output),size(robot(W2).Patch.Vertices,1));
end