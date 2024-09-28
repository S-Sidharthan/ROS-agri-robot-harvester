def my_depth_to_cloud():
    pc = rs.pointcloud()
    points = rs.points()
 
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe_profile = pipeline.start(config)
 
    for i in range(100):
        data = pipeline.wait_for_frames()
 
        depth = data.get_depth_frame()
        color = data.get_color_frame()
 
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()
 
    colorful = np.asanyarray(color.get_data())
    colorful=colorful.reshape(-1,3)
 
    pc.map_to(color)
    points = pc.calculate(depth)
 
    # Get vertex coordinates
    vtx = np.asanyarray(points.get_vertices())
    # Get texture coordinates
    #tex = np.asanyarray(points.get_texture_coordinates())
 
 
    with open('could.txt','w') as f:
        for i in range(len(vtx)):
            f.write(str(np.float(vtx[i][0])*1000)+' '+str(np.float(vtx[i][1])*1000)+' '+str(np.float(vtx[i][2])*1000)+' '+str(np.float(colorful[i][0]))+' '+str(np.float(colorful[i][1]))+' '+str(np.float(colorful[i][2]))+'\n')