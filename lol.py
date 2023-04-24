
    # remove lidar points outside detection area and with too low reflectivity
    mask = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]
    
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]  

    # convert sensor coordinates to bev-map coordinates (center is bottom-middle)
    ####### ID_S2_EX1 START #######     
    #######
    print("student task ID_S2_EX1")

    ## step 1 :  compute bev-map discretization by dividing x-range by the bev-image height (see configs)
    bev_discret = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height

    ## step 2 : create a copy of the lidar pcl and transform all metric x-coordinates into bev-image coordinates  
    lidar_pcl_cpy = np.copy(lidar_pcl)
    lidar_pcl_cpy[:, 0] = np.int_(np.floor(lidar_pcl_cpy[:, 0] / bev_discret))  

    # step 3 : perform the same operation as in step 2 for the y-coordinates but make sure that no negative bev-coordinates occur
    lidar_pcl_cpy[:, 1] = np.int_(np.floor(lidar_pcl_cpy[:, 1] / bev_discret) + (configs.bev_width + 1) / 2)

    # NOTE: x and y are rescaled, so rescale z as well to keep the point cloud proportional, otherwise
    # the visualized pcl would be very flat. Don't do this when not trying to visualize, as
    # it will change the magnitude (and thus filtering) of points later. No need to convert to int here.
    # lidar_pcl_cpy[:, 2] = lidar_pcl_cpy[:, 2] / bev_discret

    # step 4 : visualize point-cloud using the function show_pcl from a previous task
    # show_pcl(lidar_pcl_cpy)

    #######
    ####### ID_S2_EX1 END #######     
    
    
    # Compute intensity layer of the BEV map
    ####### ID_S2_EX2 START #######     
    #######
    print("student task ID_S2_EX2")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    intensity_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))

    int_upper_bound = np.quantile(lidar_pcl_cpy[:,3], 0.95)
    lidar_pcl_cpy[lidar_pcl_cpy[:, 3] > int_upper_bound, 3] = int_upper_bound

    # NOTE: the comments here suggest to calculate the intensity of the highest, as opposed to the brightest, point
    # for each pixel. I checked and the original implementation of FPN Resnet does the same, so I followed
    # this convention, though I think working with the brightest point would be more logical for the intensity map.

    # step 2 : re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then -z (use numpy.lexsort)
    idx_height = np.lexsort((-lidar_pcl_cpy[:, 2], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0]))
    lidar_pcl_hei = lidar_pcl_cpy[idx_height]

    ## step 3 : extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    ##          also, store the number of points per x,y-cell in a variable named "counts" for use in the next task
    _, idx_height_unique, counts = np.unique(lidar_pcl_hei[:, 0:2], axis=0, return_index=True, return_counts=True)
    lidar_top_pcl = lidar_pcl_hei[idx_height_unique]

    ## step 4 : assign the intensity value of each unique entry in lidar_top_pcl to the intensity map 
    ##          make sure that the intensity is scaled in such a way that objects of interest (e.g. vehicles) are clearly visible    
    ##          also, make sure that the influence of outliers is mitigated by normalizing intensity on the difference between the max. and min. value within the point cloud
    intensity_map[np.int_(lidar_top_pcl[:, 0]), np.int_(lidar_top_pcl[:, 1])] = lidar_top_pcl[:, 3] / (np.amax(lidar_top_pcl[:, 3])-np.amin(lidar_top_pcl[:, 3]))
    
    # NOTE: the intensity and height maps have values in the range of [0, 1].
    # I originally converted them to [0, 255] here for the visualization, just like in the original exercise, 
    # which broke everything.. it took a while to figure out that is not needed.
    #intensity_map = (intensity_map * 256).astype(np.uint8)

    ## step 5 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    if False:
        while (1):
                cv2.imshow('intensity_map', intensity_map)
                if cv2.waitKey(10) & 0xFF == 27:
                    break
        cv2.destroyAllWindows()

    #######
    ####### ID_S2_EX2 END ####### 


    # Compute height layer of the BEV map
    ####### ID_S2_EX3 START #######     
    #######
    print("student task ID_S2_EX3")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    height_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))

    ## step 2 : assign the height value of each unique entry in lidar_top_pcl to the height map 
    ##          make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    ##          use the lidar_pcl_top data structure from the previous task to access the pixels of the height_map
    height_map[np.int_(lidar_top_pcl[:, 0]), np.int_(lidar_top_pcl[:, 1])] = (lidar_top_pcl[:, 2]) / float(np.abs(configs.lim_z[1] - configs.lim_z[0]))
    #height_map = (height_map * 256).astype(np.uint8)

    ## step 3 : temporarily visualize the height map using OpenCV to make sure that vehicles separate well from the background
    if False:
        while (1):
                cv2.imshow('height_map', height_map)
                if cv2.waitKey(10) & 0xFF == 27:
                    break
        cv2.destroyAllWindows()

    #######
    ####### ID_S2_EX3 END #######       

    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    # NOTE: already calculated above
    # _, idx_unique, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_top_pcl[:, 0]), np.int_(lidar_top_pcl[:, 1])] = normalizedCounts
        
    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps