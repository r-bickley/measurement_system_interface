#Given a RF Filter housing with exposed PCB, this function does the following:
#Within known dimensions of the PCB, measures two 'edge profiles' each on the horizontal and vertical edges
#Identifies the discontinuity associated with the start of the PCB for each profile
#Constructs the linear equation associated with each set of two points
#Finds the intersection of the two linear equations relative to absolute coordinates

def choose_spots(filter_width, filter_length):
    horiz_1 = filter_width / 3
    horiz_2 = 2 * filter_width / 3
    vert_1 = filter_length / 3
    vert_2 = 2 * filter_length / 3
    return (horiz_1, horiz_2, vert_1, vert_2)

def edge_profile(horiz_1, horiz_2, vert_1, vert_2):
    #translate to (horiz_1, 0)
    h_edge_1 = #The coordinates of the second z-distance change between 5mm and 2cm in the profile, as an (x,y) tuple
    #translate to (horiz_2, 0)
    h_edge_2 = #The coordinates of the second z-distance change between 5mm and 2cm in the profile, as an (x,y) tuple
    #translate to (0, vert_1)
    v_edge_1 = #The coordinates of the second z-distance change between 5mm and 2cm in the profile, as an (x,y) tuple
    #translate to (0, vert_2)
    v_edge_2 = #The coordinates of the second z-distance change between 5mm and 2cm in the profile, as an (x,y) tuple
    return (h_edge_1, h_edge_2, v_edge_1, v_edge_2)

def get_origin(h_edge_1, h_edge_2, v_edge_1, v_edge_2):
    #define the horizontal axis line
    delta_y_h = h_edge_1[1]-h_edge_2[1]
    delta_x_h = h_edge_1[0]-h_edge_2[0]
    slope_h = delta_y_h / delta_x_h
    #y=mx+b; b=y-mx, choose one point and solve
    intercept_h = h_edge_1[1] - slope_h * h_edge_1[0]
    #define the vertical axis line
    delta_y_v = v_edge_1[1]-v_edge_2[1]
    delta_x_v = v_edge_1[0]-v_edge_2[0]
    slope_v = delta_y_v / delta_x_v
    #y=mx+b; b=y-mx, choose one point and solve
    intercept_v = v_edge_1[1] - slope_v * v_edge_1[0]
    #define the origin x-coordinate
    x_0 = (intercept_v - intercept_h) / (slope_h -slope_v)
    #define the origin y-coordinate as the horizontal function at x_0
    y_0 = slope_h * x_0 + intercept_h
    return (x_0, y_0)
