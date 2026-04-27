string id
string type          # desk, chair, door ...
string label
string room_id       # empty if unassigned
bool is_point
geometry_msgs/Point world_pos          # centroid or point pos
geometry_msgs/Point32[] polygon_world  # empty if is_point
