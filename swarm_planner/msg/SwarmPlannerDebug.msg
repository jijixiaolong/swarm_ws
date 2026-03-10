std_msgs/Header header

int32 self_index
float64 mass
float64 dt_input
float64 dt_used
bool dt_valid_for_update
bool structure_locked
bool used_cfo
bool valid

geometry_msgs/Vector3 payload_position_ned
geometry_msgs/Vector3 payload_velocity_ned
geometry_msgs/Vector3 payload_target_ned
geometry_msgs/Vector3 previous_thrust_vector

geometry_msgs/Vector3[3] uav_positions_ned
geometry_msgs/Vector3[3] uav_velocities_ned

geometry_msgs/Vector3[5] virtual_positions_ned
geometry_msgs/Vector3[5] virtual_velocities_ned
float64[3] beta
float64[25] rest_lengths

geometry_msgs/Vector3 passive_force
geometry_msgs/Vector3 tracking_input
geometry_msgs/Vector3 virtual_acceleration
geometry_msgs/Vector3 mapped_acceleration
geometry_msgs/Vector3 cfo_acceleration
geometry_msgs/Vector3 desired_acceleration
