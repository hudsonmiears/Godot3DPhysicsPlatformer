extends RigidDynamicBody3D

@export var ride_height: float = 0.5
@export var ride_spring_strength: float
@export var ride_spring_damper: float
@export var upright_spring_strength: float
@export var upright_spring_damper: float
@export var max_speed: float
@export var acceleration: float
@export var acceleration_factor_from_dot: Curve
@export var max_acceleration: float
@export var max_acceleration_force_factor_from_dot: Curve

@export var jump_force: float

@onready var raycast_node = $RayCast3D
@onready var timer_node = $DisableUprightTimer
var camera_node
var look_at: Vector3 = Vector3.FORWARD
var cur_goal_vel: Vector3 = Vector3.ZERO
var is_grounded: bool = false
var disable_upright = false

# Called when the node enters the scene tree for the first time.
func _ready():
	camera_node = get_node("../Camera3D")
	look_at = global_rotation
	

func _integrate_forces(state):
	var move_input = Input.get_vector("move_left", "move_right", "move_forward", "move_back")
	var jump_input = Input.is_action_just_pressed("jump")
	var input_forward_from_camera = get_move_forward(move_input)
	is_grounded = raycast_node.is_colliding()
	if is_grounded:
		if (gravity_scale != 1):
			gravity_scale = 1
		if jump_input:
			jump(state)
		elif disable_upright == false:
			var ride_force = get_ride_force(state.linear_velocity)
			state.apply_force(ride_force * Vector3.DOWN)
	else:
		if (linear_velocity.y < 4 && gravity_scale == 1):
			gravity_scale = 4
	if (move_input.length() > 0):
		look_at = -input_forward_from_camera
	var upright_force = get_upright_force(state.angular_velocity, Basis.looking_at(look_at, Vector3.UP).get_rotation_quaternion())
	state.apply_torque(upright_force)
	var linear_vel = state.linear_velocity
	var needed_acceleration = get_move_force(input_forward_from_camera, state.linear_velocity, state.step)
	state.apply_central_force(needed_acceleration)
	if Input.get_action_raw_strength("debug_break") > 0:
		print_debug("break pressed")


func get_ride_force(linear_velocity: Vector3) -> float:
	if (raycast_node.is_colliding() == false):
		return 0.0
	var ray_dir_vel = linear_velocity.dot(raycast_node.target_position)
	var collision_point = raycast_node.get_collision_point()
	var collision_to_node = collision_point - raycast_node.global_transform.origin
	var distance = collision_to_node.length()
	var x = distance - ride_height
	var spring_force = (x * ride_spring_strength) - (ray_dir_vel * ride_spring_damper)
	return spring_force


func get_upright_force(angular_velocity: Vector3, upright_target_rotation: Quaternion):
	var currentRot = global_transform.basis.get_rotation_quaternion()
	var to_goal = get_shortest_rotation(upright_target_rotation, currentRot)
	var rot_axis = to_goal.get_axis()
	var rot_degrees = to_goal.get_angle()
	var rot_radians = deg2rad(rot_degrees)
	rot_axis = rot_axis.normalized()
	var spring_force = (rot_axis * (rot_radians * upright_spring_strength)) - (angular_velocity * upright_spring_damper)
	return spring_force


func get_move_forward(move_input: Vector2) -> Vector3:
	var camera_right = camera_node.global_transform.basis.x.normalized()
	var camera_forward = camera_node.global_transform.basis.z.normalized()
	var forward_direction = camera_forward * move_input.y + camera_right * move_input.x
	return Vector3(forward_direction.x, 0, forward_direction.z).normalized()


func get_move_force(target_direction: Vector3, linear_velocity: Vector3, delta: float) -> Vector3:
	var vel_dot = target_direction.normalized().dot(cur_goal_vel.normalized())
	var vel_dot_0_1 = remap_to_0_1(vel_dot)
	var accel = acceleration * acceleration_factor_from_dot.interpolate(vel_dot_0_1)
	var goal_vel = target_direction.normalized() * max_speed
	cur_goal_vel = cur_goal_vel.move_toward(goal_vel, accel * delta)
	var needed_accel = (cur_goal_vel - linear_velocity) / delta
	var max_accel_factor = max_acceleration_force_factor_from_dot.interpolate(vel_dot_0_1)
	var max_accel = max_acceleration * max_accel_factor
	var clamped = needed_accel.limit_length(max_accel)
	if Input.get_action_raw_strength("debug_break") > 0:
		print_debug("break pressed")
	return clamped


func get_shortest_rotation(to: Quaternion, from: Quaternion) -> Quaternion:
	if to.dot(from) < 0:
		return to * (from * -1).inverse()
	else:
		return to * from.inverse()


func remap_to_0_1(val: float, old_min: float = -1.0, old_max: float = 1.0) -> float:
	var new_val = (val - old_min) / (old_max - old_min)
	return new_val


#func look_follow(state, current_transform, target_position):
#	var up_dir = Vector3(0, 1, 0)
#	var cur_dir = current_transform.basis.xform(Vector3(0, 0, 1))
#	var target_dir = (target_position - current_transform.origin).normalized()
#	var rotation_angle = acos(cur_dir.x) - acos(target_dir.x)
#
#	state.set_angular_velocity(up_dir * (rotation_angle / state.get_step()))
	
func jump(state: PhysicsDirectBodyState3D):
	disable_upright = true
	state.apply_central_impulse(jump_force * Vector3.UP)
#	state.linear_velocity = Vector3(state.linear_velocity.x, jump_force, state.linear_velocity.z)
	timer_node.start()


func _on_disable_upright_timer_timeout():
	disable_upright = false
