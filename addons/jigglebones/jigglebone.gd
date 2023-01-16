tool
extends Spatial

enum Axis {
	X_Plus, Y_Plus, Z_Plus, X_Minus, Y_Minus, Z_Minus
}

export (String) var bone_name: String = ""
export(float, 0.1, 100, 0.1) var stiffness: float = 1
export(float, 0, 100, 0.1) var damping: float = 0
export(bool) var use_gravity: bool = false
export(Vector3) var gravity: Vector3 = Vector3(0, -9.81, 0)
export(Axis) var forward_axis = Axis.Z_Minus

# Previous position
var prev_pos: Vector3 = Vector3()

# Rest length of the distance constraint
var rest_length = 1

# The skeleton that is used
var skeleton: Skeleton = null
# The bone to move
var bone_id: int = -1
# The parent of the bone that is being moved
var bone_id_parent: int = -1

func get_bone_forward_local():
	match forward_axis:
		Axis.X_Plus: return Vector3(1,0,0) 
		Axis.Y_Plus: return Vector3(0,1,0) 
		Axis.Z_Plus: return Vector3(0,0,1)
		Axis.X_Minus: return Vector3(-1,0,0)
		Axis.Y_Minus: return Vector3(0,-1,0) 
		Axis.Z_Minus: return Vector3(0,0,-1) 


func _init_skeleton():
	var obj: Object = get_parent()
	if not obj is Skeleton:
		printerr("[Jigglebones] Error: Jigglebone must be a direct child of a Skeleton node")
		return
	skeleton = obj
	
	if not bone_name:
		printerr("[Jigglebones] Error: Please enter a bone name")
		return
	
	bone_id = skeleton.find_bone(bone_name)
	if bone_id == -1:
		printerr("[Jigglebones] Error: Unknown bone %s - please enter a valid bone name" % bone_name)
		return
	bone_id_parent = skeleton.get_bone_parent(bone_id)


func _ready():
	set_as_toplevel(true)  # Ignore parent transformation
	prev_pos = global_transform.origin


func _process(delta: float):
	_init_skeleton()
	
	if skeleton == null or bone_id == -1:
		push_warning("[Jigglebones] Warning: No skeleton or bone given")
		return
	
	var used_stiffness: float = stiffness
	var used_damping: float = damping
	if Engine.editor_hint:
		used_stiffness *= 1.2
		used_damping *= 1.2
	
	# Note:
	# Local space = local to the bone
	# Object space = local to the skeleton (confusingly called "global" in get_bone_global_pose)
	# World space = global
	
	# See https://godotengine.org/qa/7631/armature-differences-between-bones-custom_pose-transform
	
	var bone_transf_obj: Transform = skeleton.get_bone_global_pose(bone_id) # Object space bone pose
	var bone_transf_world: Transform = skeleton.global_transform * bone_transf_obj
	
	var bone_transf_rest_local: Transform = skeleton.get_bone_rest(bone_id)
	var bone_transf_rest_obj: Transform = skeleton.get_bone_global_pose(bone_id_parent) * bone_transf_rest_local 
	var bone_transf_rest_world: Transform = skeleton.global_transform * bone_transf_rest_obj
	
	############### Integrate velocity (Verlet integration) ##############	
	
	# If not using gravity, apply force in the direction of the bone (so it always wants to point "forward")
	var grav: Vector3 = bone_transf_rest_world.basis.xform(Vector3(0, 0, -1)).normalized() * 9.81
	var vel: Vector3 = (global_transform.origin - prev_pos) / delta
	
	if use_gravity:
		grav = gravity
		
	grav *= used_stiffness
	vel += grav 
	vel -= vel * used_damping * delta  # Damping
	
	prev_pos = global_transform.origin
	global_transform.origin = global_transform.origin + vel * delta
	
	############### Solve distance constraint ##############
	
	var goal_pos: Vector3 = skeleton.to_global(skeleton.get_bone_global_pose(bone_id).origin)
	var new_pos_clamped: Vector3 = goal_pos + (global_transform.origin - goal_pos).normalized() * rest_length
	global_transform.origin = new_pos_clamped 
	
	############## Rotate the bone to point to this object #############

	var diff_vec_local: Vector3 = bone_transf_world.affine_inverse().xform(global_transform.origin).normalized() 
	
	var bone_forward_local = get_bone_forward_local()

	# The axis+angle to rotate on, in local-to-bone space
	var bone_rotate_axis = bone_forward_local.cross(diff_vec_local)
	var bone_rotate_angle = acos(bone_forward_local.dot(diff_vec_local))
	
	if bone_rotate_axis.length() < 1e-3:
		return  # Already aligned, no need to rotate
	
	bone_rotate_axis = bone_rotate_axis.normalized()

	# Bring the axis to object space, WITHOUT translation (so only the BASIS is used) since vectors shouldn't be translated
	var bone_rotate_axis_obj = bone_transf_obj.basis.xform(bone_rotate_axis).normalized()
	var bone_new_transf_obj = Transform(bone_transf_obj.basis.rotated(bone_rotate_axis_obj, bone_rotate_angle), bone_transf_obj.origin)  

	if is_nan(bone_new_transf_obj[0][0]):
		bone_new_transf_obj = Transform()  # Corrupted somehow

	skeleton.set_bone_global_pose_override(bone_id, bone_new_transf_obj, 0.5, true)
	
	# Orient this object to the jigglebone
	global_transform.basis = (skeleton.global_transform * skeleton.get_bone_global_pose(bone_id)).basis

