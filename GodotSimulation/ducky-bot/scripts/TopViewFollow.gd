extends Camera3D

@export var follow_speed: float = 5.0

var _target: Node3D = null
var _frame: int = 0
var _log: FileAccess = null

func _ready() -> void:
	_target = get_tree().current_scene.get_node_or_null("DuckieBot")
	_log = FileAccess.open("res://debug_path.log", FileAccess.WRITE)

func _process(delta: float) -> void:
	if _target == null:
		return
	var goal = Vector3(_target.global_position.x, global_position.y, _target.global_position.z)
	global_position = global_position.lerp(goal, follow_speed * delta)

	_frame += 1
	if _log != null and _frame % 15 == 0:
		var p = _target.global_position
		var b = _target.global_transform.basis
		_log.store_line("%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f" % [
			_frame, p.x, p.y, p.z,
			b.x.x, b.x.y, b.x.z,
			b.y.x, b.y.y, b.y.z,
			b.z.x, b.z.y, b.z.z
		])
		_log.flush()
