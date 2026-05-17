extends Node
## Lua 决策端 TCP sidecar 客户端。
## 负责与 C++ 模拟器进程（sim_sidecar）建立 TCP 长连接，完成：
## 1. 上行：周期性发送机器人位姿/资源状态 (sim.input)
## 2. 下行：接收 blackboard、decision_state、nav_target 及各种遥控指令
## 3. 超控面板：手动编辑 blackboard 的 HP/Bullet/Stage/Switch 值
## 4. 自动补给：机器人进入补给区时自动回血
## 5. 将 Lua 指令转发到 AI 机器人（底盘模式、云台控制、速度超控等）

## TCP 服务器地址。
@export var host := "127.0.0.1"
## TCP 服务器端口。
@export var port := 34567
## 上行状态推送频率 (Hz)。
@export var send_hz := 60.0
## 是否优先使用矩形补给区判定。
@export var use_resupply_rect := true
## 矩形补给区左下角坐标。
@export var resupply_rect_min := Vector2(0.0, 0.0)
## 矩形补给区右上角坐标。
@export var resupply_rect_max := Vector2(4.0, 4.0)
## 旧补给点回退半径。
@export var resupply_radius := 0.8
## 每秒回复血量。
@export var resupply_health_rate := 100.0
## 被动金币发放间隔（秒）。
@export var passive_gold_interval := 30.0
## 每次被动发放的金币数量。
@export var passive_gold_amount := 50
## 基地初始血量。
@export var initial_base_health := 5000
## 前哨站初始血量。
@export var initial_outpost_health := 1500
## 比赛初始倒计时（秒，7分钟）。
@export var initial_remaining_time := 420.0
## AI 机器人节点路径。
@export var robot_path: NodePath
## 导航目标点节点路径。
@export var target_point_path: NodePath
## 敌方机器人节点路径。
@export var enemy_path: NodePath

const LOOT_OVERLAY_SCRIPT := preload("res://scene/loot_overlay_3d.gd")
const LOOT_FLOW_VIEW_SCRIPT := preload("res://scene/loot_flow_view.gd")

@onready var robot: CharacterBody3D = get_node(robot_path)
@onready var target_point: Node3D = get_node(target_point_path)

const DEBUG_PANEL_SIZE := Vector2(2000, 1200)
## blackboard 面板显示字段。
const BLACKBOARD_USER_DISPLAY_FIELDS := [
	"health", "bullet", "gold", "x", "y", "yaw", "auto_aim_should_control",
	"chassis_power_limit", "chassis_power", "chassis_buffer_energy", "chassis_output_status",
	"shooter_cooling", "shooter_heat_limit", "bullet_42mm", "fortress_17mm_bullet",
	"initial_speed", "shoot_timestamp"
]
const BLACKBOARD_GAME_DISPLAY_FIELDS := [
	"stage", "sync_timestamp", "remaining_time", "gold_coin", "base_health", "outpost_health",
	"hero_health", "infantry_1_health", "infantry_2_health", "engineer_health",
	"exchangeable_ammunition_quantity", "our_dart_nmber_of_hits", "fortress_occupied",
	"big_energy_mechanism_activated", "small_energy_mechanism_activated", "robot_id",
	"can_confirm_free_revive", "can_exchange_instant_revive", "instant_revive_cost",
	"exchanged_bullet", "remote_bullet_exchange_count", "sentry_mode",
	"energy_mechanism_activatable"
]
const BLACKBOARD_MAP_COMMAND_DISPLAY_FIELDS := [
	"x", "y", "keyboard", "target_robot_id", "source", "sequence"
]
## 游戏阶段枚举值。
const STAGE_VALUES := ["UNKNOWN", "NOT_START", "STARTED", "ENDED"]
## 拨杆开关枚举值。
const SWITCH_VALUES := ["UNKNOWN", "UP", "MIDDLE", "DOWN"]
## 供其他节点查询 Lua Sim 面板状态的分组名。
const LUA_SIM_UI_GROUP := "lua_sim_ui"

## TCP 连接对象。
var tcp := StreamPeerTCP.new()
## 当前是否已连接。
var connected := false
## 接收缓冲区（行协议：\n 分隔的 JSON）。
var rx_buffer := ""
## 本地模拟时间 (自连接起累计，秒)。
var sim_time := 0.0
## 上行发送累计时间 (秒)。
var send_accum := 0.0

## 本地缓存的 blackboard 副本。
var blackboard: Dictionary = {}
## 最近一次决策状态快照。
var decision_state: Dictionary = {}
## 最近一次 Loot 语义监控快照。
var loot_state: Dictionary = {}
## 最近一次 sidecar 宿主运行时状态。
var runtime_state: Dictionary = {}
## Loot 快照版本号（跟随 blackboard 版本）。
var loot_rev := -1
## blackboard 是否已完成首次接收。
var blackboard_ready := false
## blackboard 版本号（去重/乱序过滤）。
var blackboard_rev := -1

## 超控模式是否激活（手动或自动补给）。
var override_enabled := false
## 手动超控面板是否开启。
var manual_override_enabled := false
## 自动补给超控是否激活。
var auto_resupply_override_enabled := false
## 被动金币同步时使用的临时超控状态。
var auto_gold_override_enabled := false
## 超控补丁的序号。
var override_rev := 0
## UI 控件防递归更新标志。
var ui_updating := false
## 机器人是否在补给区内。
var in_resupply_zone := false
## 自动补给是否正在执行。
var auto_resupply_active := false
## 到补给点的距离（米）。
var resupply_distance: Variant = null
## 血量回复累计缓冲区。
var auto_resupply_health_buffer := 0.0
## 距离下一次被动金币发放的剩余时间。
var passive_gold_left := passive_gold_interval
## 是否已从 blackboard 初始化机器人资源。
var robot_resource_initialized := false
## 本地维护的基地血量（通过 sim.input 同步到 Lua）。
var sim_base_health := initial_base_health
## 本地维护的前哨站血量（通过 sim.input 同步到 Lua）。
var sim_outpost_health := initial_outpost_health
## 本地维护的比赛剩余时间（秒）。
var sim_remaining_time := initial_remaining_time
## 本地维护的可兑换弹药量。
var sim_exchangeable_ammunition_quantity := 0
## 本地维护的己方飞镖命中次数。
var sim_our_dart_nmber_of_hits := 0
## 本地维护的堡垒占领状态。
var sim_fortress_occupied := false
## 本地维护的大能量机关激活状态。
var sim_big_energy_mechanism_activated := false
## 本地维护的小能量机关激活状态。
var sim_small_energy_mechanism_activated := false
## 比赛是否已进入 STARTED，用于驱动倒计时。
var competition_started := false

## 调试 UI 控件引用。
var lua_sim_canvas: CanvasLayer
var lua_sim_panel: Panel
var lua_sim_tabs: TabContainer
var lua_sim_panel_open := false
var lua_sim_saved_mouse_mode := Input.MOUSE_MODE_VISIBLE
var loot_flow_view: Control
var blackboard_label: RichTextLabel
var override_toggle: CheckButton
var edit_box: VBoxContainer
var hp_spin: SpinBox
var bullet_spin: SpinBox
var gold_spin: SpinBox
var chassis_power_limit_spin: SpinBox
var chassis_power_spin: SpinBox
var chassis_buffer_energy_spin: SpinBox
var shooter_cooling_spin: SpinBox
var shooter_heat_limit_spin: SpinBox
var bullet_42mm_spin: SpinBox
var fortress_17mm_bullet_spin: SpinBox
var initial_speed_spin: SpinBox
var shoot_timestamp_spin: SpinBox
var base_hp_spin: SpinBox
var outpost_hp_spin: SpinBox
var hero_hp_spin: SpinBox
var infantry_1_hp_spin: SpinBox
var infantry_2_hp_spin: SpinBox
var engineer_hp_spin: SpinBox
var remaining_time_spin: SpinBox
var sync_timestamp_spin: SpinBox
var exchangeable_ammo_spin: SpinBox
var exchanged_bullet_spin: SpinBox
var robot_id_spin: SpinBox
var instant_revive_cost_spin: SpinBox
var remote_bullet_exchange_count_spin: SpinBox
var dart_hits_spin: SpinBox
var map_command_x_spin: SpinBox
var map_command_y_spin: SpinBox
var map_command_keyboard_spin: SpinBox
var map_command_target_robot_id_spin: SpinBox
var map_command_source_spin: SpinBox
var map_command_sequence_spin: SpinBox
var fortress_occupied_toggle: CheckButton
var big_energy_mechanism_toggle: CheckButton
var small_energy_mechanism_toggle: CheckButton
var chassis_output_status_toggle: CheckButton
var can_confirm_free_revive_toggle: CheckButton
var can_exchange_instant_revive_toggle: CheckButton
var energy_mechanism_activatable_toggle: CheckButton
var stage_select: OptionButton
var rswitch_select: OptionButton
var lswitch_select: OptionButton
var gold_badge_value_label: Label
var base_badge_value_label: Label
var outpost_badge_value_label: Label
var gold_badge_panel: Control
var base_badge_panel: Control
var outpost_badge_panel: Control
var loot_overlay: Node3D
var runtime_label: RichTextLabel


func _ready() -> void:
	add_to_group(LUA_SIM_UI_GROUP)
	# 将敌方节点注入 AI 机器人作为跟踪目标。
	_bind_enemy_target()
	# 构建 Loot 3D 实时监控层。
	_build_loot_overlay()
	# 构建调试 UI 面板。
	_build_debug_ui()
	_refresh_display()

	# 发起 TCP 连接（异步）。
	var err := tcp.connect_to_host(host, port)
	if err != OK:
		push_error("connect_to_host failed: %s" % err)


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey):
		return

	var key_event := event as InputEventKey
	if not key_event.pressed or key_event.echo:
		return
	if key_event.keycode != KEY_TAB and key_event.physical_keycode != KEY_TAB:
		return

	_set_lua_sim_panel_open(not lua_sim_panel_open)
	get_viewport().set_input_as_handled()


func is_panel_open() -> bool:
	return lua_sim_panel_open


func _set_lua_sim_panel_open(open: bool) -> void:
	if lua_sim_panel_open == open:
		return

	if open:
		lua_sim_saved_mouse_mode = Input.mouse_mode

	lua_sim_panel_open = open
	if lua_sim_panel != null:
		lua_sim_panel.visible = open
	_set_overlay_badges_visible(not open)

	if open:
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	else:
		Input.mouse_mode = lua_sim_saved_mouse_mode


func _process(_delta: float) -> void:
	# 轮询 TCP 状态机，处理连接/断开/接收事件。
	tcp.poll()
	var status := tcp.get_status()

	# --- 首次建立连接 ---
	if status == StreamPeerTCP.STATUS_CONNECTED and not connected:
		connected = true
		rx_buffer = ""
		send_accum = 0.0
		sim_time = 0.0
		blackboard = {}
		decision_state = {}
		loot_state = {}
		blackboard_ready = false
		blackboard_rev = -1
		loot_rev = -1
		if loot_flow_view != null and loot_flow_view.has_method("reset"):
			loot_flow_view.call("reset")
		override_enabled = false
		manual_override_enabled = false
		auto_resupply_override_enabled = false
		auto_gold_override_enabled = false
		override_rev = 0
		in_resupply_zone = false
		auto_resupply_active = false
		resupply_distance = null
		auto_resupply_health_buffer = 0.0
		passive_gold_left = passive_gold_interval
		robot_resource_initialized = false
		sim_base_health = initial_base_health
		sim_outpost_health = initial_outpost_health
		sim_remaining_time = initial_remaining_time
		sim_exchangeable_ammunition_quantity = 0
		sim_our_dart_nmber_of_hits = 0
		sim_fortress_occupied = false
		sim_big_energy_mechanism_activated = false
		sim_small_energy_mechanism_activated = false
		competition_started = false
		if override_toggle != null:
			override_toggle.set_pressed_no_signal(false)
		if edit_box != null:
			edit_box.visible = false
		# 发送 sim.hello 握手消息，声明协议和模式。
		_send_json({
			"type": "sim.hello",
			"protocol": 1,
			"mode": "lua_sim_v1"
		})
		_sync_override_mode(true)
		_refresh_display()
		print("sim sidecar connected")

	# --- 连接断开 ---
	if status != StreamPeerTCP.STATUS_CONNECTED:
		if connected:
			connected = false
			blackboard = {}
			decision_state = {}
			loot_state = {}
			blackboard_ready = false
			loot_rev = -1
			if loot_flow_view != null and loot_flow_view.has_method("reset"):
				loot_flow_view.call("reset")
			override_enabled = false
			manual_override_enabled = false
			auto_resupply_override_enabled = false
			auto_gold_override_enabled = false
			override_rev = 0
			in_resupply_zone = false
			auto_resupply_active = false
			resupply_distance = null
			auto_resupply_health_buffer = 0.0
			passive_gold_left = passive_gold_interval
			robot_resource_initialized = false
			sim_base_health = initial_base_health
			sim_outpost_health = initial_outpost_health
			sim_remaining_time = initial_remaining_time
			sim_exchangeable_ammunition_quantity = 0
			sim_our_dart_nmber_of_hits = 0
			sim_fortress_occupied = false
			sim_big_energy_mechanism_activated = false
			sim_small_energy_mechanism_activated = false
			competition_started = false
			if override_toggle != null:
				override_toggle.set_pressed_no_signal(false)
			if edit_box != null:
				edit_box.visible = false
			rx_buffer = ""
			_refresh_display()
			print("sim sidecar disconnected")
		return

	# --- 接收消息：行协议（\n 分隔的 JSON 行）---
	var available := tcp.get_available_bytes()
	if available <= 0:
		return

	rx_buffer += tcp.get_utf8_string(available)
	while true:
		var pos := rx_buffer.find("\n")
		if pos < 0:
			break

		var line := rx_buffer.substr(0, pos).strip_edges()
		rx_buffer = rx_buffer.substr(pos + 1)
		if line.is_empty():
			continue

		var msg = JSON.parse_string(line)
		if typeof(msg) != TYPE_DICTIONARY:
			push_warning("invalid message line: %s" % line)
			continue

		_handle_message(msg)


## 物理帧：更新模拟时间、自动补给状态，并按固定频率发送 input 状态。
func _physics_process(delta: float) -> void:
	if not connected:
		return

	sim_time += delta
	send_accum += delta
	_tick_competition_clock(delta)
	_tick_passive_gold(delta)
	_update_auto_resupply(delta)

	if send_accum >= 1.0 / send_hz:
		send_accum = 0.0
		_send_input()


func _tick_passive_gold(delta: float) -> void:
	if not blackboard_ready or passive_gold_interval <= 0.0 or passive_gold_amount <= 0:
		return

	passive_gold_left -= delta
	if passive_gold_left > 0.0:
		return

	var period_count: int = int(floor(-passive_gold_left / passive_gold_interval)) + 1
	passive_gold_left += passive_gold_interval * float(period_count)

	var user: Dictionary = _get_dict_value(blackboard, "user")
	var current_gold: int = int(round(_get_numeric_value(user.get("gold", 0), 0.0)))
	var next_gold: int = current_gold + passive_gold_amount * period_count
	var patch_user: Dictionary = {"gold": next_gold}
	var patch: Dictionary = {"user": patch_user}
	_apply_local_patch(patch)
	_apply_local_patch({"game": {"gold_coin": next_gold}})
	_apply_robot_user_patch(patch_user)
	_refresh_display()


func _tick_competition_clock(delta: float) -> void:
	if not competition_started:
		return
	sim_remaining_time = max(sim_remaining_time - delta, 0.0)
	_apply_local_patch({"game": {"remaining_time": sim_remaining_time}})


## 输入处理：检测"开始决策"动作 (sim_start_decision) 或回车键。
func _unhandled_input(event: InputEvent) -> void:
	if not connected:
		return

	var start_pressed := InputMap.has_action("sim_start_decision") and event.is_action_pressed("sim_start_decision")
	if not start_pressed and event is InputEventKey:
		var key_event := event as InputEventKey
		start_pressed = key_event.pressed and key_event.keycode == KEY_ENTER

	if start_pressed:
		_send_start_decision()


## 消息分发：根据 type 字段路由到对应处理器。
func _handle_message(msg: Dictionary) -> void:
	var t := str(msg.get("type", ""))

	# blackboard 全量同步（带版本号去重）。
	if t == "sim.blackboard":
		var rev := int(msg.get("bb_rev", -1))
		if rev <= blackboard_rev:
			return

		var payload = msg.get("blackboard", {})
		if typeof(payload) != TYPE_DICTIONARY:
			push_warning("sim.blackboard missing dictionary payload")
			return

		blackboard_rev = rev
		blackboard = payload.duplicate(true)
		blackboard_ready = true
		_ensure_gold_field_initialized()
		# 首次收到 blackboard 时初始化机器人资源。
		_sync_robot_resources_from_blackboard_once()
		# 将 blackboard 控制字段同步到 UI 控件。
		_sync_controls_from_blackboard()
		_refresh_display()
		return

	# 决策层状态快照。
	if t == "sim.decision_state":
		var payload = msg.get("state", {})
		if typeof(payload) == TYPE_DICTIONARY:
			decision_state = payload.duplicate(true)
			_refresh_display()
		return

	if t == "sim.runtime_state":
		var payload = msg.get("state", {})
		if typeof(payload) == TYPE_DICTIONARY:
			runtime_state = payload.duplicate(true)
			_refresh_display()
		return

	if t == "sim.resource_sync":
		var user_patch := {
			"bullet": int(round(_get_numeric_value(msg.get("bullet", 0), 0.0))),
			"gold": int(round(_get_numeric_value(msg.get("gold", 0), 0.0))),
		}
		var patch := {
			"user": user_patch,
			"game": {
				"gold_coin": user_patch["gold"],
			}
		}
		_apply_local_patch(patch)
		_apply_robot_user_patch(user_patch)
		_sync_controls_from_blackboard()
		_refresh_display()
		return

	# Loot 语义监控快照。
	if t == "loot.snapshot":
		var rev := int(msg.get("bb_rev", -1))
		if rev <= loot_rev:
			return

		var payload = msg.get("loot", {})
		if typeof(payload) != TYPE_DICTIONARY:
			push_warning("loot.snapshot missing dictionary payload")
			return

		loot_rev = rev
		loot_state = payload.duplicate(true)
		if loot_overlay != null and loot_overlay.has_method("update_loot"):
			loot_overlay.call("update_loot", loot_state)
		if loot_flow_view != null and loot_flow_view.has_method("update_loot"):
			loot_flow_view.call("update_loot", loot_state)
		_refresh_display()
		return

	# 导航目标点更新（Lua 下发）。
	if t == "sim.nav_target":
		var x := float(msg.get("x", target_point.global_position.z))
		var y := float(msg.get("y", target_point.global_position.x))
		var p := target_point.global_position
		p.x = y
		p.z = x
		target_point.global_position = p
		_refresh_display()
		return

	# Lua 日志透传。
	if t == "sim.log":
		print("[lua/%s] %s" % [str(msg.get("level", "info")), str(msg.get("message", ""))])
		return

	# 底盘模式切换 (idle / spin)。
	if t == "sim.chassis_mode":
		_apply_robot_chassis_mode(str(msg.get("mode", "")))
		return

	if t == "sim.controller_mode":
		_apply_robot_controller_mode(str(msg.get("mode", "")))
		return

	if t == "sim.navigation_enabled":
		_apply_robot_navigation_enabled(_get_boolean_value(msg.get("enabled", false), false))
		return

	# 云台控制源切换 (manual / scan / auto)。
	if t == "sim.gimbal_dominator":
		_apply_robot_gimbal_dominator(str(msg.get("name", "")))
		return

	if t == "sim.autoaim_enabled":
		_apply_robot_autoaim_enabled(_get_boolean_value(msg.get("enabled", false), false))
		return

	# 云台手动朝向设定。
	if t == "sim.gimbal_direction":
		_apply_robot_gimbal_direction(float(msg.get("angle", 0.0)))
		return

	# 底盘速度超控（遥控）。
	if t == "sim.chassis_vel":
		_apply_robot_chassis_vel(float(msg.get("x", 0.0)), float(msg.get("y", 0.0)))
		return

	print("unknown msg: ", msg)


## 上行：发送 sim.input，包含机器人位姿和资源状态。
func _send_input() -> void:
	var resource := _get_robot_resource_state()
	var sim_status_payload := {
		"in_resupply_zone": in_resupply_zone,
	}
	var input_payload := {
		"user": {
			"x": robot.global_position.z,
			"y": robot.global_position.x,
			"yaw": robot.global_rotation.y,
			"health": int(resource.get("health", 0)),
			"bullet": int(resource.get("bullet", 0)),
			"gold": int(resource.get("gold", 0)),
			"chassis_power_limit": _get_numeric_value(resource.get("chassis_power_limit", 0.0), 0.0),
			"chassis_power": _get_numeric_value(resource.get("chassis_power", 0.0), 0.0),
			"chassis_buffer_energy": _get_numeric_value(resource.get("chassis_buffer_energy", 0.0), 0.0),
			"chassis_output_status": _get_boolean_value(resource.get("chassis_output_status", false), false),
			"shooter_cooling": _get_numeric_value(resource.get("shooter_cooling", 0.0), 0.0),
			"shooter_heat_limit": _get_numeric_value(resource.get("shooter_heat_limit", 0.0), 0.0),
			"bullet_42mm": _get_numeric_value(resource.get("bullet_42mm", 0.0), 0.0),
			"fortress_17mm_bullet": _get_numeric_value(resource.get("fortress_17mm_bullet", 0.0), 0.0),
			"initial_speed": _get_numeric_value(resource.get("initial_speed", 0.0), 0.0),
			"shoot_timestamp": _get_numeric_value(resource.get("shoot_timestamp", 0.0), 0.0),
			"auto_aim_should_control": bool(resource.get("auto_aim_should_control", false)),
		},
		"game": {
			"base_health": sim_base_health,
			"outpost_health": sim_outpost_health,
			"gold_coin": _get_current_gold_value(),
			"sync_timestamp": sim_time,
			"hero_health": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("hero_health", 0), 0.0))),
			"infantry_1_health": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("infantry_1_health", 0), 0.0))),
			"infantry_2_health": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("infantry_2_health", 0), 0.0))),
			"engineer_health": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("engineer_health", 0), 0.0))),
			"remaining_time": sim_remaining_time,
			"exchangeable_ammunition_quantity": sim_exchangeable_ammunition_quantity,
			"our_dart_nmber_of_hits": sim_our_dart_nmber_of_hits,
			"fortress_occupied": sim_fortress_occupied,
			"big_energy_mechanism_activated": sim_big_energy_mechanism_activated,
			"small_energy_mechanism_activated": sim_small_energy_mechanism_activated,
			"robot_id": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("robot_id", 0), 0.0))),
			"can_confirm_free_revive": _get_boolean_value(_get_dict_value(blackboard, "game").get("can_confirm_free_revive", false), false),
			"can_exchange_instant_revive": _get_boolean_value(_get_dict_value(blackboard, "game").get("can_exchange_instant_revive", false), false),
			"instant_revive_cost": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("instant_revive_cost", 0), 0.0))),
			"exchanged_bullet": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("exchanged_bullet", 0), 0.0))),
			"remote_bullet_exchange_count": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("remote_bullet_exchange_count", 0), 0.0))),
			"sentry_mode": int(round(_get_numeric_value(_get_dict_value(blackboard, "game").get("sentry_mode", 0), 0.0))),
			"energy_mechanism_activatable": _get_boolean_value(_get_dict_value(blackboard, "game").get("energy_mechanism_activatable", false), false),
		},
		"map_command": _get_dict_value(blackboard, "map_command"),
		"meta": {
			"timestamp": sim_time,
		},
		"sim_status": sim_status_payload,
	}

	_send_json({
		"type": "sim.input",
		"input": input_payload,
	})


## 下行：发送 sim.command (start_decision)，触发 Lua 决策开始运行。
func _send_start_decision() -> void:
	if not connected:
		return

	_begin_competition_session()

	_send_json({
		"type": "sim.command",
		"command": "start_decision",
	})


## 下行：发送 sim.override_mode 切换超控状态。
func _send_override_mode(enabled: bool) -> void:
	_send_json({
		"type": "sim.override_mode",
		"enabled": enabled,
	})


## 合并手动超控和自动补给超控的状态，仅在变化时发送。
func _sync_override_mode(force: bool = false) -> void:
	var enabled := manual_override_enabled or auto_resupply_override_enabled or auto_gold_override_enabled
	if not force and override_enabled == enabled:
		return

	override_enabled = enabled
	_send_override_mode(enabled)


## 切换手动超控状态（由 UI 复选框驱动）。
func _set_manual_override_enabled(enabled: bool) -> void:
	if manual_override_enabled == enabled:
		return

	manual_override_enabled = enabled
	_sync_override_mode()


## 切换自动补给超控状态。
## 关闭时清空累计缓冲区。
func _set_auto_resupply_override_enabled(enabled: bool) -> void:
	if auto_resupply_override_enabled == enabled:
		return

	auto_resupply_override_enabled = enabled
	if not enabled:
		auto_resupply_health_buffer = 0.0
	_sync_override_mode()


## 切换金币同步使用的临时超控状态。
func _set_auto_gold_override_enabled(enabled: bool) -> void:
	if auto_gold_override_enabled == enabled:
		return

	auto_gold_override_enabled = enabled
	_sync_override_mode()


## 发送超控补丁：将 Godot 端对 blackboard 的修改推送到 Lua 端。
## patch 格式：{"section": {"key": value}, ...}
func _send_override_patch(patch: Dictionary) -> void:
	if not connected or not override_enabled:
		return

	override_rev += 1
	_send_json({
		"type": "sim.override_patch",
		"rev": override_rev,
		"patch": patch,
	})


## 将补丁合并到本地 blackboard 缓存（乐观更新）。
## 支持多层嵌套 (section.key = value)。
func _apply_local_patch(patch: Dictionary) -> void:
	for key in patch.keys():
		var value = patch[key]
		if typeof(value) == TYPE_DICTIONARY:
			var target = blackboard.get(key, {})
			if typeof(target) != TYPE_DICTIONARY:
				target = {}
			for child_key in value.keys():
				target[child_key] = value[child_key]
			blackboard[key] = target
		else:
			blackboard[key] = value


## 安全获取 blackboard 中的子字典。
func _get_dict_value(source: Dictionary, key: String) -> Dictionary:
	var value = source.get(key, {})
	if typeof(value) == TYPE_DICTIONARY:
		return value
	return {}


## 安全获取字典中的数组值。
func _get_array_value(source: Dictionary, key: String) -> Array:
	var value = source.get(key, [])
	if typeof(value) == TYPE_ARRAY:
		return value
	return []


## 安全将 Variant 转换为数值类型。
func _get_numeric_value(value: Variant, fallback: float = 0.0) -> float:
	if typeof(value) == TYPE_INT or typeof(value) == TYPE_FLOAT:
		return float(value)
	return fallback


## 安全将 Variant 转换为布尔值。
func _get_boolean_value(value: Variant, fallback: bool = false) -> bool:
	if typeof(value) == TYPE_BOOL:
		return bool(value)
	if typeof(value) == TYPE_INT or typeof(value) == TYPE_FLOAT:
		return float(value) != 0.0
	return fallback


## 获取 blackboard 中指定分节的非负整数值。
func _get_positive_target_value(section: String, key: String) -> int:
	var data := _get_dict_value(blackboard, section)
	var target := int(round(_get_numeric_value(data.get(key, 0), 0.0)))
	if target < 0:
		return 0
	return target


## 从 blackboard rule 分节解析己方补给点坐标。
## @return: Vector2 补给点坐标，无效时返回 null。
func _get_resupply_point() -> Variant:
	var rule := _get_dict_value(blackboard, "rule")
	var resupply_zone := _get_dict_value(rule, "resupply_zone")
	var ours := _get_dict_value(resupply_zone, "ours")
	if not ours.has("x") or not ours.has("y"):
		return null

	var x := _get_numeric_value(ours.get("x", null), NAN)
	var y := _get_numeric_value(ours.get("y", null), NAN)
	if is_nan(x) or is_nan(y):
		return null

	return Vector2(x, y)


## 返回当前启用的矩形补给区；无效时返回 null。
func _get_resupply_rect() -> Variant:
	if not use_resupply_rect:
		return null

	var min_x := minf(resupply_rect_min.x, resupply_rect_max.x)
	var min_y := minf(resupply_rect_min.y, resupply_rect_max.y)
	var max_x := maxf(resupply_rect_min.x, resupply_rect_max.x)
	var max_y := maxf(resupply_rect_min.y, resupply_rect_max.y)
	if max_x <= min_x or max_y <= min_y:
		return null

	return {
		"min": Vector2(min_x, min_y),
		"max": Vector2(max_x, max_y),
	}


## 检查点是否落在矩形补给区内，边界视为在区内。
func _is_point_in_resupply_rect(point: Vector2, rect: Dictionary) -> bool:
	var rect_min: Vector2 = rect["min"]
	var rect_max: Vector2 = rect["max"]
	return point.x >= rect_min.x and point.x <= rect_max.x and point.y >= rect_min.y and point.y <= rect_max.y


## 计算点到矩形补给区的最短距离；区内返回 0。
func _distance_to_resupply_rect(point: Vector2, rect: Dictionary) -> float:
	var rect_min: Vector2 = rect["min"]
	var rect_max: Vector2 = rect["max"]
	var clamped_x := clampf(point.x, rect_min.x, rect_max.x)
	var clamped_y := clampf(point.y, rect_min.y, rect_max.y)
	return point.distance_to(Vector2(clamped_x, clamped_y))


## 按速率线性推进资源值（血量），通过 buffer 累积分整数值变化。
## @param current: 当前资源值。
## @param target: 目标值。
## @param rate: 回复速率 (每秒)。
## @param delta: 帧时间。
## @param buffer: 累计浮点缓冲区。
## @return: {next_value, buffer, changed}。
func _advance_resupply_resource(
	current: int, target: int, rate: float, delta: float, buffer: float
) -> Dictionary:
	var result := {
		"next_value": current,
		"buffer": 0.0,
		"changed": false,
	}
	if target <= 0 or current >= target:
		return result

	# 无限大速率（类 rate <= 0）：直接跳到目标值。
	if rate <= 0.0:
		result.next_value = target
		result.changed = current != target
		return result

	buffer += rate * delta
	var gain := int(floor(buffer))
	if gain <= 0:
		result.buffer = buffer
		return result

	buffer -= float(gain)
	var next_value: int = current + gain
	if next_value > target:
		next_value = target
	if next_value >= target:
		buffer = 0.0

	result.next_value = next_value
	result.buffer = buffer
	result.changed = next_value != current
	return result


## 自动补给主逻辑：检测是否进入补给区，按速率回复血量。
## 当机器人进入补给区时激活超控，退出时取消超控。
func _update_auto_resupply(delta: float) -> void:
	var previous_in_zone := in_resupply_zone
	var previous_active := auto_resupply_active

	in_resupply_zone = false
	auto_resupply_active = false
	resupply_distance = null

	# blackboard 未就绪时退出。
	if not blackboard_ready:
		_set_auto_resupply_override_enabled(false)
		if previous_in_zone or previous_active:
			_refresh_display()
		return

	var robot_position := Vector2(robot.global_position.x, robot.global_position.z)
	var resupply_rect_value: Variant = _get_resupply_rect()
	if resupply_rect_value != null:
		var resupply_rect: Dictionary = resupply_rect_value
		resupply_distance = _distance_to_resupply_rect(robot_position, resupply_rect)
		in_resupply_zone = _is_point_in_resupply_rect(robot_position, resupply_rect)
	else:
		# 矩形补给区关闭或无效时，回退到旧补给点半径判定。
		var resupply_point_value: Variant = _get_resupply_point()
		if resupply_point_value == null:
			_set_auto_resupply_override_enabled(false)
			if previous_in_zone or previous_active:
				_refresh_display()
			return

		var resupply_point: Vector2 = resupply_point_value
		var distance := robot_position.distance_to(resupply_point)
		resupply_distance = distance
		in_resupply_zone = distance <= resupply_radius

	# 读取期望回复的目标值。
	var target_health := _get_positive_target_value("rule", "health_ready")
	auto_resupply_active = in_resupply_zone and target_health > 0
	_set_auto_resupply_override_enabled(auto_resupply_active)

	if not auto_resupply_active:
		if previous_in_zone != in_resupply_zone or previous_active:
			_refresh_display()
		return

	var user := _get_dict_value(blackboard, "user")
	var patch_user := {}

	# 按速率回复血量。
	if target_health > 0:
		var current_health := int(round(_get_numeric_value(user.get("health", 0), 0.0)))
		var health_result := _advance_resupply_resource(
			current_health,
			target_health,
			resupply_health_rate,
			delta,
			auto_resupply_health_buffer
		)
		auto_resupply_health_buffer = float(health_result.get("buffer", 0.0))
		if bool(health_result.get("changed", false)):
			patch_user["health"] = int(health_result.get("next_value", current_health))
	else:
		auto_resupply_health_buffer = 0.0

	if patch_user.is_empty():
		if previous_in_zone != in_resupply_zone or previous_active != auto_resupply_active:
			_refresh_display()
		return

	# 有变化的资源：同时更新本地缓存、机器人状态、并推送超控补丁。
	var patch := {"user": patch_user}
	_apply_local_patch(patch)
	var applied_user_patch: Dictionary = patch_user
	_apply_robot_user_patch(applied_user_patch)
	_sync_controls_from_blackboard()
	_send_override_patch(patch)
	_refresh_display()


## 发送 JSON 行到 TCP 对端 (\n 分隔)。
func _send_json(data: Dictionary) -> void:
	if tcp.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return

	var line := JSON.stringify(data) + "\n"
	var err := tcp.put_data(line.to_utf8_buffer())
	if err != OK:
		push_warning("tcp put_data failed: %s" % err)


## 构建调试 UI 面板：CanvasLayer + Panel + 控件布局。
## 包含：标题、开始决策按钮、超控复选框、编辑面板 (HP/Bullet/Stage/Switch)、
## Decision State 和 Blackboard 显示。
func _build_debug_ui() -> void:
	lua_sim_canvas = CanvasLayer.new()
	var canvas := lua_sim_canvas
	canvas.name = "LuaSimCanvas"
	add_child(canvas)

	lua_sim_panel = Panel.new()
	var panel := lua_sim_panel
	panel.name = "LuaSimPanel"
	panel.anchor_left = 0.5
	panel.anchor_right = 0.5
	panel.anchor_top = 0.5
	panel.anchor_bottom = 0.5
	panel.offset_left = -DEBUG_PANEL_SIZE.x * 0.5
	panel.offset_right = DEBUG_PANEL_SIZE.x * 0.5
	panel.offset_top = -DEBUG_PANEL_SIZE.y * 0.5
	panel.offset_bottom = DEBUG_PANEL_SIZE.y * 0.5
	panel.visible = false
	canvas.add_child(panel)

	var root := VBoxContainer.new()
	root.name = "Root"
	root.set_anchors_preset(Control.PRESET_FULL_RECT)
	root.offset_left = 10
	root.offset_top = 10
	root.offset_right = -10
	root.offset_bottom = -10
	root.add_theme_constant_override("separation", 8)
	panel.add_child(root)

	var title := Label.new()
	title.text = "Lua Sim v2"
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	root.add_child(title)

	lua_sim_tabs = TabContainer.new()
	lua_sim_tabs.name = "LuaSimTabs"
	lua_sim_tabs.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	lua_sim_tabs.size_flags_vertical = Control.SIZE_EXPAND_FILL
	root.add_child(lua_sim_tabs)

	var loot_tab := VBoxContainer.new()
	loot_tab.name = "Loot"
	loot_tab.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	loot_tab.size_flags_vertical = Control.SIZE_EXPAND_FILL
	loot_tab.add_theme_constant_override("separation", 8)
	lua_sim_tabs.add_child(loot_tab)

	var edit_tab := VBoxContainer.new()
	edit_tab.name = "Edit"
	edit_tab.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	edit_tab.size_flags_vertical = Control.SIZE_EXPAND_FILL
	edit_tab.add_theme_constant_override("separation", 8)
	lua_sim_tabs.add_child(edit_tab)

	var edit_header := VBoxContainer.new()
	edit_header.name = "EditHeader"
	edit_header.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	edit_header.add_theme_constant_override("separation", 6)
	edit_tab.add_child(edit_header)

	var edit_scroll := ScrollContainer.new()
	edit_scroll.name = "EditScroll"
	edit_scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	edit_scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	edit_tab.add_child(edit_scroll)

	var edit_root := VBoxContainer.new()
	edit_root.name = "EditRoot"
	edit_root.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	edit_root.add_theme_constant_override("separation", 6)
	edit_scroll.add_child(edit_root)

	var blackboard_tab := VBoxContainer.new()
	blackboard_tab.name = "Blackboard"
	blackboard_tab.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	blackboard_tab.size_flags_vertical = Control.SIZE_EXPAND_FILL
	blackboard_tab.add_theme_constant_override("separation", 8)
	lua_sim_tabs.add_child(blackboard_tab)

	override_toggle = CheckButton.new()
	override_toggle.text = "Open Edit Panel (Godot Override)"
	override_toggle.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	override_toggle.toggled.connect(_on_override_toggled)
	edit_header.add_child(override_toggle)
	override_toggle.button_pressed = true

	var gold_badge := PanelContainer.new()
	gold_badge.name = "GoldBadge"
	gold_badge.anchor_left = 1.0
	gold_badge.anchor_right = 1.0
	gold_badge.anchor_top = 0.0
	gold_badge.anchor_bottom = 0.0
	gold_badge.offset_left = -220
	gold_badge.offset_top = 16
	gold_badge.offset_right = -16
	gold_badge.offset_bottom = 78
	var gold_badge_style := StyleBoxFlat.new()
	gold_badge_style.bg_color = Color(0.14, 0.11, 0.03, 0.94)
	gold_badge_style.border_width_left = 2
	gold_badge_style.border_width_top = 2
	gold_badge_style.border_width_right = 2
	gold_badge_style.border_width_bottom = 2
	gold_badge_style.border_color = Color(1.0, 0.82, 0.16, 1.0)
	gold_badge_style.corner_radius_top_left = 16
	gold_badge_style.corner_radius_top_right = 16
	gold_badge_style.corner_radius_bottom_right = 16
	gold_badge_style.corner_radius_bottom_left = 16
	gold_badge.add_theme_stylebox_override("panel", gold_badge_style)
	canvas.add_child(gold_badge)
	gold_badge_panel = gold_badge

	var gold_badge_margin := MarginContainer.new()
	gold_badge_margin.add_theme_constant_override("margin_left", 14)
	gold_badge_margin.add_theme_constant_override("margin_top", 10)
	gold_badge_margin.add_theme_constant_override("margin_right", 14)
	gold_badge_margin.add_theme_constant_override("margin_bottom", 10)
	gold_badge.add_child(gold_badge_margin)

	var gold_badge_row := HBoxContainer.new()
	gold_badge_row.add_theme_constant_override("separation", 12)
	gold_badge_margin.add_child(gold_badge_row)

	var coin_mark := Panel.new()
	coin_mark.custom_minimum_size = Vector2(28, 28)
	var coin_mark_style := StyleBoxFlat.new()
	coin_mark_style.bg_color = Color(1.0, 0.83, 0.18, 1.0)
	coin_mark_style.border_width_left = 2
	coin_mark_style.border_width_top = 2
	coin_mark_style.border_width_right = 2
	coin_mark_style.border_width_bottom = 2
	coin_mark_style.border_color = Color(1.0, 0.94, 0.56, 1.0)
	coin_mark_style.corner_radius_top_left = 14
	coin_mark_style.corner_radius_top_right = 14
	coin_mark_style.corner_radius_bottom_right = 14
	coin_mark_style.corner_radius_bottom_left = 14
	coin_mark.add_theme_stylebox_override("panel", coin_mark_style)
	gold_badge_row.add_child(coin_mark)

	var gold_badge_text := VBoxContainer.new()
	gold_badge_text.add_theme_constant_override("separation", 0)
	gold_badge_row.add_child(gold_badge_text)

	var gold_title := Label.new()
	gold_title.text = "GOLD"
	gold_title.add_theme_font_size_override("font_size", 12)
	gold_title.add_theme_color_override("font_color", Color(1.0, 0.86, 0.34, 0.95))
	gold_badge_text.add_child(gold_title)

	gold_badge_value_label = Label.new()
	gold_badge_value_label.text = "0"
	gold_badge_value_label.add_theme_font_size_override("font_size", 24)
	gold_badge_value_label.add_theme_color_override("font_color", Color(1.0, 0.95, 0.74, 1.0))
	gold_badge_text.add_child(gold_badge_value_label)

	var base_badge := PanelContainer.new()
	base_badge.name = "BaseBadge"
	base_badge.anchor_left = 1.0
	base_badge.anchor_right = 1.0
	base_badge.anchor_top = 0.0
	base_badge.anchor_bottom = 0.0
	base_badge.offset_left = -424
	base_badge.offset_top = 16
	base_badge.offset_right = -240
	base_badge.offset_bottom = 78
	var base_badge_style := StyleBoxFlat.new()
	base_badge_style.bg_color = Color(0.12, 0.08, 0.08, 0.94)
	base_badge_style.border_width_left = 2
	base_badge_style.border_width_top = 2
	base_badge_style.border_width_right = 2
	base_badge_style.border_width_bottom = 2
	base_badge_style.border_color = Color(0.97, 0.42, 0.32, 1.0)
	base_badge_style.corner_radius_top_left = 16
	base_badge_style.corner_radius_top_right = 16
	base_badge_style.corner_radius_bottom_right = 16
	base_badge_style.corner_radius_bottom_left = 16
	base_badge.add_theme_stylebox_override("panel", base_badge_style)
	canvas.add_child(base_badge)
	base_badge_panel = base_badge

	var base_badge_margin := MarginContainer.new()
	base_badge_margin.add_theme_constant_override("margin_left", 14)
	base_badge_margin.add_theme_constant_override("margin_top", 10)
	base_badge_margin.add_theme_constant_override("margin_right", 14)
	base_badge_margin.add_theme_constant_override("margin_bottom", 10)
	base_badge.add_child(base_badge_margin)

	var base_badge_row := HBoxContainer.new()
	base_badge_row.add_theme_constant_override("separation", 12)
	base_badge_margin.add_child(base_badge_row)

	var base_mark := Panel.new()
	base_mark.custom_minimum_size = Vector2(28, 28)
	var base_mark_style := StyleBoxFlat.new()
	base_mark_style.bg_color = Color(0.93, 0.32, 0.28, 1.0)
	base_mark_style.border_width_left = 2
	base_mark_style.border_width_top = 2
	base_mark_style.border_width_right = 2
	base_mark_style.border_width_bottom = 2
	base_mark_style.border_color = Color(1.0, 0.67, 0.60, 1.0)
	base_mark_style.corner_radius_top_left = 14
	base_mark_style.corner_radius_top_right = 14
	base_mark_style.corner_radius_bottom_right = 14
	base_mark_style.corner_radius_bottom_left = 14
	base_mark.add_theme_stylebox_override("panel", base_mark_style)
	base_badge_row.add_child(base_mark)

	var base_badge_text := VBoxContainer.new()
	base_badge_text.add_theme_constant_override("separation", 0)
	base_badge_row.add_child(base_badge_text)

	var base_title := Label.new()
	base_title.text = "BASE"
	base_title.add_theme_font_size_override("font_size", 12)
	base_title.add_theme_color_override("font_color", Color(1.0, 0.69, 0.62, 0.95))
	base_badge_text.add_child(base_title)

	base_badge_value_label = Label.new()
	base_badge_value_label.text = str(initial_base_health)
	base_badge_value_label.add_theme_font_size_override("font_size", 24)
	base_badge_value_label.add_theme_color_override("font_color", Color(1.0, 0.90, 0.88, 1.0))
	base_badge_text.add_child(base_badge_value_label)

	var outpost_badge := PanelContainer.new()
	outpost_badge.name = "OutpostBadge"
	outpost_badge.anchor_left = 1.0
	outpost_badge.anchor_right = 1.0
	outpost_badge.anchor_top = 0.0
	outpost_badge.anchor_bottom = 0.0
	outpost_badge.offset_left = -628
	outpost_badge.offset_top = 16
	outpost_badge.offset_right = -444
	outpost_badge.offset_bottom = 78
	var outpost_badge_style := StyleBoxFlat.new()
	outpost_badge_style.bg_color = Color(0.08, 0.11, 0.14, 0.94)
	outpost_badge_style.border_width_left = 2
	outpost_badge_style.border_width_top = 2
	outpost_badge_style.border_width_right = 2
	outpost_badge_style.border_width_bottom = 2
	outpost_badge_style.border_color = Color(0.34, 0.74, 1.0, 1.0)
	outpost_badge_style.corner_radius_top_left = 16
	outpost_badge_style.corner_radius_top_right = 16
	outpost_badge_style.corner_radius_bottom_right = 16
	outpost_badge_style.corner_radius_bottom_left = 16
	outpost_badge.add_theme_stylebox_override("panel", outpost_badge_style)
	canvas.add_child(outpost_badge)
	outpost_badge_panel = outpost_badge

	var outpost_badge_margin := MarginContainer.new()
	outpost_badge_margin.add_theme_constant_override("margin_left", 14)
	outpost_badge_margin.add_theme_constant_override("margin_top", 10)
	outpost_badge_margin.add_theme_constant_override("margin_right", 14)
	outpost_badge_margin.add_theme_constant_override("margin_bottom", 10)
	outpost_badge.add_child(outpost_badge_margin)

	var outpost_badge_row := HBoxContainer.new()
	outpost_badge_row.add_theme_constant_override("separation", 12)
	outpost_badge_margin.add_child(outpost_badge_row)

	var outpost_mark := Panel.new()
	outpost_mark.custom_minimum_size = Vector2(28, 28)
	var outpost_mark_style := StyleBoxFlat.new()
	outpost_mark_style.bg_color = Color(0.22, 0.67, 0.94, 1.0)
	outpost_mark_style.border_width_left = 2
	outpost_mark_style.border_width_top = 2
	outpost_mark_style.border_width_right = 2
	outpost_mark_style.border_width_bottom = 2
	outpost_mark_style.border_color = Color(0.70, 0.90, 1.0, 1.0)
	outpost_mark_style.corner_radius_top_left = 14
	outpost_mark_style.corner_radius_top_right = 14
	outpost_mark_style.corner_radius_bottom_right = 14
	outpost_mark_style.corner_radius_bottom_left = 14
	outpost_mark.add_theme_stylebox_override("panel", outpost_mark_style)
	outpost_badge_row.add_child(outpost_mark)

	var outpost_badge_text := VBoxContainer.new()
	outpost_badge_text.add_theme_constant_override("separation", 0)
	outpost_badge_row.add_child(outpost_badge_text)

	var outpost_title := Label.new()
	outpost_title.text = "OUTPOST"
	outpost_title.add_theme_font_size_override("font_size", 12)
	outpost_title.add_theme_color_override("font_color", Color(0.72, 0.92, 1.0, 0.95))
	outpost_badge_text.add_child(outpost_title)

	outpost_badge_value_label = Label.new()
	outpost_badge_value_label.text = str(initial_outpost_health)
	outpost_badge_value_label.add_theme_font_size_override("font_size", 24)
	outpost_badge_value_label.add_theme_color_override("font_color", Color(0.90, 0.98, 1.0, 1.0))
	outpost_badge_text.add_child(outpost_badge_value_label)

	edit_box = VBoxContainer.new()
	edit_box.visible = true
	edit_box.add_theme_constant_override("separation", 6)
	edit_root.add_child(edit_box)

	hp_spin = _add_spin_row(edit_box, "HP", 0.0, 800.0, 1.0)
	hp_spin.value_changed.connect(_on_hp_changed)

	bullet_spin = _add_spin_row(edit_box, "Bullet", 0.0, 500.0, 1.0)
	bullet_spin.value_changed.connect(_on_bullet_changed)

	gold_spin = _add_spin_row(edit_box, "Gold", 0.0, 99999.0, 1.0)
	gold_spin.value_changed.connect(_on_gold_changed)

	base_hp_spin = _add_spin_row(edit_box, "Base HP", 0.0, 99999.0, 1.0)
	base_hp_spin.value_changed.connect(_on_base_hp_changed)

	outpost_hp_spin = _add_spin_row(edit_box, "Outpost HP", 0.0, 99999.0, 1.0)
	outpost_hp_spin.value_changed.connect(_on_outpost_hp_changed)

	remaining_time_spin = _add_spin_row(edit_box, "Remain Time", 0.0, 420.0, 1.0)
	remaining_time_spin.value_changed.connect(_on_remaining_time_changed)

	exchangeable_ammo_spin = _add_spin_row(edit_box, "Ammo Bank", 0.0, 99999.0, 1.0)
	exchangeable_ammo_spin.value_changed.connect(_on_exchangeable_ammo_changed)

	dart_hits_spin = _add_spin_row(edit_box, "Dart Hits", 0.0, 999.0, 1.0)
	dart_hits_spin.value_changed.connect(_on_dart_hits_changed)

	fortress_occupied_toggle = _add_check_row(edit_box, "Fortress")
	fortress_occupied_toggle.toggled.connect(_on_fortress_occupied_toggled)

	big_energy_mechanism_toggle = _add_check_row(edit_box, "Big Energy")
	big_energy_mechanism_toggle.toggled.connect(_on_big_energy_mechanism_toggled)

	small_energy_mechanism_toggle = _add_check_row(edit_box, "Small Energy")
	small_energy_mechanism_toggle.toggled.connect(_on_small_energy_mechanism_toggled)

	stage_select = _add_option_row(edit_box, "Stage", STAGE_VALUES)
	stage_select.item_selected.connect(_on_stage_selected)

	rswitch_select = _add_option_row(edit_box, "R Switch", SWITCH_VALUES)
	rswitch_select.item_selected.connect(_on_rswitch_selected)

	lswitch_select = _add_option_row(edit_box, "L Switch", SWITCH_VALUES)
	lswitch_select.item_selected.connect(_on_lswitch_selected)

	var decision_title := Label.new()
	decision_title.text = "Loot Runtime Graph"
	loot_tab.add_child(decision_title)

	var loot_graph_scroll := ScrollContainer.new()
	loot_graph_scroll.name = "LootGraphScroll"
	loot_graph_scroll.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	loot_graph_scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	loot_tab.add_child(loot_graph_scroll)

	loot_flow_view = LOOT_FLOW_VIEW_SCRIPT.new()
	loot_flow_view.name = "LootFlowView"
	loot_flow_view.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	loot_flow_view.size_flags_vertical = Control.SIZE_FILL
	loot_graph_scroll.add_child(loot_flow_view)

	var bb_title := Label.new()
	bb_title.text = "Blackboard"
	blackboard_tab.add_child(bb_title)

	blackboard_label = RichTextLabel.new()
	blackboard_label.fit_content = false
	blackboard_label.custom_minimum_size = Vector2(0, 360)
	blackboard_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	blackboard_label.size_flags_vertical = Control.SIZE_EXPAND_FILL
	blackboard_label.scroll_active = true
	blackboard_label.selection_enabled = true
	blackboard_tab.add_child(blackboard_label)

	lua_sim_tabs.current_tab = 0
	_set_manual_override_enabled(true)
	_set_overlay_badges_visible(not lua_sim_panel_open)


func _set_overlay_badges_visible(visible: bool) -> void:
	if gold_badge_panel != null:
		gold_badge_panel.visible = visible
	if base_badge_panel != null:
		base_badge_panel.visible = visible
	if outpost_badge_panel != null:
		outpost_badge_panel.visible = visible


## 创建一行 SpinBox 控件（标签 + 数值输入），返回 SpinBox 引用。
func _add_spin_row(parent: VBoxContainer, label_text: String, min_v: float, max_v: float, step_v: float) -> SpinBox:
	var row := HBoxContainer.new()
	parent.add_child(row)

	var label := Label.new()
	label.text = label_text
	label.custom_minimum_size = Vector2(90, 0)
	row.add_child(label)

	var spin := SpinBox.new()
	spin.min_value = min_v
	spin.max_value = max_v
	spin.step = step_v
	spin.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(spin)
	return spin


## 创建一行 OptionButton 控件（标签 + 下拉选择），返回 OptionButton 引用。
func _add_option_row(parent: VBoxContainer, label_text: String, values: Array) -> OptionButton:
	var row := HBoxContainer.new()
	parent.add_child(row)

	var label := Label.new()
	label.text = label_text
	label.custom_minimum_size = Vector2(90, 0)
	row.add_child(label)

	var option := OptionButton.new()
	option.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	for entry in values:
		option.add_item(str(entry))
	row.add_child(option)
	return option


## 创建一行布尔开关控件（标签 + 勾选框），返回 CheckButton 引用。
func _add_check_row(parent: VBoxContainer, label_text: String) -> CheckButton:
	var row := HBoxContainer.new()
	parent.add_child(row)

	var label := Label.new()
	label.text = label_text
	label.custom_minimum_size = Vector2(90, 0)
	row.add_child(label)

	var toggle := CheckButton.new()
	toggle.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(toggle)
	return toggle


## UI 回调：超控复选框切换，显示/隐藏编辑面板并更新超控状态。
func _on_override_toggled(enabled: bool) -> void:
	if edit_box != null:
		edit_box.visible = enabled
	_set_manual_override_enabled(enabled)
	_refresh_display()


## UI 回调：HP 数值改变 → 推送超控补丁。
func _on_hp_changed(value: float) -> void:
	if ui_updating:
		return

	var patch := {"user": {"health": int(round(value))}}
	_apply_local_patch(patch)
	var user_patch: Dictionary = patch["user"]
	_apply_robot_user_patch(user_patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：子弹数值改变 → 推送超控补丁。
func _on_bullet_changed(value: float) -> void:
	if ui_updating:
		return

	var patch := {"user": {"bullet": int(round(value))}}
	_apply_local_patch(patch)
	var user_patch: Dictionary = patch["user"]
	_apply_robot_user_patch(user_patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：金币数值改变 → 推送超控补丁。
func _on_gold_changed(value: float) -> void:
	if ui_updating:
		return

	var patch: Dictionary = {"user": {"gold": int(round(value))}}
	_apply_local_patch(patch)
	_apply_local_patch({"game": {"gold_coin": int(round(value))}})
	var user_patch: Dictionary = patch["user"]
	_apply_robot_user_patch(user_patch)
	_refresh_display()


## UI 回调：基地血量改变 → 更新本地比赛状态，后续通过 sim.input 同步。
func _on_base_hp_changed(value: float) -> void:
	if ui_updating:
		return

	sim_base_health = maxi(0, int(round(value)))
	_apply_local_patch({"game": {"base_health": sim_base_health}})
	_refresh_display()


## UI 回调：前哨站血量改变 → 更新本地比赛状态，后续通过 sim.input 同步。
func _on_outpost_hp_changed(value: float) -> void:
	if ui_updating:
		return

	sim_outpost_health = maxi(0, int(round(value)))
	_apply_local_patch({"game": {"outpost_health": sim_outpost_health}})
	_refresh_display()


## UI 回调：比赛剩余时间改变 → 更新本地比赛状态，后续通过 sim.input 同步。
func _on_remaining_time_changed(value: float) -> void:
	if ui_updating:
		return

	sim_remaining_time = clampf(value, 0.0, initial_remaining_time)
	_apply_local_patch({"game": {"remaining_time": sim_remaining_time}})
	_refresh_display()


## UI 回调：可兑换弹药量改变 → 更新本地比赛状态，后续通过 sim.input 同步。
func _on_exchangeable_ammo_changed(value: float) -> void:
	if ui_updating:
		return

	sim_exchangeable_ammunition_quantity = maxi(0, int(round(value)))
	_apply_local_patch({
		"game": {
			"exchangeable_ammunition_quantity": sim_exchangeable_ammunition_quantity
		}
	})
	_refresh_display()


## UI 回调：己方飞镖命中次数改变。
func _on_dart_hits_changed(value: float) -> void:
	if ui_updating:
		return

	sim_our_dart_nmber_of_hits = maxi(0, int(round(value)))
	var patch := {
		"game": {
			"our_dart_nmber_of_hits": sim_our_dart_nmber_of_hits,
		}
	}
	_apply_local_patch(patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：堡垒占领状态改变。
func _on_fortress_occupied_toggled(enabled: bool) -> void:
	if ui_updating:
		return

	sim_fortress_occupied = enabled
	var patch := {
		"game": {
			"fortress_occupied": sim_fortress_occupied,
		}
	}
	_apply_local_patch(patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：大能量机关激活状态改变。
func _on_big_energy_mechanism_toggled(enabled: bool) -> void:
	if ui_updating:
		return

	sim_big_energy_mechanism_activated = enabled
	var patch := {
		"game": {
			"big_energy_mechanism_activated": sim_big_energy_mechanism_activated,
		}
	}
	_apply_local_patch(patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：小能量机关激活状态改变。
func _on_small_energy_mechanism_toggled(enabled: bool) -> void:
	if ui_updating:
		return

	sim_small_energy_mechanism_activated = enabled
	var patch := {
		"game": {
			"small_energy_mechanism_activated": sim_small_energy_mechanism_activated,
		}
	}
	_apply_local_patch(patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：Stage 选择改变 → 推送 game.stage。
func _on_stage_selected(index: int) -> void:
	if ui_updating:
		return

	var stage : String = STAGE_VALUES[clamp(index, 0, STAGE_VALUES.size() - 1)]
	if stage == "STARTED":
		_begin_competition_session()
	else:
		competition_started = false
	var patch := {"game": {"stage": stage, "remaining_time": sim_remaining_time}}
	_apply_local_patch(patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：R Switch 选择改变 → 推送 play.rswitch。
func _on_rswitch_selected(index: int) -> void:
	if ui_updating:
		return

	var value : String = SWITCH_VALUES[clamp(index, 0, SWITCH_VALUES.size() - 1)]
	var patch := {"play": {"rswitch": value}}
	_apply_local_patch(patch)
	_send_override_patch(patch)
	_refresh_display()


## UI 回调：L Switch 选择改变 → 推送 play.lswitch。
func _on_lswitch_selected(index: int) -> void:
	if ui_updating:
		return

	var value : String = SWITCH_VALUES[clamp(index, 0, SWITCH_VALUES.size() - 1)]
	var patch := {"play": {"lswitch": value}}
	_apply_local_patch(patch)
	_send_override_patch(patch)
	_refresh_display()


## 从 blackboard 同步 UI 控件显示值（仅更新显示，不触发回调）。
func _sync_controls_from_blackboard() -> void:
	if hp_spin == null:
		return

	ui_updating = true
	var user: Dictionary = blackboard.get("user", {})
	var game: Dictionary = blackboard.get("game", {})
	var play: Dictionary = blackboard.get("play", {})

	hp_spin.value = float(user.get("health", hp_spin.value))
	bullet_spin.value = float(user.get("bullet", bullet_spin.value))
	gold_spin.value = float(user.get("gold", gold_spin.value))
	sim_base_health = int(round(_get_numeric_value(game.get("base_health", sim_base_health), 0.0)))
	sim_outpost_health = int(round(_get_numeric_value(game.get("outpost_health", sim_outpost_health), 0.0)))
	sim_remaining_time = _get_numeric_value(game.get("remaining_time", sim_remaining_time), 0.0)
	sim_exchangeable_ammunition_quantity = int(round(
		_get_numeric_value(
			game.get("exchangeable_ammunition_quantity", sim_exchangeable_ammunition_quantity),
			0.0
		)
	))
	sim_our_dart_nmber_of_hits = int(round(
		_get_numeric_value(game.get("our_dart_nmber_of_hits", sim_our_dart_nmber_of_hits), 0.0)
	))
	sim_fortress_occupied = _get_boolean_value(
		game.get("fortress_occupied", sim_fortress_occupied),
		sim_fortress_occupied
	)
	sim_big_energy_mechanism_activated = _get_boolean_value(
		game.get("big_energy_mechanism_activated", sim_big_energy_mechanism_activated),
		sim_big_energy_mechanism_activated
	)
	sim_small_energy_mechanism_activated = _get_boolean_value(
		game.get("small_energy_mechanism_activated", sim_small_energy_mechanism_activated),
		sim_small_energy_mechanism_activated
	)
	if base_hp_spin != null:
		base_hp_spin.value = float(sim_base_health)
	if outpost_hp_spin != null:
		outpost_hp_spin.value = float(sim_outpost_health)
	if remaining_time_spin != null:
		remaining_time_spin.value = sim_remaining_time
	if exchangeable_ammo_spin != null:
		exchangeable_ammo_spin.value = float(sim_exchangeable_ammunition_quantity)
	if dart_hits_spin != null:
		dart_hits_spin.value = float(sim_our_dart_nmber_of_hits)
	if fortress_occupied_toggle != null:
		fortress_occupied_toggle.set_pressed_no_signal(sim_fortress_occupied)
	if big_energy_mechanism_toggle != null:
		big_energy_mechanism_toggle.set_pressed_no_signal(sim_big_energy_mechanism_activated)
	if small_energy_mechanism_toggle != null:
		small_energy_mechanism_toggle.set_pressed_no_signal(
			sim_small_energy_mechanism_activated
		)

	_select_option_value(stage_select, STAGE_VALUES, str(game.get("stage", "UNKNOWN")))
	_select_option_value(rswitch_select, SWITCH_VALUES, str(play.get("rswitch", "UNKNOWN")))
	_select_option_value(lswitch_select, SWITCH_VALUES, str(play.get("lswitch", "UNKNOWN")))
	competition_started = str(game.get("stage", "UNKNOWN")) == "STARTED"
	ui_updating = false


func _begin_competition_session() -> void:
	sim_remaining_time = initial_remaining_time
	competition_started = true
	_apply_local_patch({
		"game": {
			"stage": "STARTED",
			"remaining_time": sim_remaining_time,
		}
	})


## 在 OptionButton 中选中匹配的枚举值。
func _select_option_value(button: OptionButton, values: Array, value: String) -> void:
	var target := values.find(value)
	if target < 0:
		target = 0
	button.select(target)


## 刷新调试面板显示（决策状态 + blackboard）。
func _refresh_display() -> void:
	if blackboard_label != null:
		blackboard_label.text = _format_blackboard_text()
	_refresh_gold_badge()
	_refresh_competition_badges()


func _refresh_gold_badge() -> void:
	if gold_badge_value_label == null:
		return

	gold_badge_value_label.text = str(_get_current_gold_value())


func _get_current_gold_value() -> int:
	var user: Dictionary = _get_dict_value(blackboard, "user")
	if user.has("gold"):
		return int(round(_get_numeric_value(user.get("gold", 0), 0.0)))

	var resource: Dictionary = _get_robot_resource_state()
	return int(round(_get_numeric_value(resource.get("gold", 0), 0.0)))


func _refresh_competition_badges() -> void:
	if base_badge_value_label != null:
		base_badge_value_label.text = str(_get_current_base_health_value())
	if outpost_badge_value_label != null:
		outpost_badge_value_label.text = str(_get_current_outpost_health_value())


func _get_current_base_health_value() -> int:
	var game: Dictionary = _get_dict_value(blackboard, "game")
	if game.has("base_health"):
		return int(round(_get_numeric_value(game.get("base_health", sim_base_health), 0.0)))
	return sim_base_health


func _get_current_outpost_health_value() -> int:
	var game: Dictionary = _get_dict_value(blackboard, "game")
	if game.has("outpost_health"):
		return int(round(_get_numeric_value(game.get("outpost_health", sim_outpost_health), 0.0)))
	return sim_outpost_health


func get_structure_health(kind: String) -> int:
	if kind == "base":
		return sim_base_health
	if kind == "outpost":
		return sim_outpost_health
	return 0


func sync_structure_health(kind: String, value: int) -> void:
	var next_value := maxi(0, value)
	match kind:
		"base":
			sim_base_health = next_value
			_apply_local_patch({"game": {"base_health": sim_base_health}})
			if base_hp_spin != null:
				base_hp_spin.value = float(sim_base_health)
		"outpost":
			sim_outpost_health = next_value
			_apply_local_patch({"game": {"outpost_health": sim_outpost_health}})
			if outpost_hp_spin != null:
				outpost_hp_spin.value = float(sim_outpost_health)
		_:
			return
	_refresh_display()


func apply_structure_damage(kind: String, damage: int) -> bool:
	if damage <= 0:
		return false

	if kind == "outpost":
		sync_structure_health("outpost", sim_outpost_health - damage)
		return true

	if kind == "base":
		if sim_outpost_health > 0:
			return false
		sync_structure_health("base", sim_base_health - damage)
		return true

	return false


## 格式化决策状态文本（JSON 美观输出）。
func _format_decision_text() -> String:
	var view := {
		"connected": connected,
		"blackboard_ready": blackboard_ready,
		"bb_rev": blackboard_rev,
		"loot_rev": loot_rev,
		"override_enabled": override_enabled,
		"manual_override_enabled": manual_override_enabled,
		"auto_resupply_override_enabled": auto_resupply_override_enabled,
		"override_rev": override_rev,
		"in_resupply_zone": in_resupply_zone,
		"auto_resupply_active": auto_resupply_active,
		"resupply_distance": resupply_distance,
		"sim_time": _round_float(sim_time),
		"target": {
			"x": _round_float(target_point.global_position.x),
			"y": _round_float(target_point.global_position.z),
		},
		"decision": decision_state,
		"loot": _format_loot_summary(),
	}
	return JSON.stringify(_prepare_display_value(view), "  ", true)


## 生成 Loot 语义监控摘要，避免把完整事件流直接塞进主面板。
func _format_loot_summary() -> Dictionary:
	var actions: Dictionary = _get_dict_value(loot_state, "actions")
	var decision_graph: Dictionary = _get_dict_value(loot_state, "decision_graph")
	var fsm_root: Dictionary = _get_dict_value(loot_state, "fsm")
	var tasks_root: Dictionary = _get_dict_value(loot_state, "tasks")

	return {
		"serial": loot_state.get("serial", 0),
		"decision_graph": _summarize_decision_graph(decision_graph),
		"fsm": _summarize_fsm_items(_get_array_value(fsm_root, "items")),
		"active_tasks": _summarize_active_tasks(_get_array_value(tasks_root, "items")),
		"last_action": actions.get("last", null),
		"nav_target": actions.get("nav_target", null),
	}


func _summarize_decision_graph(graph: Dictionary) -> Dictionary:
	var nodes: Array = _get_array_value(graph, "nodes")
	var edges: Array = _get_array_value(graph, "edges")
	return {
		"id": graph.get("id", null),
		"label": graph.get("label", null),
		"node_count": nodes.size(),
		"edge_count": edges.size(),
		"current_state": graph.get("current_state", null),
		"current_intent": graph.get("current_intent", null),
		"current_phase": graph.get("current_phase", null),
		"active_nodes": _get_array_value(graph, "active_nodes"),
		"active_edges": _get_array_value(graph, "active_edges"),
	}


func _summarize_fsm_items(items: Array) -> Array:
	var result: Array = []
	for item in items:
		if typeof(item) != TYPE_DICTIONARY:
			continue
		var fsm: Dictionary = item as Dictionary
		var observed_edges: Array = _get_array_value(fsm, "observed_edges")
		var declared_edges: Array = _get_array_value(fsm, "declared_edges")
		result.append({
			"id": fsm.get("id", null),
			"current_state": fsm.get("current_state", null),
			"last_state": fsm.get("last_state", null),
			"transition_count": fsm.get("transition_count", 0),
			"last_transition": fsm.get("last_transition", null),
			"declared_edge_count": declared_edges.size(),
			"declared_edges": declared_edges,
			"observed_edge_count": observed_edges.size(),
			"observed_edges": observed_edges,
		})
	return result


func _summarize_active_tasks(items: Array) -> Array:
	var result: Array = []
	for item in items:
		if typeof(item) != TYPE_DICTIONARY:
			continue
		var task: Dictionary = item as Dictionary
		var status := str(task.get("status", ""))
		if status == "dead" or status == "cancelled":
			continue
		result.append({
			"id": task.get("id", null),
			"source": task.get("source", null),
			"line": task.get("line", null),
			"status": task.get("status", null),
			"wait_kind": task.get("wait_kind", null),
			"wait_until": task.get("wait_until", null),
			"resume_count": task.get("resume_count", 0),
		})
	return result


## 格式化 blackboard 文本（按分节标注）。
func _format_blackboard_text() -> String:
	var blocks: Array[String] = []
	blocks.append("[user]\n%s" % _stringify_pretty(
		_pick_display_fields(_get_dict_value(blackboard, "user"), BLACKBOARD_USER_DISPLAY_FIELDS)
	))
	blocks.append("[game]\n%s" % _stringify_pretty(
		_pick_display_fields(_get_dict_value(blackboard, "game"), BLACKBOARD_GAME_DISPLAY_FIELDS)
	))
	return "\n\n".join(blocks)


## 从字典中按顺序挑选显示字段，字段不存在时不补空值。
func _pick_display_fields(source: Dictionary, fields: Array) -> Dictionary:
	var result := {}
	for field in fields:
		if source.has(field):
			result[field] = source.get(field)
	return result


## 将 Variant 格式化为美观字符串（字典/数组用 JSON）。
func _stringify_pretty(value: Variant) -> String:
	var prepared = _prepare_display_value(value)
	if typeof(prepared) == TYPE_DICTIONARY or typeof(prepared) == TYPE_ARRAY:
		return JSON.stringify(prepared, "  ", true)
	return str(prepared)


## 递归准备显示值：浮点四舍五入、字典按键排序、限深防展平。
func _prepare_display_value(value: Variant, depth: int = 0) -> Variant:
	if depth >= 6:
		return "..."

	match typeof(value):
		TYPE_FLOAT:
			return _round_float(value)
		TYPE_DICTIONARY:
			var source: Dictionary = value
			var result := {}
			var keys := source.keys()
			keys.sort_custom(func(a, b): return str(a) < str(b))
			for key in keys:
				result[key] = _prepare_display_value(source[key], depth + 1)
			return result
		TYPE_ARRAY:
			var source: Array = value
			var result: Array = []
			for entry in source:
				result.append(_prepare_display_value(entry, depth + 1))
			return result
		_:
			return value


## 浮点数四舍五入到三位小数。
func _round_float(value: float) -> float:
	return round(value * 1000.0) / 1000.0


## 将敌方机器人节点绑定为 AI 机器人的跟踪目标。
func _bind_enemy_target() -> void:
	var enemy_node := _resolve_enemy_node()
	if enemy_node != null and robot.has_method("set_enemy_target"):
		robot.call("set_enemy_target", enemy_node)


## 构建 Loot 3D 监控覆盖层。
func _build_loot_overlay() -> void:
	loot_overlay = LOOT_OVERLAY_SCRIPT.new()
	loot_overlay.name = "LootOverlay3D"
	get_parent().add_child.call_deferred(loot_overlay)
	if loot_overlay.has_method("setup"):
		loot_overlay.call("setup", robot, target_point)


## 解析敌方节点：优先使用 enemy_path，回退到 ../Enemy。
func _resolve_enemy_node() -> Node3D:
	if enemy_path != NodePath():
		var explicit_node := get_node_or_null(enemy_path)
		if explicit_node is Node3D:
			return explicit_node
	var fallback := get_node_or_null("../Enemy")
	if fallback is Node3D:
		return fallback
	return null


## 将 Lua 底盘模式指令转发到 AI 机器人。
func _apply_robot_chassis_mode(mode: String) -> void:
	if robot.has_method("set_chassis_mode"):
		robot.call("set_chassis_mode", mode)


func _apply_robot_controller_mode(mode: String) -> void:
	if robot.has_method("set_controller_mode"):
		robot.call("set_controller_mode", mode)


func _apply_robot_navigation_enabled(enabled: bool) -> void:
	if robot.has_method("set_navigation_enabled"):
		robot.call("set_navigation_enabled", enabled)


## 将 Lua 云台控制源指令转发到 AI 机器人。
func _apply_robot_gimbal_dominator(dominator_name: String) -> void:
	if robot.has_method("set_gimbal_dominator"):
		robot.call("set_gimbal_dominator", dominator_name)


func _apply_robot_autoaim_enabled(enabled: bool) -> void:
	if robot.has_method("set_autoaim_enabled"):
		robot.call("set_autoaim_enabled", enabled)


## 将 Lua 云台方向指令转发到 AI 机器人。
func _apply_robot_gimbal_direction(angle: float) -> void:
	if robot.has_method("set_gimbal_direction"):
		robot.call("set_gimbal_direction", angle)


## 将 Lua 底盘速度超控指令转发到 AI 机器人。
func _apply_robot_chassis_vel(x: float, y: float) -> void:
	if robot.has_method("set_external_chassis_velocity"):
		robot.call("set_external_chassis_velocity", x, y)


## 从 AI 机器人获取当前模拟资源状态（血量、子弹、金币、死亡状态）。
func _get_robot_resource_state() -> Dictionary:
	if robot.has_method("get_sim_resource_state"):
		var resource = robot.call("get_sim_resource_state")
		if typeof(resource) == TYPE_DICTIONARY:
			return resource
	return {
		"health": 0,
		"bullet": 0,
		"gold": 0,
	}


func _ensure_gold_field_initialized() -> void:
	var user: Dictionary = _get_dict_value(blackboard, "user")
	if user.has("gold"):
		return

	var resource: Dictionary = _get_robot_resource_state()
	var initial_gold: int = int(round(_get_numeric_value(resource.get("gold", 0), 0.0)))
	var patch_user: Dictionary = {"gold": initial_gold}
	var patch: Dictionary = {"user": patch_user}
	_apply_local_patch(patch)
	_apply_robot_user_patch(patch_user)


## 首次收到 blackboard 时，用其中的血量/子弹/金币值初始化机器人状态。
func _sync_robot_resources_from_blackboard_once() -> void:
	if robot_resource_initialized:
		return

	var user: Dictionary = _get_dict_value(blackboard, "user")
	if not user.has("health") and not user.has("bullet") and not user.has("gold"):
		return

	if robot.has_method("sync_resources_from_blackboard"):
		robot.call(
			"sync_resources_from_blackboard",
			user.get("health", null),
			user.get("bullet", null),
			user.get("gold", null)
		)
		robot_resource_initialized = true


## 将超控补丁中的血量/子弹/金币值应用到 AI 机器人。
func _apply_robot_user_patch(user_patch: Dictionary) -> void:
	if robot.has_method("sync_resources_from_blackboard"):
		robot.call(
			"sync_resources_from_blackboard",
			user_patch.get("health", null),
			user_patch.get("bullet", null),
			user_patch.get("gold", null)
		)
	
