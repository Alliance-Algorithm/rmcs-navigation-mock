---
--- train.lua — 训练 endpoint
---
--- 架构：
---   order（拨杆序列检测）─→ 触发训练启动
---   edges（上升沿检测）─→ rswitch UP 强制撤退
---
--- 拨杆触发：
---   rswitch MIDDLE → UP → MIDDLE  →  重启导航 + 启动 GETOUT
---   rswitch UP                     →  * → ESCAPE_TO_HOME（手动强制撤退）
---
--- 状态触发：
---   low_health  →  true        →  * → ESCAPE_TO_HOME
---   low_bullet  →  true        →  * → ESCAPE_TO_HOME
---   health_ready → true        →  ESCAPE_TO_HOME → CRUISE
---   bullet_ready → true        →  ESCAPE_TO_HOME → CRUISE
---   auto_aim    →  true        →  CRUISE → CHASE
---   auto_aim    →  false       →  CHASE → CRUISE
---

local action = require("action")
local ascii = require("util.ascii_art")
local clock = require("util.clock")
local fsm = require("util.fsm")
local order = require("util.order")
local option = require("option")

local Scheduler = require("util.scheduler")
local scheduler = Scheduler.new()
local request = Scheduler.request

local edges = require("util.edge").new()

local intent_idle = require("intent.idle")
local intent_getout = require("intent.getout")
local intent_cruise = require("intent.cruise")
local intent_chase = require("intent.chase")
local intent_escape = require("intent.escape-to-home")

blackboard = require("blackboard").singleton()

local Intent = {
	idle           = "idle",
	getout         = "getout",
	cruise         = "cruise",
	chase          = "chase",
	escape_to_home = "escape_to_home",
}

on_init = function()
	action:info(ascii.banner)
	action:warn("⚠️ TRAIN 训练模式")

	clock:reset(blackboard.meta.timestamp)

	option:set_handler(function(error)
		action:fuck("while fetch option: " .. error)
	end)

	if option.enable_goal_topic_forward then
		action:switch_topic_forward(true)
	end

	action:bind(scheduler)
	action:info("use decision: '" .. option.decision .. "'")

	local intent_fsm = fsm:new(Intent.idle)

	intent_fsm:use {
		state = Intent.idle,
		enter = intent_idle.enter,
		event = intent_idle.event,
	}
	intent_fsm:use {
		state = Intent.getout,
		enter = intent_getout.enter,
		event = intent_getout.event,
	}
	intent_fsm:use {
		state = Intent.cruise,
		enter = intent_cruise.enter,
		event = intent_cruise.event,
	}
	intent_fsm:use {
		state = Intent.chase,
		enter = intent_chase.enter,
		event = intent_chase.event,
	}
	intent_fsm:use {
		state = Intent.escape_to_home,
		enter = intent_escape.enter,
		event = intent_escape.event,
	}

	if not intent_fsm:init_ready(Intent) then
		error("意图状态机未完全初始化，有未注册的状态")
	end

	-- ================================================================
	-- 拨杆序列触发：rswitch MIDDLE → UP → MIDDLE → 启动训练
	-- ================================================================
	scheduler:append_task(function()
		local switch_order = order.new(blackboard.getter.rswitch, 5.0)
		switch_order:on({ "MIDDLE", "UP", "MIDDLE" }, function()
			action:info("训练导航即将重启")
			action:restart_navigation({
				global_map = "rmuc",
				launch_livox = true,
				launch_odin1 = false,
				use_sim_time = false,
				launch_relocation = true
			})
			request:sleep(5)

			action:switch_mode(1)

			local max_retries = 3
			local reloc_ok = false
			for attempt = 1, max_retries do
				action:info(string.format(
					"[RELOC] 第 %d/%d 次 initial 重定位...", attempt, max_retries))
				local ok, st = action:relocalize_initial(0, 0, 0, 20)
				if ok then
					reloc_ok = true
					break
				end
				action:warn(string.format(
					"[RELOC] 第 %d/%d 次失败: %s", attempt, max_retries,
					tostring(st and st.message or "unknown")))
			end

			if not reloc_ok then
				action:fuck("[RELOC] 3 次 initial 重定位全部失败，取消出区")
				return
			end

			action:info("[ORDER] 重定位成功，启动出区")
			intent_fsm:start_on(Intent.getout)
		end)

		while true do
			switch_order:spin()
			request:yield()
		end
	end)

	-- ================================================================
	-- 上升沿触发：右拨杆 UP → 强制撤退（手动安全保护）
	-- ================================================================
	-- edges:on(blackboard.getter.rswitch, "UP", function()
	-- 	action:warn("[EDGE] 右拨杆 UP，强制撤退")
	-- 	intent_fsm:start_on(Intent.escape_to_home)
	-- end)

	-- ================================================================
	-- FSM 驱动任务 + 健康/弹药布尔上升沿检测
	--
	-- 这里不用 edges:on() 是因为 boolean getter 的初始值可能是 true，
	-- edges 的 _last 初始为 nil 会在第一帧误触发。所以在此手动做
	-- 上升沿检测：记录上一帧值，当前为 true 且上一帧为 false 时触发。
	-- ================================================================

	scheduler:append_task(function()
		while true do
			action:switch_navigation(blackboard.play.rswitch == "UP")
			request:yield()
			if blackboard.game.can_confirm_free_revive then
				action:confirm_revive()
			end
		end
	end)

	scheduler:append_task(function()
		local prev = {
			low_health   = blackboard.condition.low_health(),
			low_bullet   = blackboard.condition.low_bullet(),
			health_ready = blackboard.condition.health_ready(),
			bullet_ready = blackboard.condition.bullet_ready(),
			auto_aim     = blackboard.user.auto_aim_should_control,
		}

		while true do
			local cur = {
				low_health   = blackboard.condition.low_health(),
				low_bullet   = blackboard.condition.low_bullet(),
				health_ready = blackboard.condition.health_ready(),
				bullet_ready = blackboard.condition.bullet_ready(),
				auto_aim     = blackboard.user.auto_aim_should_control,
			}

			if cur.low_health and not prev.low_health then
				action:warn("[EDGE] 血量低于阈值")
				intent_fsm:start_on(Intent.escape_to_home)
			end
			-- if cur.low_bullet and not prev.low_bullet then
			-- 	action:warn("[EDGE] 弹药低于阈值")
			-- 	intent_fsm:start_on(Intent.escape_to_home)
			-- end

			if cur.health_ready and not prev.health_ready then
				action:info("[EDGE] 血量已恢复")
				intent_fsm:start_on(Intent.cruise)
				if blackboard.user.auto_aim_should_control then
					action:info("[EDGE] 进入巡航时自瞄已锁定，直接追击")
					intent_fsm:start_on(Intent.chase)
				end
			end
			-- if cur.bullet_ready and not prev.bullet_ready then
			-- 	action:info("[EDGE] 弹药已补充")
			-- 	intent_fsm:start_on(Intent.cruise)
			-- 	if blackboard.user.auto_aim_should_control then
			-- 		action:info("[EDGE] 进入巡航时自瞄已锁定，直接追击")
			-- 		intent_fsm:start_on(Intent.chase)
			-- 	end
			-- end

			if cur.auto_aim and not prev.auto_aim then
				-- if intent_fsm.details.current_state == Intent.cruise then
				action:info("[EDGE] 自瞄锁定目标，进入追击")
				intent_fsm:start_on(Intent.chase)
				-- end
			end
			if not cur.auto_aim and prev.auto_aim then
				if intent_fsm.details.current_state == Intent.chase then
					action:info("[EDGE] 自瞄丢失目标，返回巡航")
					intent_fsm:start_on(Intent.cruise)
				end
			end

			prev = cur
			intent_fsm:spin_once()
			request:yield()
		end
	end)

	scheduler:append_task(function()
		while true do
			local current = blackboard.game.sentry_mode
			local target = blackboard.game.target_mode

			if target ~= 0 and current ~= target then
				action:warn("[SENTRY] 模式不匹配: " .. current .. " → " .. target)
				action:switch_mode(target)
			end
			request:yield()
		end
	end)
	scheduler:append_task(function()
		while true do
			request:sleep(1)
			action:info("[SENTRY] mode: " .. blackboard.game.sentry_mode)
		end
	end)
end

on_tick = function()
	clock:update(blackboard.meta.timestamp)
	edges:spin()
	scheduler:spin_once()
end

on_exit = function()
	action:stop_navigation()
end

on_control = function(vx, vy, qx)
	if blackboard.play.rswitch == "UP" then
		local _ = qx
		action:update_chassis_vel(vx, vy)
		action:info("accept speed")
	else
		action:update_chassis_vel(0, 0)
		action:info("cancel speed")
	end
end
