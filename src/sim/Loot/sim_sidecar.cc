/**
 * @file sim_sidecar.cc
 * @brief RMCS导航系统仿真
 *
 * 该文件实现了RMCS导航系统的仿真侧车服务，提供Lua脚本运行时环境、
 * 网络通信接口和状态管理功能，用于在仿真环境中运行导航逻辑。
 *
 * @author RMCS Team
 * @version 1.0.0
 */

#if defined(__clang__)
# pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sol/sol.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace rmcs::navigation::sim {

/**
 * @brief JSON值类型定义
 *
 * 支持布尔值、双精度浮点数和字符串三种基本数据类型
 */
using JsonValue = std::variant<bool, double, std::string>;

/**
 * @brief JSON字段定义
 *
 * 包含字段名和对应的JSON值
 */
using JsonField = std::pair<std::string, JsonValue>;

/**
 * @brief JSON字段集合
 *
 * 用于构建JSON对象的字段列表
 */
using JsonFields = std::vector<JsonField>;

/**
 * @brief 日志级别枚举
 *
 * 定义系统日志的严重程度级别
 */
enum class LogLevel : std::uint8_t { Info, Warn, Error };

/**
 * @brief 作用域文件描述符管理类
 *
 * 提供RAII(资源获取即初始化)机制管理文件描述符，
 * 确保文件描述符在作用域结束时自动关闭，避免资源泄漏。
 */
struct ScopedFd {
    int value = -1;

    /// @brief 默认构造函数，创建无效的文件描述符
    ScopedFd() = default;

    /// @brief 显式构造函数，接管现有文件描述符
    /// @param fd 要管理的文件描述符
    explicit ScopedFd(int fd)
        : value{fd} {}

    /// @brief 禁用拷贝构造，避免重复关闭文件描述符
    ScopedFd(const ScopedFd&) = delete;
    auto operator=(const ScopedFd&) -> ScopedFd& = delete;

    /// @brief 移动构造函数，转移文件描述符所有权
    /// @param rhs 源对象
    ScopedFd(ScopedFd&& rhs) noexcept
        : value{std::exchange(rhs.value, -1)} {}

    /// @brief 移动赋值运算符
    /// @param rhs 源对象
    /// @return 当前对象引用
    auto operator=(ScopedFd&& rhs) noexcept -> ScopedFd& {
        if (this == &rhs)
            return *this;
        reset();
        value = std::exchange(rhs.value, -1);
        return *this;
    }

    /// @brief 析构函数，自动关闭文件描述符
    ~ScopedFd() { reset(); }

    /// @brief 重置文件描述符
    /// @param fd 新的文件描述符，默认为-1（无效）
    auto reset(int fd = -1) -> void {
        if (value >= 0) {
            ::close(value);
        }
        value = fd;
    }

    /// @brief 检查文件描述符是否有效
    /// @return true表示有效，false表示无效
    [[nodiscard]] auto valid() const -> bool { return value >= 0; }
};

[[nodiscard]] auto to_string(LogLevel level) -> std::string_view {
    switch (level) {
    case LogLevel::Info: return "info";
    case LogLevel::Warn: return "warn";
    case LogLevel::Error: return "error";
    default: return "unknown";
    }
}

[[nodiscard]] auto trim(std::string text) -> std::string {
    auto is_space = [](char ch) { return std::isspace(static_cast<unsigned char>(ch)); };
    text.erase(text.begin(), std::find_if_not(text.begin(), text.end(), is_space));
    text.erase(std::find_if_not(text.rbegin(), text.rend(), is_space).base(), text.end());
    return text;
}

[[nodiscard]] auto escape_json(std::string_view text) -> std::string {
    auto result = std::string{};
    result.reserve(text.size() + 8);
    for (const auto ch : text) {
        switch (ch) {
        case '\\': result += "\\\\"; break;
        case '"': result += "\\\""; break;
        case '\b': result += "\\b"; break;
        case '\f': result += "\\f"; break;
        case '\n': result += "\\n"; break;
        case '\r': result += "\\r"; break;
        case '\t': result += "\\t"; break;
        default:
            if (static_cast<unsigned char>(ch) < 0x20) {
                result += std::format("\\u{:04x}", static_cast<unsigned>(ch));
            } else {
                result += ch;
            }
            break;
        }
    }
    return result;
}

[[nodiscard]] auto to_json(const JsonFields& fields) -> std::string {
    auto result = std::string{"{"};
    for (std::size_t index = 0; index < fields.size(); ++index) {
        const auto& [name, value] = fields[index];
        result += std::format("\"{}\":", escape_json(name));
        std::visit(
            [&](const auto& entry) {
                using T = std::decay_t<decltype(entry)>;
                if constexpr (std::is_same_v<T, bool>) {
                    result += (entry ? "true" : "false");
                } else if constexpr (std::is_same_v<T, double>) {
                    if (!std::isfinite(entry)) {
                        result += "null";
                    } else {
                        result += std::format("{:.8f}", entry);
                    }
                } else if constexpr (std::is_same_v<T, std::string>) {
                    result += std::format("\"{}\"", escape_json(entry));
                }
            },
            value);

        if (index + 1 < fields.size()) {
            result += ",";
        }
    }
    result += "}";
    return result;
}

/**
 * @brief 递归将YAML节点追加到JSON字符串
 *
 * 深度优先遍历YAML数据结构，将其转换为JSON格式字符串
 * 支持映射、序列和标量值的转换
 *
 * @param result 目标JSON字符串引用
 * @param node 要转换的YAML节点
 */
auto append_json_node(std::string& result, const YAML::Node& node) -> void {
    if (!node || node.IsNull()) {
        result += "null";
        return;
    }

    if (node.IsMap()) {
        result += "{";
        auto first = true;
        for (const auto& entry : node) {
            if (!first) {
                result += ",";
            }
            first = false;

            result += std::format("\"{}\":", escape_json(entry.first.as<std::string>()));
            append_json_node(result, entry.second);
        }
        result += "}";
        return;
    }

    if (node.IsSequence()) {
        result += "[";
        for (auto index = std::size_t{0}; index < node.size(); ++index) {
            if (index > 0) {
                result += ",";
            }
            append_json_node(result, node[index]);
        }
        result += "]";
        return;
    }

    if (auto scalar = node.Scalar(); scalar == "true" || scalar == "false") {
        result += scalar;
        return;
    }

    try {
        result += std::to_string(node.as<long long>());
        return;
    } catch (...) {}

    try {
        auto number = node.as<double>();
        if (!std::isfinite(number)) {
            result += "null";
        } else {
            result += std::format("{:.8f}", number);
        }
        return;
    } catch (...) {}

    result += std::format("\"{}\"", escape_json(node.as<std::string>()));
}

[[nodiscard]] auto to_json(const YAML::Node& node) -> std::string {
    auto result = std::string{};
    append_json_node(result, node);
    return result;
}

/**
 * @brief 发送带换行符的完整数据行
 *
 * 将数据包添加换行符后通过套接字发送，确保完整发送所有数据
 * 使用循环发送机制处理部分发送情况
 *
 * @param fd 目标套接字文件描述符
 * @param payload 要发送的数据内容
 * @throws std::runtime_error 当发送失败时抛出
 */
auto send_line(int fd, std::string_view payload) -> void {
    auto packed = std::string{payload};
    packed.push_back('\n');

    std::size_t offset = 0;
    while (offset < packed.size()) {
        auto written = ::send(fd, packed.data() + offset, packed.size() - offset, MSG_NOSIGNAL);
        if (written <= 0) {
            throw std::runtime_error(std::format("socket send failed: {}", std::strerror(errno)));
        }
        offset += static_cast<std::size_t>(written);
    }
}

/**
 * @brief 条件赋值模板函数
 *
 * 如果YAML节点中存在指定键且值不为空，则将其转换为目标类型并赋值
 *
 * @tparam T 目标类型
 * @param root 根YAML节点
 * @param key 要查找的键名
 * @param target 目标变量引用
 * @param convert 类型转换函数
 */
template <typename T>
auto assign_if_present(
    const YAML::Node& root, std::string_view key, T& target,
    const std::function<T(const YAML::Node&)>& convert) -> void {
    auto value = root[std::string{key}];
    if (!value || value.IsNull()) {
        return;
    }
    target = convert(value);
}

/**
 * @brief 安全标量解析模板函数
 *
 * 尝试解析YAML标量值，如果解析失败则返回默认值
 * 提供异常安全的类型转换机制
 *
 * @tparam T 目标类型
 * @param node 要解析的YAML节点
 * @param fallback 解析失败时的默认值
 * @param convert 类型转换函数
 * @return T 解析结果或默认值
 */
template <typename T>
auto parse_scalar_or(
    const YAML::Node& node, const T& fallback, const std::function<T(const YAML::Node&)>& convert)
    -> T {
    try {
        return convert(node);
    } catch (...) {
        return fallback;
    }
}

[[nodiscard]] auto parse_boolean(const YAML::Node& node, bool fallback = false) -> bool {
    return parse_scalar_or<bool>(
        node, fallback, [](const YAML::Node& value) { return value.as<bool>(); });
}

[[nodiscard]] auto parse_double(const YAML::Node& node, double fallback = 0.0) -> double {
    return parse_scalar_or<double>(
        node, fallback, [](const YAML::Node& value) { return value.as<double>(); });
}

[[nodiscard]] auto parse_string(const YAML::Node& node, std::string fallback = {}) -> std::string {
    return parse_scalar_or<std::string>(
        node, std::move(fallback), [](const YAML::Node& value) { return value.as<std::string>(); });
}

[[nodiscard]] auto normalize_switch_value(std::string value) -> std::string {
    if (value == "MID") {
        return "MIDDLE";
    }
    return value;
}

[[nodiscard]] auto parse_revision(const YAML::Node& node, std::uint64_t fallback = 0)
    -> std::uint64_t {
    if (!node) {
        return fallback;
    }

    try {
        auto value = node.as<long long>();
        if (value < 0) {
            return fallback;
        }
        return static_cast<std::uint64_t>(value);
    } catch (...) {}

    try {
        auto value = node.as<double>();
        if (!std::isfinite(value) || value < 0) {
            return fallback;
        }
        return static_cast<std::uint64_t>(value);
    } catch (...) {}

    return fallback;
}

[[nodiscard]] auto to_steady_timeout(std::chrono::steady_clock::time_point deadline) -> int {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
        return 0;
    }

    auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
    if (timeout < 0) {
        return 0;
    }

    constexpr auto kMaxPollTimeoutMs = 100;
    return static_cast<int>(std::min<std::int64_t>(timeout, kMaxPollTimeoutMs));
}

/**
 * @brief 仿真状态数据结构
 *
 * 包含用户控制状态和元数据信息，用于描述仿真环境的当前状态
 */
struct SimState {
    /**
     * @brief 用户控制状态
     *
     * 包含机器人底盘功率限制、位置坐标、朝向角等控制参数
     */
    struct User {
        double chassis_power_limit = 0.0; ///< 底盘功率限制(W)
        double x = 0.0;                   ///< X坐标(m)
        double y = 0.0;                   ///< Y坐标(m)
        double yaw = 0.0;                 ///< 偏航角(rad)
        std::optional<double> health;     ///< 健康值(可选)
        std::optional<double> bullet;     ///< 子弹数量(可选)
        std::optional<double> gold;       ///< 金币(可选)
        std::optional<double> chassis_power;         ///< 底盘功率(可选)
        std::optional<double> chassis_buffer_energy; ///< 底盘缓冲能量(可选)
        std::optional<bool> chassis_output_status;   ///< 底盘输出状态(可选)
        std::optional<double> shooter_cooling;       ///< 枪口冷却值(可选)
        std::optional<double> shooter_heat_limit;    ///< 枪口热量上限(可选)
        std::optional<double> bullet_42mm;           ///< 42mm 弹量(可选)
        std::optional<double> fortress_17mm_bullet;  ///< 堡垒 17mm 弹量(可选)
        std::optional<double> initial_speed;         ///< 初速(可选)
        std::optional<double> shoot_timestamp;       ///< 发射时间戳(可选)
        bool auto_aim_should_control = false; ///< 自瞄是否接管
    };

    /**
     * @brief 元数据信息
     *
     * 包含时间戳等系统级信息
     */
    struct Meta {
        double timestamp = 0.0; ///< 时间戳(s)
    };

    /**
     * @brief 比赛态仿真状态
     *
     * 包含全局比赛对象（基地、前哨站）的生命值，
     * 用于驱动比赛策略中的守家和前哨站相关判断。
     */
    struct Game {
        std::optional<double> base_health;                      ///< 基地血量(可选)
        std::optional<double> outpost_health;                   ///< 前哨站血量(可选)
        std::optional<double> gold_coin;                        ///< 队伍金币(可选)
        std::optional<double> sync_timestamp;                   ///< 裁判同步时间(可选)
        std::optional<double> hero_health;                      ///< 英雄血量(可选)
        std::optional<double> infantry_1_health;                ///< 步兵1血量(可选)
        std::optional<double> infantry_2_health;                ///< 步兵2血量(可选)
        std::optional<double> engineer_health;                  ///< 工程血量(可选)
        std::optional<double> hero_position_x;                  ///< 英雄位置X(可选)
        std::optional<double> hero_position_y;                  ///< 英雄位置Y(可选)
        std::optional<double> infantry_1_position_x;            ///< 步兵1位置X(可选)
        std::optional<double> infantry_1_position_y;            ///< 步兵1位置Y(可选)
        std::optional<double> infantry_2_position_x;            ///< 步兵2位置X(可选)
        std::optional<double> infantry_2_position_y;            ///< 步兵2位置Y(可选)
        std::optional<double> engineer_position_x;              ///< 工程位置X(可选)
        std::optional<double> engineer_position_y;              ///< 工程位置Y(可选)
        std::optional<double> remaining_time;                   ///< 比赛剩余时间(可选)
        std::optional<double> exchangeable_ammunition_quantity; ///< 可兑换弹药量(可选)
        std::optional<double> our_dart_nmber_of_hits;           ///< 己方飞镖命中次数(可选)
        std::optional<bool> fortress_occupied;                  ///< 己方堡垒是否被占领(可选)
        std::optional<bool> big_energy_mechanism_activated;     ///< 大能量机关是否激活(可选)
        std::optional<bool> small_energy_mechanism_activated;   ///< 小能量机关是否激活(可选)
        std::optional<std::string> stage;                       ///< 比赛阶段(可选)
        std::optional<double> robot_id;                         ///< 机器人ID(可选)
        std::optional<bool> can_confirm_free_revive;            ///< 是否可确认免费复活
        std::optional<bool> can_exchange_instant_revive;        ///< 是否可兑换立即复活
        std::optional<double> instant_revive_cost;              ///< 立即复活花费
        std::optional<double> exchanged_bullet;                 ///< 已兑换弹量
        std::optional<double> remote_bullet_exchange_count;     ///< 远程换弹次数
        std::optional<double> sentry_mode;                      ///< 当前哨兵模式
        std::optional<bool> energy_mechanism_activatable;       ///< 能量机关是否可激活
    };

    struct Play {
        std::optional<std::string> rswitch;
        std::optional<std::string> lswitch;
    };

    struct MapCommand {
        std::optional<double> x;
        std::optional<double> y;
        std::optional<double> keyboard;
        std::optional<double> target_robot_id;
        std::optional<double> source;
        std::optional<double> sequence;
    };

    User user;                                                  ///< 用户控制状态
    Game game;                                                  ///< 比赛全局状态
    Play play;                                                  ///< 遥控拨杆状态
    MapCommand map_command;                                     ///< 小地图指令状态
    Meta meta;                                                  ///< 元数据信息
};

/**
 * @brief 状态覆盖配置结构
 *
 * 用于管理状态覆盖功能，支持动态修改仿真状态
 */
struct OverrideState {
    bool enabled = false;            ///< 是否启用状态覆盖
    std::uint64_t last_rev = 0;      ///< 最后修订版本号
    std::optional<YAML::Node> patch; ///< 状态补丁配置
};

struct SimStatus {
    bool in_resupply_zone = false;
};

struct ExchangeRuntimeState {
    std::optional<double> user_bullet;
    std::optional<double> user_gold;
    std::optional<double> game_gold_coin;
    std::optional<double> exchangeable_ammunition_quantity;
    std::optional<double> exchanged_bullet;
    std::optional<double> remote_bullet_exchange_count;
    bool initialized = false;
    bool pending_sync = false;
};

enum class RelocalizeStateCode : int {
    Idle = 0,
    InFlight = 1,
    Succeeded = 2,
    Failed = 3,
};

struct RelocalizeRuntimeState {
    RelocalizeStateCode state = RelocalizeStateCode::Idle;
    bool success = false;
    std::string message = "idle";
    double fitness_score = 0.0;
    double confidence = 0.0;
    double estimated_x = 0.0;
    double estimated_y = 0.0;
    double estimated_z = 0.0;
    double estimated_qx = 0.0;
    double estimated_qy = 0.0;
    double estimated_qz = 0.0;
    double estimated_qw = 1.0;
};

struct HostCommandState {
    bool navigation_enabled = false;
    bool autoaim_enabled = false;
    bool topic_forward_enabled = false;
    bool navigation_running = false;
    std::string controller_mode = "normal";
    std::string chassis_mode = "auto";
    std::string gimbal_dominator = "manual";
    double gimbal_direction = 0.0;
    double manual_chassis_velocity_x = 0.0;
    double manual_chassis_velocity_y = 0.0;
};

/**
 * @brief Lua运行时环境管理类
 *
 * 负责管理Lua脚本的执行环境，包括脚本加载、函数调用、
 * 状态同步和错误处理等功能。
 */
class LuaRuntime {
private:
    using EmitFn = std::function<void(const JsonFields&)>;           ///< 事件发射函数类型
    using LogFn = std::function<void(LogLevel, const std::string&)>; ///< 日志记录函数类型

    std::unique_ptr<sol::state> lua;                                 ///< Lua状态机实例
    sol::table blackboard;                                           ///< 共享数据黑板
    sol::protected_function on_init;                                 ///< 初始化回调函数
    sol::protected_function on_tick;                                 ///< 定时回调函数
    sol::protected_function on_exit;                                 ///< 退出回调函数
    sol::protected_function on_control;                              ///< 控制输入回调函数
    sol::protected_function blackboard_snapshot;                     ///< 黑板快照函数
    sol::protected_function loot_snapshot;                           ///< Loot语义监控快照函数

    EmitFn emit;                                                     ///< 事件发射器
    LogFn log;                                                       ///< 日志记录器
    bool closed = false;                                             ///< 运行时是否已关闭
    HostCommandState command_state;                                  ///< 宿主命令态
    RelocalizeRuntimeState relocalize_state;                         ///< 重定位可视态
    ExchangeRuntimeState exchange_state;                             ///< 仿真换弹状态
    SimStatus sim_status;                                            ///< Godot 仿真状态

    /**
     * @brief 解包Lua函数调用结果
     *
     * 检查Lua函数调用是否成功，如果失败则抛出异常并包含错误信息
     *
     * @tparam Result 结果类型
     * @param result Lua函数调用结果
     * @param message 错误消息前缀
     * @return Result 解包后的结果
     * @throws std::runtime_error 当Lua函数调用失败时抛出
     */
    template <typename Result>
    auto unwrap_result(Result&& result, std::string_view message) -> Result {
        if (!result.valid()) {
            auto error = result.template get<sol::error>();
            throw std::runtime_error(std::format("{}: {}", message, error.what()));
        }
        return result;
    }

    /**
     * @brief 发射日志事件
     *
     * 将日志消息同时发送到本地日志系统和远程客户端
     *
     * @param level 日志级别
     * @param message 日志消息内容
     */
    auto emit_log(LogLevel level, const std::string& message) -> void {
        log(level, message);
        emit(
            JsonFields{
                {"type", std::string{"sim.log"}},
                {"level", std::string{to_string(level)}},
                {"message", message},
            });
    }

    auto emit_runtime_status() -> void {
        emit(
            JsonFields{
                {"type", std::string{"sim.runtime_status"}},
                {"navigation_enabled", command_state.navigation_enabled},
                {"autoaim_enabled", command_state.autoaim_enabled},
                {"topic_forward_enabled", command_state.topic_forward_enabled},
                {"navigation_running", command_state.navigation_running},
                {"controller_mode", command_state.controller_mode},
                {"chassis_mode", command_state.chassis_mode},
                {"gimbal_dominator", command_state.gimbal_dominator},
                {"gimbal_direction", command_state.gimbal_direction},
                {"manual_chassis_velocity_x", command_state.manual_chassis_velocity_x},
                {"manual_chassis_velocity_y", command_state.manual_chassis_velocity_y},
                {"relocalize_state", static_cast<double>(static_cast<int>(relocalize_state.state))},
                {"relocalize_success", relocalize_state.success},
                {"relocalize_message", relocalize_state.message},
                {"relocalize_fitness_score", relocalize_state.fitness_score},
                {"relocalize_confidence", relocalize_state.confidence},
            });
    }

    auto emit_sim_resource_sync() -> void {
        emit(
            JsonFields{
                {"type", std::string{"sim.resource_sync"}},
                {"bullet", optional_or_zero(exchange_state.user_bullet)},
                {"gold", optional_or_zero(exchange_state.user_gold)},
            });
    }

    auto sync_exchange_state_from_blackboard() -> void {
        auto user = blackboard["user"].get<sol::table>();
        auto game = blackboard["game"].get<sol::table>();

        auto assign_number =
            [](const sol::table& table, const char* key, std::optional<double>& target) -> void {
                auto value = table[key].get_or(sol::object{});
                if (!value.valid() || value == sol::lua_nil || !value.is<double>()) {
                    return;
                }
                target = value.as<double>();
            };

        assign_number(user, "bullet", exchange_state.user_bullet);
        assign_number(user, "gold", exchange_state.user_gold);
        assign_number(game, "gold_coin", exchange_state.game_gold_coin);
        assign_number(
            game, "exchangeable_ammunition_quantity",
            exchange_state.exchangeable_ammunition_quantity);
        assign_number(game, "exchanged_bullet", exchange_state.exchanged_bullet);
        assign_number(
            game, "remote_bullet_exchange_count", exchange_state.remote_bullet_exchange_count);
        exchange_state.initialized = true;
    }

    auto apply_exchange_state_to_blackboard() -> void {
        if (!exchange_state.initialized) {
            return;
        }

        auto user = blackboard["user"].get<sol::table>();
        auto game = blackboard["game"].get<sol::table>();

        if (exchange_state.user_bullet) {
            user["bullet"] = *exchange_state.user_bullet;
        }
        if (exchange_state.user_gold) {
            user["gold"] = *exchange_state.user_gold;
        }
        if (exchange_state.game_gold_coin) {
            game["gold_coin"] = *exchange_state.game_gold_coin;
        }
        if (exchange_state.exchangeable_ammunition_quantity) {
            game["exchangeable_ammunition_quantity"] =
                *exchange_state.exchangeable_ammunition_quantity;
        }
        if (exchange_state.exchanged_bullet) {
            game["exchanged_bullet"] = *exchange_state.exchanged_bullet;
        }
        if (exchange_state.remote_bullet_exchange_count) {
            game["remote_bullet_exchange_count"] = *exchange_state.remote_bullet_exchange_count;
        }
    }

    auto clear_exchange_pending_sync_if_matched() -> void {
        if (!exchange_state.pending_sync) {
            return;
        }

        auto user = blackboard["user"].get<sol::table>();
        auto game = blackboard["game"].get<sol::table>();
        const auto current_bullet = user["bullet"].get_or(0.0);
        const auto current_gold = user["gold"].get_or(0.0);
        const auto current_gold_coin = game["gold_coin"].get_or(0.0);
        const auto current_bank = game["exchangeable_ammunition_quantity"].get_or(0.0);
        const auto current_exchanged = game["exchanged_bullet"].get_or(0.0);
        const auto current_remote_count = game["remote_bullet_exchange_count"].get_or(0.0);

        if (exchange_state.user_bullet && current_bullet != *exchange_state.user_bullet) {
            return;
        }
        if (exchange_state.user_gold && current_gold != *exchange_state.user_gold) {
            return;
        }
        if (exchange_state.game_gold_coin && current_gold_coin != *exchange_state.game_gold_coin) {
            return;
        }
        if (exchange_state.exchangeable_ammunition_quantity
            && current_bank != *exchange_state.exchangeable_ammunition_quantity) {
            return;
        }
        if (exchange_state.exchanged_bullet && current_exchanged != *exchange_state.exchanged_bullet) {
            return;
        }
        if (exchange_state.remote_bullet_exchange_count
            && current_remote_count != *exchange_state.remote_bullet_exchange_count) {
            return;
        }

        exchange_state.pending_sync = false;
    }

    [[nodiscard]] auto optional_or_zero(const std::optional<double>& value) const -> double {
        return value.value_or(0.0);
    }

    auto mark_relocalize_success(std::string_view mode, double x, double y) -> bool {
        relocalize_state.state = RelocalizeStateCode::Succeeded;
        relocalize_state.success = true;
        relocalize_state.message = std::format("{} mocked success", mode);
        relocalize_state.fitness_score = 1.0;
        relocalize_state.confidence = 1.0;
        relocalize_state.estimated_x = x;
        relocalize_state.estimated_y = y;
        relocalize_state.estimated_z = 0.0;
        relocalize_state.estimated_qx = 0.0;
        relocalize_state.estimated_qy = 0.0;
        relocalize_state.estimated_qz = 0.0;
        relocalize_state.estimated_qw = 1.0;
        emit_log(LogLevel::Info, relocalize_state.message);
        emit_runtime_status();
        return true;
    }

    /**
     * @brief 设置Lua包搜索路径
     *
     * 配置Lua模块搜索路径，确保能够正确加载项目中的Lua脚本
     *
     * @param lua_root Lua脚本根目录路径
     * @param loot_lua_root Loot Lua脚本根目录路径
     */
    auto set_package_path(const std::string& lua_root, const std::string& loot_lua_root) -> void {
        auto package = (*lua)["package"].get<sol::table>();
        auto package_path = package["path"].get_or(std::string{});
        package["path"] = std::format(
            "{};{}/?.lua;{}/?/init.lua;{}/?.lua;{}/?/init.lua", package_path, lua_root, lua_root,
            loot_lua_root, loot_lua_root);
    }

    /**
     * @brief 将YAML节点转换为Lua对象
     *
     * 递归地将YAML数据结构转换为对应的Lua数据类型，
     * 支持映射、序列和标量值的转换
     *
     * @param node YAML节点
     * @return sol::object 对应的Lua对象
     */
    [[nodiscard]] auto yaml_to_lua_object(const YAML::Node& node) -> sol::object {
        if (!node || node.IsNull()) {
            return sol::make_object(*lua, sol::lua_nil);
        }

        if (node.IsMap()) {
            auto table = lua->create_table();
            apply_table_patch(table, node);
            return sol::make_object(*lua, table);
        }

        if (node.IsSequence()) {
            auto table = lua->create_table(static_cast<int>(node.size()), 0);
            for (auto index = std::size_t{0}; index < node.size(); ++index) {
                table[index + 1] = yaml_to_lua_object(node[index]);
            }
            return sol::make_object(*lua, table);
        }

        if (auto scalar = node.Scalar(); scalar == "true" || scalar == "false") {
            return sol::make_object(*lua, scalar == "true");
        }

        try {
            return sol::make_object(*lua, node.as<long long>());
        } catch (...) {}

        try {
            return sol::make_object(*lua, node.as<double>());
        } catch (...) {}

        return sol::make_object(*lua, node.as<std::string>());
    }

    /**
     * @brief 应用表格补丁
     *
     * 递归地将YAML补丁应用到Lua表格中，支持嵌套结构的深度合并
     *
     * @param table 目标Lua表格
     * @param patch YAML补丁配置
     */
    auto apply_table_patch(sol::table table, const YAML::Node& patch) -> void {
        if (!patch || !patch.IsMap()) {
            return;
        }

        for (const auto& entry : patch) {
            auto key = entry.first.as<std::string>();
            auto value = entry.second;
            if (!value || value.IsNull()) {
                table[key] = sol::lua_nil;
                continue;
            }

            if (value.IsMap()) {
                auto existing = table[key].get_or(sol::object{});
                if (existing.valid() && existing.get_type() == sol::type::table) {
                    apply_table_patch(existing.as<sol::table>(), value);
                } else {
                    auto nested = lua->create_table();
                    apply_table_patch(nested, value);
                    table[key] = nested;
                }
                continue;
            }

            table[key] = yaml_to_lua_object(value);
        }
    }

    /**
     * @brief 判断Lua表格是否为数组
     *
     * 检查Lua表格是否具有连续的整数键（从1开始），
     * 用于确定应该将表格转换为YAML序列还是映射
     *
     * @param table 要检查的Lua表格
     * @return true 表格是数组，false 表格是映射
     */
    [[nodiscard]] auto lua_table_is_array(sol::table table) -> bool {
        auto count = std::size_t{0};
        auto has_entries = false;
        for (const auto& entry : table) {
            has_entries = true;
            if (!entry.first.is<double>()) {
                return false;
            }

            auto numeric_key = entry.first.as<double>();
            auto integer_key = static_cast<std::size_t>(numeric_key);
            if (numeric_key != static_cast<double>(integer_key) || integer_key == 0) {
                return false;
            }

            count = std::max(count, integer_key);
        }

        if (!has_entries) {
            return false;
        }

        for (auto index = std::size_t{1}; index <= count; ++index) {
            auto value = table[static_cast<int>(index)].get_or(sol::object{});
            if (!value.valid() || value == sol::lua_nil) {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief 将Lua对象转换为YAML节点
     *
     * 递归地将Lua数据类型转换为对应的YAML数据结构，
     * 支持nil、布尔值、数字、字符串和表格的转换
     *
     * @param object Lua对象
     * @return YAML::Node 对应的YAML节点
     */
    [[nodiscard]] auto lua_to_yaml(const sol::object& object) -> YAML::Node {
        switch (object.get_type()) {
        case sol::type::lua_nil: return YAML::Node{};
        case sol::type::boolean: return YAML::Node{object.as<bool>()};
        case sol::type::number: return YAML::Node{object.as<double>()};
        case sol::type::string: return YAML::Node{object.as<std::string>()};
        case sol::type::table: {
            auto table = object.as<sol::table>();
            if (lua_table_is_array(table)) {
                auto sequence = YAML::Node{YAML::NodeType::Sequence};
                auto count = std::size_t{0};
                for (const auto& entry : table) {
                    count = std::max(count, static_cast<std::size_t>(entry.first.as<double>()));
                }
                for (auto index = std::size_t{1}; index <= count; ++index) {
                    sequence.push_back(
                        lua_to_yaml(table[static_cast<int>(index)].get<sol::object>()));
                }
                return sequence;
            }

            auto mapping = YAML::Node{YAML::NodeType::Map};
            for (const auto& entry : table) {
                if (!entry.first.is<std::string>()) {
                    continue;
                }

                mapping[entry.first.as<std::string>()] =
                    lua_to_yaml(entry.second.as<sol::object>());
            }
            return mapping;
        }
        default: return YAML::Node{};
        }
    }

    /**
     * @brief 注入API函数到Lua环境
     *
     * 将C++实现的API函数注册到Lua环境中，供Lua脚本调用
     * 包括日志记录、导航控制、底盘控制等功能
     */
    auto inject_api() -> void {
        auto api_result = unwrap_result(
            lua->safe_script("return require('api')", sol::script_pass_on_error),
            "failed to load api");

        auto api = api_result.get<sol::table>();
        api.set_function(
            "info", [this](const std::string& text) { emit_log(LogLevel::Info, text); });
        api.set_function(
            "warn", [this](const std::string& text) { emit_log(LogLevel::Warn, text); });
        api.set_function(
            "fuck", [this](const std::string& text) { emit_log(LogLevel::Error, text); });
        api.set_function("update_enable_control", [this](bool enable) {
            command_state.navigation_enabled = enable;
            emit(
                JsonFields{
                    {"type", std::string{"sim.navigation_enabled"}},
                    {"enabled", enable},
                });
            emit_runtime_status();
        });

        api.set_function("send_target", [this](double x, double y) {
            emit(
                JsonFields{
                    {"type", std::string{"sim.nav_target"}},
                    {"x", x},
                    {"y", y},
                });
        });
        api.set_function("update_gimbal_direction", [this](double angle) {
            command_state.gimbal_direction = angle;
            emit(
                JsonFields{
                    {"type", std::string{"sim.gimbal_direction"}},
                    {"angle", angle},
                });
            emit_runtime_status();
        });
        api.set_function("update_gimbal_dominator", [this](const std::string& name) {
            command_state.gimbal_dominator = name;
            emit(
                JsonFields{
                    {"type", std::string{"sim.gimbal_dominator"}},
                    {"name", name},
                });
            emit_runtime_status();
        });
        api.set_function("switch_controller", [this](const std::string& mode) {
            command_state.controller_mode = mode;
            emit(
                JsonFields{
                    {"type", std::string{"sim.controller_mode"}},
                    {"mode", mode},
                });
            emit_runtime_status();
        });
        api.set_function("update_chassis_mode", [this](const std::string& mode) {
            command_state.chassis_mode = mode;
            emit(
                JsonFields{
                    {"type", std::string{"sim.chassis_mode"}},
                    {"mode", mode},
                });
            emit_runtime_status();
        });
        api.set_function("update_enable_autoaim", [this](bool enable) {
            command_state.autoaim_enabled = enable;
            command_state.gimbal_dominator = enable ? "auto" : "scan";
            emit(
                JsonFields{
                    {"type", std::string{"sim.gimbal_dominator"}},
                    {"name", std::string{enable ? "auto" : "scan"}},
                });
            emit(
                JsonFields{
                    {"type", std::string{"sim.autoaim_enabled"}},
                    {"enabled", enable},
                });
            emit_runtime_status();
        });
        api.set_function("update_chassis_vel", [this](double x, double y) {
            command_state.manual_chassis_velocity_x = x;
            command_state.manual_chassis_velocity_y = y;
            emit(
                JsonFields{
                    {"type", std::string{"sim.chassis_vel"}},
                    {"x", x},
                    {"y", y},
                });
            emit_runtime_status();
        });
        api.set_function("relocalize_initial", [this](double x, double y, double) {
            return mark_relocalize_success("relocalize_initial", x, y);
        });
        api.set_function("relocalize_local", [this](double x, double y, double) {
            return mark_relocalize_success("relocalize_local", x, y);
        });
        api.set_function("relocalize_wide", [this](double x, double y, double) {
            return mark_relocalize_success("relocalize_wide", x, y);
        });
        api.set_function("relocalize_status", [this]() {
            return lua->create_table_with(
                "state", static_cast<int>(relocalize_state.state), "success",
                relocalize_state.success, "message", relocalize_state.message, "fitness_score",
                relocalize_state.fitness_score, "confidence", relocalize_state.confidence,
                "estimated_x", relocalize_state.estimated_x, "estimated_y",
                relocalize_state.estimated_y, "estimated_z", relocalize_state.estimated_z,
                "estimated_qx", relocalize_state.estimated_qx, "estimated_qy",
                relocalize_state.estimated_qy, "estimated_qz", relocalize_state.estimated_qz,
                "estimated_qw", relocalize_state.estimated_qw);
        });
        api.set_function("exchange_17mm_bullet", [this](int amount) {
            sync_exchange_state_from_blackboard();

            constexpr int kBatchSize = 100;
            constexpr int kResupplyCostPerBatch = 100;
            constexpr int kRemoteCostPerBatch = 150;

            const int current_exchanged =
                static_cast<int>(std::lround(optional_or_zero(exchange_state.exchanged_bullet)));
            if (amount <= 0) {
                exchange_state.exchanged_bullet = 0.0;
                apply_exchange_state_to_blackboard();
                emit_log(LogLevel::Info, "exchange_17mm_bullet canceled");
                return;
            }

            const int delta = amount - current_exchanged;
            if (delta <= 0) {
                exchange_state.exchanged_bullet = static_cast<double>(amount);
                apply_exchange_state_to_blackboard();
                emit_log(
                    LogLevel::Info,
                    std::format(
                        "exchange_17mm_bullet accepted existing target: current={} requested={}",
                        current_exchanged, amount));
                return;
            }

            if (delta % kBatchSize != 0) {
                emit_log(
                    LogLevel::Warn,
                    std::format(
                        "exchange_17mm_bullet rejected: delta {} is not multiple of {}",
                        delta, kBatchSize));
                return;
            }

            const int batch_count = delta / kBatchSize;
            const bool in_resupply_zone = sim_status.in_resupply_zone;
            const int cost_per_batch =
                in_resupply_zone ? kResupplyCostPerBatch : kRemoteCostPerBatch;
            const int total_cost = batch_count * cost_per_batch;

            const int current_bullet =
                static_cast<int>(std::lround(optional_or_zero(exchange_state.user_bullet)));
            const int current_gold =
                static_cast<int>(std::lround(optional_or_zero(exchange_state.user_gold)));
            const int current_gold_coin =
                static_cast<int>(std::lround(optional_or_zero(exchange_state.game_gold_coin)));
            const int current_ammo_bank = static_cast<int>(std::lround(
                optional_or_zero(exchange_state.exchangeable_ammunition_quantity)));
            const int current_remote_count = static_cast<int>(std::lround(
                optional_or_zero(exchange_state.remote_bullet_exchange_count)));

            if (current_gold < total_cost || current_gold_coin < total_cost) {
                emit_log(
                    LogLevel::Warn,
                    std::format(
                        "exchange_17mm_bullet rejected: insufficient gold (user={}, game={}, cost={})",
                        current_gold, current_gold_coin, total_cost));
                return;
            }

            if (current_ammo_bank < delta) {
                emit_log(
                    LogLevel::Warn,
                    std::format(
                        "exchange_17mm_bullet rejected: insufficient ammo bank (bank={}, need={})",
                        current_ammo_bank, delta));
                return;
            }

            exchange_state.user_bullet = static_cast<double>(current_bullet + delta);
            exchange_state.user_gold = static_cast<double>(current_gold - total_cost);
            exchange_state.game_gold_coin = static_cast<double>(current_gold_coin - total_cost);
            exchange_state.exchangeable_ammunition_quantity =
                static_cast<double>(current_ammo_bank - delta);
            exchange_state.exchanged_bullet = static_cast<double>(amount);
            if (in_resupply_zone) {
                exchange_state.remote_bullet_exchange_count = static_cast<double>(current_remote_count);
            } else {
                exchange_state.remote_bullet_exchange_count =
                    static_cast<double>(current_remote_count + batch_count);
            }
            exchange_state.pending_sync = true;
            apply_exchange_state_to_blackboard();
            emit_sim_resource_sync();

            emit_log(
                LogLevel::Info,
                std::format(
                    "exchange_17mm_bullet simulated: requested={} delta={} zone={} cost={} bullet={} gold={} bank={} remote_count={}",
                    amount, delta, in_resupply_zone ? "resupply" : "remote", total_cost,
                    static_cast<int>(std::lround(optional_or_zero(exchange_state.user_bullet))),
                    static_cast<int>(std::lround(optional_or_zero(exchange_state.user_gold))),
                    static_cast<int>(std::lround(
                        optional_or_zero(exchange_state.exchangeable_ammunition_quantity))),
                    static_cast<int>(std::lround(
                        optional_or_zero(exchange_state.remote_bullet_exchange_count)))));
        });
        api.set_function("switch_mode", [this](int mode) {
            auto game = blackboard["game"].get<sol::table>();
            game["sentry_mode"] = mode;
            emit_log(LogLevel::Info, std::format("switch_mode simulated: {}", mode));
        });
        api.set_function("confirm_revive", [this]() {
            auto game = blackboard["game"].get<sol::table>();
            game["can_confirm_free_revive"] = false;
            emit_log(LogLevel::Info, "confirm_revive simulated");
        });

        // Sidecar mode: keep these entry points but avoid touching ROS processes.
        api.set_function("switch_topic_forward", [this](bool enable) {
            command_state.topic_forward_enabled = enable;
            emit_runtime_status();
            emit_log(LogLevel::Info, std::format("switch_topic_forward -> {}", enable));
        });
        api.set_function("restart_navigation", [this](const sol::table&) {
            command_state.navigation_running = true;
            emit(
                JsonFields{
                    {"type", std::string{"sim.navigation_lifecycle"}},
                    {"state", std::string{"running"}},
                });
            emit_runtime_status();
            emit_log(LogLevel::Info, "restart_navigation simulated");
            return std::tuple{true, std::string{"ok"}};
        });
        api.set_function("stop_navigation", [this]() {
            command_state.navigation_running = false;
            emit(
                JsonFields{
                    {"type", std::string{"sim.navigation_lifecycle"}},
                    {"state", std::string{"stopped"}},
                });
            emit_runtime_status();
            emit_log(LogLevel::Info, "stop_navigation simulated");
            return std::tuple{true, std::string{"ok"}};
        });
    }

    auto inject_option(const std::optional<YAML::Node>& option_patch) -> void {
        auto option_result = unwrap_result(
            lua->safe_script("return require('option')", sol::script_pass_on_error),
            "failed to load option");
        auto option = option_result.get<sol::table>();

        option["decision"] = std::string{"auxiliary"};
        option["enable_goal_topic_forward"] = false;
        option["sim_mode"] = true;

        if (option_patch && option_patch->IsMap()) {
            apply_table_patch(option, *option_patch);
        }
    }

public:
    /**
     * @brief LuaRuntime构造函数
     *
     * 初始化Lua运行时环境，加载必要的库、配置包路径、注入API函数，
     * 并加载指定的Lua端点脚本
     *
     * @param lua_root Lua脚本根目录路径
     * @param loot_lua_root Loot Lua脚本根目录路径
     * @param endpoint 要加载的Lua端点名称
     * @param emit_action 事件发射函数
     * @param log_action 日志记录函数
     * @param option_patch 可选配置补丁
     * @throws std::runtime_error 当Lua环境初始化失败时抛出
     */

    explicit LuaRuntime(
        const std::string& lua_root, const std::string& loot_lua_root, const std::string& endpoint,
        EmitFn emit_action, LogFn log_action, std::optional<YAML::Node> option_patch = std::nullopt,
        std::optional<YAML::Node> blackboard_patch = std::nullopt)
        : lua{std::make_unique<sol::state>()}
        , emit{std::move(emit_action)}
        , log{std::move(log_action)} {
        lua->open_libraries(
            sol::lib::base, sol::lib::coroutine, sol::lib::math, sol::lib::os, sol::lib::package,
            sol::lib::string, sol::lib::table, sol::lib::debug, sol::lib::io);

        set_package_path(lua_root, loot_lua_root);
        inject_api();
        inject_option(option_patch);

        auto loot_result = unwrap_result(
            lua->safe_script("return require('Loot')", sol::script_pass_on_error),
            "failed to load Loot");
        auto loot = loot_result.get<sol::table>();
        auto loot_install = loot["install"].get<sol::protected_function>();
        if (!loot_install.valid()) {
            throw std::runtime_error("Loot must define install()");
        }
        auto installed_result = unwrap_result(loot_install(endpoint), "Loot install failed");
        auto installed = installed_result.get<sol::table>();
        loot_snapshot = installed["snapshot"];
        if (!loot_snapshot.valid()) {
            throw std::runtime_error("Loot install result must define snapshot()");
        }

        auto required = std::format("require('endpoint.{}')", endpoint);
        auto endpoint_result = unwrap_result(
            lua->safe_script(required, sol::script_pass_on_error), "failed to load endpoint");
        (void)endpoint_result;

        auto blackboard_sync_result = unwrap_result(
            lua->safe_script("return require('Loot.blackboard_sync')", sol::script_pass_on_error),
            "failed to load Loot.blackboard_sync");
        auto blackboard_sync = blackboard_sync_result.get<sol::table>();

        blackboard = (*lua)["blackboard"];
        on_init = (*lua)["on_init"];
        on_tick = (*lua)["on_tick"];
        if (!on_init.valid() || !on_tick.valid()) {
            throw std::runtime_error("lua endpoint must define on_init() and on_tick()");
        }

        on_exit = (*lua)["on_exit"];
        if (on_exit == sol::lua_nil) {
            on_exit = lua->safe_script("return function() end", sol::script_pass_on_error);
        }
        on_control = (*lua)["on_control"];
        if (on_control == sol::lua_nil) {
            on_control =
                lua->safe_script("return function(_, _, _) end", sol::script_pass_on_error);
        }

        blackboard_snapshot = blackboard_sync["snapshot"];
        if (!blackboard_snapshot.valid()) {
            throw std::runtime_error("Loot.blackboard_sync must define snapshot()");
        }

        if (blackboard_patch && blackboard_patch->IsMap()) {
            apply_override_patch(*blackboard_patch);
        }

        unwrap_result(on_init(), "lua on_init failed");
        emit_log(LogLevel::Info, "lua runtime initialized");
        emit_runtime_status();
    }

    /// @brief 析构函数，自动关闭Lua运行时
    ~LuaRuntime() { close(); }

    /**
     * @brief 关闭Lua运行时
     *
     * 执行清理操作，调用Lua端点的on_exit函数，
     * 确保资源正确释放，避免内存泄漏
     */
    auto close() -> void {
        if (closed) {
            return;
        }
        closed = true;
        try {
            unwrap_result(on_exit(), "lua on_exit failed");
        } catch (const std::exception& exception) {
            emit_log(LogLevel::Error, std::format("on_exit failed: {}", exception.what()));
        }
    }

    /**
     * @brief 应用仿真状态到Lua黑板
     *
     * 将C++端的仿真状态同步到Lua运行时的共享数据黑板，
     * 支持可选字段的健康值和子弹数量更新
     *
     * @param state 要应用的仿真状态
     */

    auto apply_state(const SimState& state) -> void {
        auto user = blackboard["user"].get<sol::table>();
        user["chassis_power_limit"] = state.user.chassis_power_limit;
        user["x"] = state.user.x;
        user["y"] = state.user.y;
        user["yaw"] = state.user.yaw;
        if (state.user.health) {
            user["health"] = *state.user.health;
        }
        if (state.user.bullet) {
            user["bullet"] = *state.user.bullet;
        }
        if (state.user.gold) {
            user["gold"] = *state.user.gold;
        }
        user["auto_aim_should_control"] = state.user.auto_aim_should_control;

        auto game = blackboard["game"].get<sol::table>();
        if (state.game.base_health) {
            game["base_health"] = *state.game.base_health;
        }
        if (state.game.outpost_health) {
            game["outpost_health"] = *state.game.outpost_health;
        }
        if (state.game.gold_coin) {
            game["gold_coin"] = *state.game.gold_coin;
        }
        if (state.game.remaining_time) {
            game["remaining_time"] = *state.game.remaining_time;
        }
        if (state.game.exchangeable_ammunition_quantity) {
            game["exchangeable_ammunition_quantity"] = *state.game.exchangeable_ammunition_quantity;
        }
        if (state.game.our_dart_nmber_of_hits) {
            game["our_dart_nmber_of_hits"] = *state.game.our_dart_nmber_of_hits;
        }
        if (state.game.fortress_occupied) {
            game["fortress_occupied"] = *state.game.fortress_occupied;
        }
        if (state.game.big_energy_mechanism_activated) {
            game["big_energy_mechanism_activated"] = *state.game.big_energy_mechanism_activated;
        }
        if (state.game.small_energy_mechanism_activated) {
            game["small_energy_mechanism_activated"] = *state.game.small_energy_mechanism_activated;
        }
        if (state.game.stage) {
            game["stage"] = *state.game.stage;
        }
        if (state.game.can_confirm_free_revive) {
            game["can_confirm_free_revive"] = *state.game.can_confirm_free_revive;
        }
        if (state.game.exchanged_bullet) {
            game["exchanged_bullet"] = *state.game.exchanged_bullet;
        }
        if (state.game.sentry_mode) {
            game["sentry_mode"] = *state.game.sentry_mode;
        }

        auto play = blackboard["play"].get<sol::table>();
        if (state.play.rswitch) {
            play["rswitch"] = *state.play.rswitch;
        }
        if (state.play.lswitch) {
            play["lswitch"] = *state.play.lswitch;
        }

        auto meta = blackboard["meta"].get<sol::table>();
        meta["timestamp"] = state.meta.timestamp;

        sync_exchange_state_from_blackboard();
        clear_exchange_pending_sync_if_matched();
        if (!exchange_state.pending_sync) {
            if (state.user.bullet) {
                exchange_state.user_bullet = *state.user.bullet;
            }
            if (state.user.gold) {
                exchange_state.user_gold = *state.user.gold;
            }
            if (state.game.gold_coin) {
                exchange_state.game_gold_coin = *state.game.gold_coin;
            }
            if (state.game.exchangeable_ammunition_quantity) {
                exchange_state.exchangeable_ammunition_quantity =
                    *state.game.exchangeable_ammunition_quantity;
            }
            if (state.game.exchanged_bullet) {
                exchange_state.exchanged_bullet = *state.game.exchanged_bullet;
            }
            if (state.game.remote_bullet_exchange_count) {
                exchange_state.remote_bullet_exchange_count =
                    *state.game.remote_bullet_exchange_count;
            }
        }
        apply_exchange_state_to_blackboard();
    }

    /**
     * @brief 应用状态覆盖补丁
     *
     * 将YAML格式的状态覆盖补丁应用到Lua黑板，
     * 支持动态修改仿真状态参数
     *
     * @param patch 状态覆盖补丁
     */

    auto apply_override_patch(const YAML::Node& patch) -> void {
        apply_table_patch(blackboard, patch);
        sync_exchange_state_from_blackboard();
    }

    /**
     * @brief 获取Lua黑板快照
     *
     * 调用Lua端的快照函数，将当前黑板状态转换为YAML格式，
     * 用于状态报告和决策状态构建
     *
     * @return YAML::Node 黑板状态快照
     * @throws std::runtime_error 当快照操作失败时抛出
     */
    [[nodiscard]] auto snapshot_blackboard() -> YAML::Node {
        auto result = unwrap_result(blackboard_snapshot(), "lua blackboard snapshot failed");
        return lua_to_yaml(result.get<sol::object>());
    }

    [[nodiscard]] auto snapshot_loot() -> YAML::Node {
        auto result = unwrap_result(loot_snapshot(), "lua loot snapshot failed");
        return lua_to_yaml(result.get<sol::object>());
    }

    [[nodiscard]] auto snapshot_host_runtime() const -> YAML::Node {
        auto state = YAML::Node{YAML::NodeType::Map};
        state["navigation_enabled"] = command_state.navigation_enabled;
        state["autoaim_enabled"] = command_state.autoaim_enabled;
        state["topic_forward_enabled"] = command_state.topic_forward_enabled;
        state["navigation_running"] = command_state.navigation_running;
        state["controller_mode"] = command_state.controller_mode;
        state["chassis_mode"] = command_state.chassis_mode;
        state["gimbal_dominator"] = command_state.gimbal_dominator;
        state["gimbal_direction"] = command_state.gimbal_direction;
        state["manual_chassis_velocity_x"] = command_state.manual_chassis_velocity_x;
        state["manual_chassis_velocity_y"] = command_state.manual_chassis_velocity_y;

        auto relocalize = YAML::Node{YAML::NodeType::Map};
        relocalize["state"] = static_cast<int>(relocalize_state.state);
        relocalize["success"] = relocalize_state.success;
        relocalize["message"] = relocalize_state.message;
        relocalize["fitness_score"] = relocalize_state.fitness_score;
        relocalize["confidence"] = relocalize_state.confidence;
        relocalize["estimated_x"] = relocalize_state.estimated_x;
        relocalize["estimated_y"] = relocalize_state.estimated_y;
        relocalize["estimated_z"] = relocalize_state.estimated_z;
        relocalize["estimated_qx"] = relocalize_state.estimated_qx;
        relocalize["estimated_qy"] = relocalize_state.estimated_qy;
        relocalize["estimated_qz"] = relocalize_state.estimated_qz;
        relocalize["estimated_qw"] = relocalize_state.estimated_qw;
        state["relocalize"] = relocalize;
        return state;
    }

    /**
     * @brief 执行定时回调
     *
     * 调用Lua端点的on_tick函数，执行周期性的逻辑处理，
     * 这是仿真循环的核心执行点
     *
     * @throws std::runtime_error 当定时回调执行失败时抛出
     */

    auto tick() -> void { unwrap_result(on_tick(), "lua on_tick failed"); }

    /**
     * @brief 输入控制命令
     *
     * 将控制输入传递给Lua端点，用于处理底盘速度控制等命令
     *
     * @param vx X方向速度
     * @param vy Y方向速度
     * @param qx 旋转控制参数
     * @throws std::runtime_error 当控制命令处理失败时抛出
     */

    auto feed_control(double vx, double vy, double qx) -> void {
        unwrap_result(on_control(vx, vy, qx), "lua on_control failed");
    }

    auto update_sim_status(const SimStatus& status) -> void { sim_status = status; }

    /**
     * @brief 调用仿真目标设置函数
     *
     * 如果Lua端点定义了on_sim_set_target函数，则调用该函数设置目标位置
     *
     * @param x 目标X坐标
     * @param y 目标Y坐标
     */
    auto call_sim_set_target(double x, double y) -> void {
        auto function = (*lua)["on_sim_set_target"];
        if (!function.valid() || function.get_type() != sol::type::function) {
            return;
        }
        auto callback = function.get<sol::protected_function>();
        unwrap_result(callback(x, y), "lua on_sim_set_target failed");
    }

    /**
     * @brief 调用仿真启动函数
     *
     * 如果Lua端点定义了on_sim_start函数，则调用该函数启动仿真，
     * 支持可选的目标位置参数
     *
     * @param x 可选的目标X坐标
     * @param y 可选的目标Y坐标
     */
    auto call_sim_start(const std::optional<double>& x, const std::optional<double>& y) -> void {
        auto function = (*lua)["on_sim_start"];
        if (!function.valid() || function.get_type() != sol::type::function) {
            return;
        }
        auto callback = function.get<sol::protected_function>();
        if (x && y) {
            unwrap_result(callback(*x, *y), "lua on_sim_start failed");
            return;
        }

        unwrap_result(callback(sol::lua_nil, sol::lua_nil), "lua on_sim_start failed");
    }
};

/**
 * @brief 命令行参数结构
 *
 * 存储仿真侧车服务的运行配置参数
 */
struct Args {
    std::string host = "0.0.0.0";                                 ///< 服务器绑定主机地址
    int port = 34567;                                             ///< 服务器绑定端口号
    std::string endpoint = "train";                               ///< Lua端点名称
    std::string lua_root = RMCS_NAVIGATION_SOURCE_DIR "/src/lua"; ///< Lua脚本根目录
    std::string loot_lua_root = RMCS_NAVIGATION_LOOT_LUA_DIR;     ///< Loot Lua脚本根目录
    double tick_hz = 10.0;                                        ///< 定时器频率(Hz)
    int state_timeout_ms = 500;                                   ///< 状态超时时间(毫秒)
    std::optional<std::string> option_file;                       ///< 可选配置文件路径
    std::optional<std::string> blackboard_file;                   ///< 可选黑板补丁路径
};

[[nodiscard]] auto parse_args(int argc, char** argv) -> Args {
    auto args = Args{};
    for (auto index = 1; index < argc; ++index) {
        auto token = std::string_view{argv[index]};
        auto take_next = [&](std::string_view name) {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::format("{} requires a value", name));
            }
            ++index;
            return std::string{argv[index]};
        };

        if (token == "--host") {
            args.host = take_next(token);
        } else if (token == "--port") {
            args.port = std::stoi(take_next(token));
        } else if (token == "--endpoint") {
            args.endpoint = take_next(token);
        } else if (token == "--lua-root") {
            args.lua_root = take_next(token);
        } else if (token == "--loot-lua-root") {
            args.loot_lua_root = take_next(token);
        } else if (token == "--tick-hz") {
            args.tick_hz = std::stod(take_next(token));
        } else if (token == "--state-timeout-ms") {
            args.state_timeout_ms = std::stoi(take_next(token));
        } else if (token == "--option-file") {
            args.option_file = take_next(token);
        } else if (token == "--blackboard-file") {
            args.blackboard_file = take_next(token);
        } else if (token == "--help" || token == "-h") {
            std::cout << "rmcs-navigation-sim-sidecar\n"
                      << "  --host <host>                (default: 0.0.0.0)\n"
                      << "  --port <port>                (default: 34567)\n"
                      << "  --endpoint <name>            (default: train)\n"
                      << "  --lua-root <path>            (default: <source>/src/lua)\n"
                      << "  --loot-lua-root <path>       (default: <source>/src/sim/Loot/lua)\n"
                      << "  --tick-hz <hz>               (default: 10)\n"
                      << "  --state-timeout-ms <ms>      (default: 500)\n"
                      << "  --option-file <yaml/json>\n"
                      << "  --blackboard-file <yaml/json>\n";
            std::exit(0);
        } else {
            throw std::runtime_error(std::format("unknown argument: {}", token));
        }
    }

    if (args.port <= 0 || args.port > 65535) {
        throw std::runtime_error("port must be between 1 and 65535");
    }
    if (!(args.tick_hz > 0.0)) {
        throw std::runtime_error("tick-hz must be > 0");
    }
    if (args.state_timeout_ms < 0) {
        throw std::runtime_error("state-timeout-ms must be >= 0");
    }

    return args;
}

[[nodiscard]] auto create_server_socket(const std::string& host, int port) -> ScopedFd {
    auto hints = addrinfo{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    auto service = std::to_string(port);
    addrinfo* result = nullptr;
    auto ret = ::getaddrinfo(host.c_str(), service.c_str(), &hints, &result);
    if (ret != 0) {
        throw std::runtime_error(std::format("getaddrinfo failed: {}", gai_strerror(ret)));
    }

    auto cleanup = std::unique_ptr<addrinfo, decltype(&::freeaddrinfo)>{result, ::freeaddrinfo};
    for (auto* it = result; it != nullptr; it = it->ai_next) {
        auto fd = ScopedFd{::socket(it->ai_family, it->ai_socktype, it->ai_protocol)};
        if (!fd.valid()) {
            continue;
        }

        int enable = 1;
        ::setsockopt(fd.value, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

        if (::bind(fd.value, it->ai_addr, it->ai_addrlen) != 0) {
            continue;
        }

        if (::listen(fd.value, 1) != 0) {
            continue;
        }

        return fd;
    }

    throw std::runtime_error("failed to bind server socket");
}

/**
 * @brief 从YAML负载更新仿真状态
 *
 * 解析接收到的YAML消息，更新用户状态和元数据
 * 支持可选字段的健康值和子弹数量更新
 *
 * @param root 接收到的YAML根节点
 * @param state 要更新的仿真状态引用
 */
auto update_state_from_payload(const YAML::Node& root, SimState& state) -> void {
    auto user = root["user"];
    if (user && user.IsMap()) {
        assign_if_present<double>(
            user, "chassis_power_limit", state.user.chassis_power_limit,
            [](const YAML::Node& node) { return parse_double(node); });
        assign_if_present<double>(
            user, "x", state.user.x, [](const YAML::Node& node) { return parse_double(node); });
        assign_if_present<double>(
            user, "y", state.user.y, [](const YAML::Node& node) { return parse_double(node); });
        assign_if_present<double>(
            user, "yaw", state.user.yaw, [](const YAML::Node& node) { return parse_double(node); });
        if (user["health"] && !user["health"].IsNull()) {
            state.user.health = parse_double(user["health"]);
        }
        if (user["bullet"] && !user["bullet"].IsNull()) {
            state.user.bullet = parse_double(user["bullet"]);
        }
        if (user["gold"] && !user["gold"].IsNull()) {
            state.user.gold = parse_double(user["gold"]);
        }
        if (user["auto_aim_should_control"] && !user["auto_aim_should_control"].IsNull()) {
            state.user.auto_aim_should_control = parse_boolean(user["auto_aim_should_control"]);
        }
    }

    auto game = root["game"];
    if (game && game.IsMap()) {
        if (game["base_health"] && !game["base_health"].IsNull()) {
            state.game.base_health = parse_double(game["base_health"]);
        }
        if (game["outpost_health"] && !game["outpost_health"].IsNull()) {
            state.game.outpost_health = parse_double(game["outpost_health"]);
        }
        if (game["gold_coin"] && !game["gold_coin"].IsNull()) {
            state.game.gold_coin = parse_double(game["gold_coin"]);
        }
        if (game["remaining_time"] && !game["remaining_time"].IsNull()) {
            state.game.remaining_time = parse_double(game["remaining_time"]);
        }
        if (game["exchangeable_ammunition_quantity"]
            && !game["exchangeable_ammunition_quantity"].IsNull()) {
            state.game.exchangeable_ammunition_quantity =
                parse_double(game["exchangeable_ammunition_quantity"]);
        }
        if (game["our_dart_nmber_of_hits"] && !game["our_dart_nmber_of_hits"].IsNull()) {
            state.game.our_dart_nmber_of_hits = parse_double(game["our_dart_nmber_of_hits"]);
        }
        if (game["fortress_occupied"] && !game["fortress_occupied"].IsNull()) {
            state.game.fortress_occupied = parse_boolean(game["fortress_occupied"]);
        }
        if (game["big_energy_mechanism_activated"]
            && !game["big_energy_mechanism_activated"].IsNull()) {
            state.game.big_energy_mechanism_activated =
                parse_boolean(game["big_energy_mechanism_activated"]);
        }
        if (game["small_energy_mechanism_activated"]
            && !game["small_energy_mechanism_activated"].IsNull()) {
            state.game.small_energy_mechanism_activated =
                parse_boolean(game["small_energy_mechanism_activated"]);
        }
        if (game["stage"] && !game["stage"].IsNull()) {
            state.game.stage = parse_string(game["stage"]);
        }
        if (game["can_confirm_free_revive"] && !game["can_confirm_free_revive"].IsNull()) {
            state.game.can_confirm_free_revive = parse_boolean(game["can_confirm_free_revive"]);
        }
        if (game["exchanged_bullet"] && !game["exchanged_bullet"].IsNull()) {
            state.game.exchanged_bullet = parse_double(game["exchanged_bullet"]);
        }
        if (game["remote_bullet_exchange_count"]
            && !game["remote_bullet_exchange_count"].IsNull()) {
            state.game.remote_bullet_exchange_count =
                parse_double(game["remote_bullet_exchange_count"]);
        }
        if (game["sentry_mode"] && !game["sentry_mode"].IsNull()) {
            state.game.sentry_mode = parse_double(game["sentry_mode"]);
        }
    }

    auto play = root["play"];
    if (play && play.IsMap()) {
        if (play["rswitch"] && !play["rswitch"].IsNull()) {
            state.play.rswitch = normalize_switch_value(parse_string(play["rswitch"]));
        }
        if (play["lswitch"] && !play["lswitch"].IsNull()) {
            state.play.lswitch = normalize_switch_value(parse_string(play["lswitch"]));
        }
    }

    auto meta = root["meta"];
    if (meta && meta.IsMap()) {
        assign_if_present<double>(
            meta, "timestamp", state.meta.timestamp,
            [](const YAML::Node& node) { return parse_double(node); });
    }
}

/**
 * @brief 检查YAML节点是否包含有效条目
 *
 * 验证节点是否为非空映射，用于判断是否需要进行进一步处理
 *
 * @param node 要检查的YAML节点
 * @return true 节点包含有效条目，false 节点为空或无效
 */
[[nodiscard]] auto has_entries(const YAML::Node& node) -> bool {
    return node && node.IsMap() && node.size() > 0;
}

/**
 * @brief 过滤状态覆盖补丁
 *
 * 从完整的覆盖补丁中提取允许修改的字段，
 * 限制客户端只能修改特定的状态字段，确保系统安全性
 *
 * @param patch 原始覆盖补丁
 * @return YAML::Node 过滤后的安全补丁
 */
[[nodiscard]] auto filter_override_patch(const YAML::Node& patch) -> YAML::Node {
    auto result = YAML::Node{YAML::NodeType::Map};
    if (!patch || !patch.IsMap()) {
        return result;
    }

    auto user = patch["user"];
    if (user && user.IsMap()) {
        auto filtered_user = YAML::Node{YAML::NodeType::Map};
        if (user["health"]) {
            filtered_user["health"] = user["health"];
        }
        if (user["bullet"]) {
            filtered_user["bullet"] = user["bullet"];
        }
        if (user["gold"]) {
            filtered_user["gold"] = user["gold"];
        }
        if (has_entries(filtered_user)) {
            result["user"] = filtered_user;
        }
    }

    auto game = patch["game"];
    if (game && game.IsMap()) {
        auto filtered_game = YAML::Node{YAML::NodeType::Map};
        if (game["stage"])
            filtered_game["stage"] = game["stage"];
        if (game["remaining_time"])
            filtered_game["remaining_time"] = game["remaining_time"];
        if (game["our_dart_nmber_of_hits"])
            filtered_game["our_dart_nmber_of_hits"] = game["our_dart_nmber_of_hits"];
        if (game["fortress_occupied"])
            filtered_game["fortress_occupied"] = game["fortress_occupied"];
        if (game["big_energy_mechanism_activated"])
            filtered_game["big_energy_mechanism_activated"] =
                game["big_energy_mechanism_activated"];
        if (game["small_energy_mechanism_activated"])
            filtered_game["small_energy_mechanism_activated"] =
                game["small_energy_mechanism_activated"];
        if (game["gold_coin"])
            filtered_game["gold_coin"] = game["gold_coin"];
        if (game["exchangeable_ammunition_quantity"])
            filtered_game["exchangeable_ammunition_quantity"] =
                game["exchangeable_ammunition_quantity"];
        if (game["exchanged_bullet"])
            filtered_game["exchanged_bullet"] = game["exchanged_bullet"];
        if (game["remote_bullet_exchange_count"])
            filtered_game["remote_bullet_exchange_count"] = game["remote_bullet_exchange_count"];
        if (has_entries(filtered_game))
            result["game"] = filtered_game;
    }

    auto play = patch["play"];
    if (play && play.IsMap()) {
        auto filtered_play = YAML::Node{YAML::NodeType::Map};
        if (play["rswitch"]) {
            filtered_play["rswitch"] = play["rswitch"];
        }
        if (play["lswitch"]) {
            filtered_play["lswitch"] = play["lswitch"];
        }
        if (has_entries(filtered_play)) {
            result["play"] = filtered_play;
        }
    }

    return result;
}

/**
 * @brief 构建决策状态快照
 *
 * 从完整的Lua黑板快照中提取决策相关的关键状态信息，
 * 用于向客户端报告当前决策状态和任务进度
 *
 * @param snapshot Lua黑板完整快照
 * @return YAML::Node 精简的决策状态信息
 */
[[nodiscard]] auto build_decision_state(const YAML::Node& snapshot) -> YAML::Node {
    auto state = YAML::Node{YAML::NodeType::Map};

    auto meta = snapshot["meta"];
    if (meta && meta.IsMap() && meta["fsm_state"]) {
        state["fsm_state"] = meta["fsm_state"];
    }

    auto result = snapshot["result"];
    if (result && result.IsMap()) {
        if (result["intent"]) {
            state["intent"] = result["intent"];
        }
        if (result["task"]) {
            state["task"] = result["task"];
        }
        if (result["progress"]) {
            state["progress"] = result["progress"];
        }
        if (result["last_reason"]) {
            state["last_reason"] = result["last_reason"];
        }
        if (result["job_done"]) {
            state["job_done"] = result["job_done"];
        }
        if (result["job_success"]) {
            state["job_success"] = result["job_success"];
        }
    }

    auto game = snapshot["game"];
    if (game && game.IsMap() && game["stage"]) {
        state["stage"] = game["stage"];
    }

    auto user = snapshot["user"];
    if (user && user.IsMap()) {
        if (user["health"]) {
            state["health"] = user["health"];
        }
        if (user["bullet"]) {
            state["bullet"] = user["bullet"];
        }
    }

    return state;
}

struct ClientRuntime {
    SimState state;
    OverrideState override_state;
    SimStatus sim_status;
    bool has_input = false;
    bool has_initial_remaining_time = false;
    bool waiting_for_initial_remaining_time_logged = false;
    bool start_bridge_armed = false;
    int start_bridge_phase = 0;
    std::chrono::steady_clock::time_point last_input_time = std::chrono::steady_clock::now();
};

/**
 * @brief 处理客户端消息
 *
 * 根据消息类型分发处理逻辑，支持状态更新、控制输入、
 * 状态覆盖和仿真控制等不同类型的消息
 *
 * @param runtime Lua运行时环境
 * @param root 接收到的消息根节点
 * @param context 客户端运行时上下文
 * @param log 日志记录函数
 */
auto process_message(
    LuaRuntime& runtime, const YAML::Node& root, ClientRuntime& context,
    const std::function<void(LogLevel, const std::string&)>& log) -> void {
    auto type_node = root["type"];
    if (!type_node) {
        log(LogLevel::Warn, "received message without type");
        return;
    }

    auto type = parse_string(type_node);
    auto now = std::chrono::steady_clock::now();

    if (type == "sim.hello") {
        log(LogLevel::Info, "client hello");
        return;
    }

    if (type == "sim.input") {
        auto payload = root["input"];
        if (!payload || !payload.IsMap()) {
            payload = root;
        }

        update_state_from_payload(payload, context.state);
        auto sim_status = payload["sim_status"];
        if (sim_status && sim_status.IsMap()) {
            context.sim_status.in_resupply_zone =
                parse_boolean(sim_status["in_resupply_zone"], false);
        }
        if (context.state.game.remaining_time.has_value()) {
            context.has_initial_remaining_time = true;
            context.waiting_for_initial_remaining_time_logged = false;
        }

        auto control = payload["control"];
        if (control && control.IsMap()) {
            auto vx = parse_double(control["vx"], 0.0);
            auto vy = parse_double(control["vy"], 0.0);
            auto qx = parse_double(control["qx"], 0.0);
            runtime.feed_control(vx, vy, qx);
        }

        context.has_input = true;
        context.last_input_time = now;
        return;
    }

    if (type == "sim.override_mode") {
        context.override_state.enabled = parse_boolean(root["enabled"], false);
        if (!context.override_state.enabled) {
            context.override_state.patch.reset();
        }

        log(LogLevel::Info,
            std::format(
                "override mode -> {}", context.override_state.enabled ? "enabled" : "disabled"));

        context.has_input = true;
        context.last_input_time = now;
        return;
    }

    if (type == "sim.override_patch") {
        if (!root["rev"]) {
            log(LogLevel::Warn, "override patch missing rev");
            return;
        }

        auto rev = parse_revision(root["rev"], 0);
        if (rev <= context.override_state.last_rev) {
            log(LogLevel::Warn, std::format("ignored stale override patch rev={}", rev));
            return;
        }

        auto patch = root["patch"];
        if (!patch || !patch.IsMap()) {
            log(LogLevel::Warn, std::format("override patch rev={} missing patch map", rev));
            context.override_state.last_rev = rev;
            return;
        }

        auto filtered = filter_override_patch(patch);
        context.override_state.last_rev = rev;

        if (!has_entries(filtered)) {
            log(LogLevel::Warn, std::format("override patch rev={} has no writable fields", rev));
            return;
        }

        if (!context.override_state.enabled) {
            context.override_state.patch.reset();
            log(LogLevel::Warn,
                std::format("override patch rev={} ignored while mode disabled", rev));
            return;
        }

        context.override_state.patch = filtered;
        context.has_input = true;
        context.last_input_time = now;
        return;
    }

    if (type == "sim.command") {
        auto command = parse_string(root["command"]);
        if (command == "start_decision") {
            auto x = std::optional<double>{};
            auto y = std::optional<double>{};
            if (root["x"]) {
                x = parse_double(root["x"], 0.0);
            }
            if (root["y"]) {
                y = parse_double(root["y"], 0.0);
            }
            runtime.call_sim_start(x, y);
            context.start_bridge_armed = true;
            context.start_bridge_phase = 0;
            context.has_input = true;
            context.last_input_time = now;
            return;
        }

        if (command == "set_target") {
            auto x = parse_double(root["x"], 0.0);
            auto y = parse_double(root["y"], 0.0);
            runtime.call_sim_set_target(x, y);
            return;
        }

        log(LogLevel::Warn, std::format("unknown sim.command: {}", command));
        return;
    }

    log(LogLevel::Warn, std::format("unknown message type: {}", type));
}

auto handle_client(const Args& args, int client_fd) -> void {
    auto context = ClientRuntime{};
    auto input_is_stale = false;
    auto input_buffer = std::string{};
    auto bb_rev = std::uint64_t{0};

    auto write_message = [client_fd](const JsonFields& fields) {
        auto payload = to_json(fields);
        send_line(client_fd, payload);
    };

    auto logger = [&](LogLevel level, const std::string& message) {
        std::cerr << std::format("[sim-sidecar][{}] {}\n", to_string(level), message);
    };

    auto option_patch = std::optional<YAML::Node>{};
    if (args.option_file) {
        option_patch = YAML::LoadFile(*args.option_file);
    }
    auto startup_blackboard_patch = std::optional<YAML::Node>{};
    if (args.blackboard_file) {
        startup_blackboard_patch = YAML::LoadFile(*args.blackboard_file);
    }

    auto runtime = LuaRuntime{
        args.lua_root, args.loot_lua_root, args.endpoint, write_message, logger, option_patch,
        startup_blackboard_patch,
    };

    auto emit_runtime_state = [&]() {
        auto snapshot = runtime.snapshot_blackboard();
        ++bb_rev;

        auto blackboard_msg = YAML::Node{YAML::NodeType::Map};
        blackboard_msg["type"] = "sim.blackboard";
        blackboard_msg["bb_rev"] = static_cast<long long>(bb_rev);
        blackboard_msg["blackboard"] = snapshot;
        send_line(client_fd, to_json(blackboard_msg));

        auto decision_msg = YAML::Node{YAML::NodeType::Map};
        decision_msg["type"] = "sim.decision_state";
        decision_msg["bb_rev"] = static_cast<long long>(bb_rev);
        decision_msg["state"] = build_decision_state(snapshot);
        send_line(client_fd, to_json(decision_msg));

        auto loot_msg = YAML::Node{YAML::NodeType::Map};
        loot_msg["type"] = "loot.snapshot";
        loot_msg["bb_rev"] = static_cast<long long>(bb_rev);
        loot_msg["loot"] = runtime.snapshot_loot();
        send_line(client_fd, to_json(loot_msg));

        auto runtime_msg = YAML::Node{YAML::NodeType::Map};
        runtime_msg["type"] = "sim.runtime_state";
        runtime_msg["bb_rev"] = static_cast<long long>(bb_rev);
        runtime_msg["state"] = runtime.snapshot_host_runtime();
        send_line(client_fd, to_json(runtime_msg));
    };

    emit_runtime_state();

    auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>{1.0 / args.tick_hz});
    auto next_tick = std::chrono::steady_clock::now() + period;

    while (true) {
        auto timeout = to_steady_timeout(next_tick);
        auto fd = pollfd{
            .fd = client_fd,
            .events = POLLIN,
            .revents = 0,
        };
        auto result = ::poll(&fd, 1, timeout);
        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::format("poll failed: {}", std::strerror(errno)));
        }

        if ((fd.revents & POLLIN) != 0) {
            auto chunk = std::array<char, 4096>{};
            auto recv_size = ::recv(client_fd, chunk.data(), chunk.size(), 0);
            if (recv_size <= 0) {
                return;
            }

            input_buffer.append(chunk.data(), static_cast<std::size_t>(recv_size));
            while (true) {
                auto pos = input_buffer.find('\n');
                if (pos == std::string::npos) {
                    break;
                }

                auto line = trim(input_buffer.substr(0, pos));
                input_buffer.erase(0, pos + 1);
                if (line.empty()) {
                    continue;
                }

                try {
                    auto message = YAML::Load(line);
                    if (!message || !message.IsMap()) {
                        logger(LogLevel::Warn, "ignored non-object message");
                        continue;
                    }
                    process_message(runtime, message, context, logger);
                } catch (const std::exception& exception) {
                    logger(LogLevel::Warn, std::format("invalid message: {}", exception.what()));
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            continue;
        }

        while (next_tick <= now) {
            next_tick += period;
        }

        if (!context.has_input) {
            continue;
        }

        auto stale_duration = std::chrono::milliseconds{args.state_timeout_ms};
        if ((now - context.last_input_time) > stale_duration) {
            if (!input_is_stale) {
                input_is_stale = true;
                logger(
                    LogLevel::Warn,
                    std::format("input timeout > {}ms, tick paused", args.state_timeout_ms));
            }
            continue;
        }

        if (input_is_stale) {
            input_is_stale = false;
            logger(LogLevel::Info, "input stream resumed");
        }

        runtime.update_sim_status(context.sim_status);
        runtime.apply_state(context.state);
        if (context.override_state.enabled && context.override_state.patch
            && context.override_state.patch->IsMap()) {
            runtime.apply_override_patch(*context.override_state.patch);
        }

        if (context.start_bridge_armed) {
            auto patch = YAML::Node{YAML::NodeType::Map};
            patch["game"]["stage"] = "STARTED";

            if (context.start_bridge_phase == 0) {
                patch["play"]["rswitch"] = "MIDDLE";
                ++context.start_bridge_phase;
            } else if (context.start_bridge_phase == 1) {
                patch["play"]["rswitch"] = "UP";
                ++context.start_bridge_phase;
            } else if (context.start_bridge_phase == 2) {
                patch["play"]["rswitch"] = "MIDDLE";
                ++context.start_bridge_phase;
                context.start_bridge_armed = false;
            }

            runtime.apply_override_patch(patch);
        }

        auto blackboard_snapshot = runtime.snapshot_blackboard();
        auto game = blackboard_snapshot["game"];
        auto stage = parse_string(game["stage"], "UNKNOWN");
        if (stage == "STARTED" && !context.has_initial_remaining_time) {
            if (!context.waiting_for_initial_remaining_time_logged) {
                context.waiting_for_initial_remaining_time_logged = true;
                logger(
                    LogLevel::Info, "waiting for initial game.remaining_time before STARTED tick");
            }
            emit_runtime_state();
            continue;
        }

        runtime.tick();

        if (context.override_state.enabled && context.override_state.patch
            && context.override_state.patch->IsMap()) {
            runtime.apply_override_patch(*context.override_state.patch);
        }

        emit_runtime_state();
    }
}

auto run(const Args& args) -> int {
    auto server = create_server_socket(args.host, args.port);
    std::cerr << std::format(
        "[sim-sidecar][info] listening on {}:{}, endpoint={}\n", args.host, args.port,
        args.endpoint);

    while (true) {
        auto client = ScopedFd{::accept(server.value, nullptr, nullptr)};
        if (!client.valid()) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::format("accept failed: {}", std::strerror(errno)));
        }

        std::cerr << "[sim-sidecar][info] client connected\n";
        try {
            handle_client(args, client.value);
        } catch (const std::exception& exception) {
            std::cerr << std::format("[sim-sidecar][error] client failed: {}\n", exception.what());
        }
        std::cerr << "[sim-sidecar][info] client disconnected\n";
    }
}

} // namespace rmcs::navigation::sim

auto main(int argc, char** argv) -> int {
    try {
        auto args = rmcs::navigation::sim::parse_args(argc, argv);
        return rmcs::navigation::sim::run(args);
    } catch (const std::exception& exception) {
        std::cerr << std::format("[sim-sidecar][fatal] {}\n", exception.what());
        return 1;
    }
}
