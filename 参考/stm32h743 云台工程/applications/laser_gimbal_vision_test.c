  /**
 * @file laser_gimbal_vision_test.c
 * @brief 激光云台视觉追踪测试和使用示例
 * @version 1.0
 * @date 2025-07-30
 * @author Your Name
 * @copyright Copyright (c) 2025
 */

#include "laser_gimbal_vision_tracker.h"
#include "laser_gimbal.h"
#include <rtthread.h>
#include <string.h>

#define DBG_TAG "vision_test"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* 全局变量 */
 vision_tracker_t g_vision_tracker;
 bool g_vision_initialized = false;

/* 外部变量 */
extern laser_gimbal_t g_laser_gimbal;

/* 回调函数 */
static void on_vision_target_acquired(vision_target_t target)
{
    LOG_I("🎯 Target acquired: (%.1f, %.1f), confidence: %.2f",
          target.pixel_coord.x, target.pixel_coord.y, target.confidence);
}

static void on_vision_target_lost(void)
{
    LOG_W("❌ Target lost!");
}

static void on_vision_tracking_error(int error_code)
{
    LOG_E("⚠️ Vision tracking error: %d", error_code);
}

/* ======================== 初始化和配置函数 ======================== */

/**
 * @brief 初始化视觉追踪系统
 */
static rt_err_t vision_tracking_system_init(uint16_t width, uint16_t height, float fov_h, float fov_v)
{
    rt_err_t result;

    LOG_I("=== Initializing Vision Tracking System ===");

    /* 检查云台是否已初始化 */
    if (g_laser_gimbal.yaw_motor == RT_NULL || g_laser_gimbal.pitch_motor == RT_NULL)
    {
        LOG_E("Laser gimbal not initialized! Please run 'laser_init' first.");
        return -RT_ERROR;
    }

    /* 初始化视觉追踪器 */
    result = vision_tracker_init(&g_vision_tracker, &g_laser_gimbal, width, height, fov_h, fov_v);
    if (result != RT_EOK)
    {
        LOG_E("Failed to initialize vision tracker: %d", result);
        return result;
    }

    /* 设置回调函数 */
    g_vision_tracker.on_target_acquired = on_vision_target_acquired;
    g_vision_tracker.on_target_lost = on_vision_target_lost;
    g_vision_tracker.on_tracking_error = on_vision_tracking_error;

    g_vision_initialized = true;

    LOG_I("=== Vision Tracking System Initialized Successfully ===");
    return RT_EOK;
}

/**
 * @brief 视觉追踪演示测试
 */
static void vision_tracking_demo(void)
{
    LOG_I("=== Starting Vision Tracking Demonstration ===");

    if (!g_vision_initialized)
    {
        LOG_E("Vision tracker not initialized! Please run 'vision_init' first.");
        return;
    }

    /* 1. 基本追踪演示 */
    LOG_I("Demo 1: Basic target tracking simulation");
    
    vision_tracker_set_mode(&g_vision_tracker, VISION_TRACK_MODE_CONTINUOUS);
    vision_tracker_enable(&g_vision_tracker, true);
    
    /* 模拟目标从左向右移动 */
    LOG_W("Simulating target shaking left and right...");
    // 左右抖动：在中心附近左右反复移动
    int center_x11 = 320;
    int amplitude = 20;
    int steps = 4;
    for (int i = 0; i < steps; ++i)
    {
        // 向右
        vision_tracker_input_target(&g_vision_tracker, center_x11 + amplitude, 240, 0.9f);
        rt_thread_mdelay(400);
        LOG_I("Target at: (%d, 240)", center_x11 + amplitude);

        // 向左
        vision_tracker_input_target(&g_vision_tracker, center_x11 - amplitude, 240, 0.9f);
        rt_thread_mdelay(400);
        LOG_I("Target at: (%d, 240)", center_x11 - amplitude);
    }
    // 最后回到中心
    vision_tracker_input_target(&g_vision_tracker, center_x11, 240, 0.9f);
    LOG_I("Target at: (%d, 240)", center_x11);
    
    rt_thread_mdelay(1000);

    /* 2. 垂直移动演示 */
    LOG_W("Demo 2: Vertical target movement");
    
    /* 模拟目标从上向下移动 */
    LOG_W("Simulating target moving up and down...");
    // 上下抖动：在中心附近上下反复移动
    int center_y1 = 240;
    int amplitude1 = 16;
    int steps1 = 4;
    for (int i = 0; i < steps1; ++i)
    {
        // 向下
        vision_tracker_input_target(&g_vision_tracker, 320, center_y1 + amplitude1, 0.85f);
        rt_thread_mdelay(400);
        LOG_I("Target at: (320, %d)", center_y1 + amplitude1);

        // 向上
        vision_tracker_input_target(&g_vision_tracker, 320, center_y1 - amplitude1, 0.85f);
        rt_thread_mdelay(400);
        LOG_I("Target at: (320, %d)", center_y1 - amplitude1);
    }
    // 最后回到中心
    vision_tracker_input_target(&g_vision_tracker, 320, center_y1, 0.85f);
    LOG_I("Target at: (320, %d)", center_y1);
    
    rt_thread_mdelay(1000);

    /* 3. 圆形轨迹演示 */
    LOG_W("Demo 3: Circular target movement");
    
    float center_x = 320.0f, center_y = 240.0f, radius = 10.0f;
    
    LOG_W("Simulating circular target movement...");
    for (int angle = 0; angle < 360; angle += 30)
    {
        float rad = angle * M_PI / 180.0f;
        float x = center_x + radius * cosf(rad);
        float y = center_y + radius * sinf(rad);
        
        vision_tracker_input_target(&g_vision_tracker, x, y, 0.8f);
        rt_thread_mdelay(300);
        LOG_I("Target at: (%.1f, %.1f), angle: %d°", x, y, angle);
    }
    
    /* 4. 目标丢失和重新获取演示 */
    LOG_W("Demo 4: Target lost and reacquisition");
    
    LOG_W("Target lost simulation...");
    vision_tracker_target_lost(&g_vision_tracker);
    rt_thread_mdelay(2000);
    
    LOG_I("Target reacquired...");
    vision_tracker_input_target(&g_vision_tracker, 320, 240, 0.95f);
    rt_thread_mdelay(1000);

    /* 停止追踪 */
    vision_tracker_enable(&g_vision_tracker, false);
    
    LOG_W("=== Vision Tracking Demonstration Completed ===");
}

/* ======================== MSH命令实现 ======================== */

/**
 * @brief 视觉追踪初始化命令
 */
void vision_init(int argc, char **argv)
{
    uint16_t width = VISION_TRACKER_DEFAULT_CAMERA_WIDTH;
    uint16_t height = VISION_TRACKER_DEFAULT_CAMERA_HEIGHT;
    float fov_h = VISION_TRACKER_DEFAULT_FOV_H;
    float fov_v = VISION_TRACKER_DEFAULT_FOV_V;

    if (argc >= 3)
    {
        width = atoi(argv[1]);
        height = atoi(argv[2]);
    }
    if (argc >= 5)
    {
        fov_h = atof(argv[3]);
        fov_v = atof(argv[4]);
    }

    if (argc > 1 && argc != 3 && argc != 5)
    {
        LOG_D("Usage: vision_init [width height] [fov_h fov_v]\n");
        LOG_D("Example: vision_init 640 480 60.0 45.0\n");
        LOG_D("Default: vision_init (640x480, 60°x45°)\n");
        return;
    }

    rt_err_t result = vision_tracking_system_init(width, height, fov_h, fov_v);
    if (result == RT_EOK)
    {
        LOG_D("✅ Vision tracking system initialized: %dx%d, FOV: %.1f°x%.1f°\n",
               width, height, fov_h, fov_v);
    }
    else
    {
        LOG_D("❌ Failed to initialize vision tracking system: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_init, Initialize vision tracking system: vision_init [width height] [fov_h fov_v]);

/**
 * @brief 视觉追踪使能命令
 */
static void vision_enable(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: vision_enable <on|off>\n");
        LOG_D("Example: vision_enable on\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    bool enable = (rt_strcmp(argv[1], "on") == 0);
    
    rt_err_t result = vision_tracker_enable(&g_vision_tracker, enable);
    if (result == RT_EOK)
    {
        LOG_D("Vision tracking: %s\n", enable ? "ENABLED" : "DISABLED");
    }
    else
    {
        LOG_D("Failed to %s vision tracking: %d\n", enable ? "enable" : "disable", result);
    }
}
MSH_CMD_EXPORT(vision_enable, Enable/disable vision tracking: vision_enable <on|off>);

/**
 * @brief 设置视觉追踪模式命令
 */
static void vision_mode(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: vision_mode <manual|continuous|lock>\n");
        LOG_D("  manual     - Manual mode (single adjustment)\n");
        LOG_D("  continuous - Continuous tracking mode\n");
        LOG_D("  lock       - Lock-on mode (high precision)\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    vision_track_mode_t mode;
    const char* mode_name;
    
    if (rt_strcmp(argv[1], "manual") == 0)
    {
        mode = VISION_TRACK_MODE_MANUAL;
        mode_name = "MANUAL";
    }
    else if (rt_strcmp(argv[1], "continuous") == 0)
    {
        mode = VISION_TRACK_MODE_CONTINUOUS;
        mode_name = "CONTINUOUS";
    }
    else if (rt_strcmp(argv[1], "lock") == 0)
    {
        mode = VISION_TRACK_MODE_LOCK_ON;
        mode_name = "LOCK_ON";
    }
    else
    {
        LOG_D("❌ Invalid mode: %s\n", argv[1]);
        return;
    }

    rt_err_t result = vision_tracker_set_mode(&g_vision_tracker, mode);
    if (result == RT_EOK)
    {
        LOG_D("Vision tracking mode set to: %s\n", mode_name);
    }
    else
    {
        LOG_D("Failed to set vision tracking mode: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_mode, Set vision tracking mode: vision_mode <manual|continuous|lock>);

/**
 * @brief 输入目标坐标命令
 */
static void vision_target(int argc, char **argv)
{
    if (argc < 3 || argc > 4)
    {
        LOG_D("Usage: vision_target <pixel_x> <pixel_y> [confidence]\n");
        LOG_D("Example: vision_target 320 240 0.9\n");
        LOG_D("  pixel_x, pixel_y: Target pixel coordinates\n");
        LOG_D("  confidence: Target confidence (0.0-1.0, default: 0.8)\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float pixel_x = atof(argv[1]);
    float pixel_y = atof(argv[2]);
    float confidence = (argc == 4) ? atof(argv[3]) : 0.8f;

    rt_err_t result = vision_tracker_input_target(&g_vision_tracker, pixel_x, pixel_y, confidence);
    if (result == RT_EOK)
    {
        LOG_D("Target input: (%.1f, %.1f), confidence: %.2f\n", pixel_x, pixel_y, confidence);
    }
    else
    {
        LOG_D("Failed to input target: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_target, Input target pixel coordinates: vision_target <x> <y> [confidence]);

/**
 * @brief 标记目标丢失命令
 */
static void vision_lost(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    rt_err_t result = vision_tracker_target_lost(&g_vision_tracker);
    if (result == RT_EOK)
    {
        LOG_D("Target marked as lost\n");
    }
    else
    {
        LOG_D("Failed to mark target as lost: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_lost, Mark target as lost);

/**
 * @brief 设置PID参数命令
 */
static void vision_pid(int argc, char **argv)
{
    if (argc != 5)
    {
        LOG_D("Usage: vision_pid <axis> <kp> <ki> <kd>\n");
        LOG_D("Example: vision_pid yaw 0.8 0.02 0.15\n");
        LOG_D("  axis: yaw or pitch\n");
        LOG_D("  kp, ki, kd: PID coefficients\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float kp = atof(argv[2]);
    float ki = atof(argv[3]);
    float kd = atof(argv[4]);

    rt_err_t result;
    if (rt_strcmp(argv[1], "yaw") == 0)
    {
        result = vision_tracker_set_yaw_pid(&g_vision_tracker, kp, ki, kd);
    }
    else if (rt_strcmp(argv[1], "pitch") == 0)
    {
        result = vision_tracker_set_pitch_pid(&g_vision_tracker, kp, ki, kd);
    }
    else
    {
        LOG_D("❌ Invalid axis: %s (use 'yaw' or 'pitch')\n", argv[1]);
        return;
    }

    if (result == RT_EOK)
    {
        LOG_D("%s PID set: kp=%.3f, ki=%.3f, kd=%.3f\n", argv[1], kp, ki, kd);
    }
    else
    {
        LOG_D("Failed to set %s PID parameters: %d\n", argv[1], result);
    }
}
MSH_CMD_EXPORT(vision_pid, Set PID parameters: vision_pid <yaw|pitch> <kp> <ki> <kd>);

/**
 * @brief 设置PID限制命令
 */
static void vision_limits(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: vision_limits <max_output> <max_integral>\n");
        LOG_D("Example: vision_limits 45.0 10.0\n");
        LOG_D("  max_output: Maximum PID output (degrees/second)\n");
        LOG_D("  max_integral: Maximum integral value\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float max_output = atof(argv[1]);
    float max_integral = atof(argv[2]);

    rt_err_t result = vision_tracker_set_pid_limits(&g_vision_tracker, max_output, max_integral);
    if (result == RT_EOK)
    {
        LOG_D("PID limits set: max_output=%.1f°/s, max_integral=%.1f\n", max_output, max_integral);
    }
    else
    {
        LOG_D("Failed to set PID limits: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_limits, Set PID limits: vision_limits <max_output> <max_integral>);

/**
 * @brief 设置追踪参数命令
 */
static void vision_params(int argc, char **argv)
{
    if (argc != 4)
    {
        LOG_D("Usage: vision_params <deadzone> <max_error> <timeout>\n");
        LOG_D("Example: vision_params 5.0 50.0 5000\n");
        LOG_D("  deadzone: Deadzone in pixels\n");
        LOG_D("  max_error: Maximum error threshold in pixels\n");
        LOG_D("  timeout: Target lost timeout in milliseconds\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float deadzone = atof(argv[1]);
    float max_error = atof(argv[2]);
    uint32_t timeout = atoi(argv[3]);

    rt_err_t result = vision_tracker_set_track_params(&g_vision_tracker, deadzone, max_error, timeout);
    if (result == RT_EOK)
    {
        LOG_D("Tracking params set: deadzone=%.1fpx, max_error=%.1fpx, timeout=%dms\n",
               deadzone, max_error, timeout);
    }
    else
    {
        LOG_D("Failed to set tracking parameters: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_params, Set tracking parameters: vision_params <deadzone> <max_error> <timeout>);

/**
 * @brief 显示视觉追踪状态命令
 */
static void vision_status(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    vision_tracker_show_status(&g_vision_tracker);
    
    /* 显示更多详细信息 */
    vision_target_t current_target;
    if (vision_tracker_get_current_target(&g_vision_tracker, &current_target) == RT_EOK)
    {
        float yaw_error, pitch_error;
        if (vision_tracker_get_pid_errors(&g_vision_tracker, &yaw_error, &pitch_error) == RT_EOK)
        {
            LOG_D("\n📊 PID Output:\n");
            LOG_D("  Yaw error: %.2f°, output: %.2f°/s\n", yaw_error, g_vision_tracker.pid_yaw.output);
            LOG_D("  Pitch error: %.2f°, output: %.2f°/s\n", pitch_error, g_vision_tracker.pid_pitch.output);
        }
    }
}
MSH_CMD_EXPORT(vision_status, Show vision tracking status);

/**
 * @brief 视觉追踪测试命令
 */
static void vision_test(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("Usage: vision_test <demo|manual>\n");
        LOG_D("  demo   - Run automatic demonstration\n");
        LOG_D("  manual - Enter manual testing mode\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    if (rt_strcmp(argv[1], "demo") == 0)
    {
        vision_tracking_demo();
    }
    else if (rt_strcmp(argv[1], "manual") == 0)
    {
        LOG_D("🎮 Manual vision tracking test mode\n");
        LOG_D("Commands available:\n");
        LOG_D("  vision_target <x> <y> [conf] - Input target coordinates\n");
        LOG_D("  vision_lost                  - Mark target as lost\n");
        LOG_D("  vision_status                - Check tracking status\n");
        LOG_D("  vision_enable off            - Stop tracking\n");
        
        vision_tracker_set_mode(&g_vision_tracker, VISION_TRACK_MODE_CONTINUOUS);
        vision_tracker_enable(&g_vision_tracker, true);
        LOG_D("✅ Manual tracking mode enabled. Use 'vision_target' to input coordinates.\n");
    }
    else
    {
        LOG_D("❌ Unknown test type: %s\n", argv[1]);
    }
}
MSH_CMD_EXPORT(vision_test, Run vision tracking tests: vision_test <demo|manual>);

/**
 * @brief 视觉追踪帮助命令
 */
static void vision_help(int argc, char **argv)
{
    LOG_D("📷 === Vision Tracking Control Commands ===\n\n");
    
    LOG_D("🚀 Quick Start:\n");
    LOG_D("  1. laser_init                    # Initialize gimbal first\n");
    LOG_D("  2. vision_init                   # Initialize vision tracker\n");
    LOG_D("  3. vision_mode continuous        # Set tracking mode\n");
    LOG_D("  4. vision_enable on              # Enable tracking\n");
    LOG_D("  5. vision_target 320 240         # Input target coordinates\n\n");
    
    LOG_D("⚙️ Initialization & Configuration:\n");
    LOG_D("  vision_init [w h] [fov_h fov_v]  # Initialize with camera parameters\n");
    LOG_D("  vision_mode <manual|continuous|lock> # Set tracking mode\n");
    LOG_D("  vision_enable <on|off>           # Enable/disable tracking\n\n");
    
    LOG_D("🎯 Target Input:\n");
    LOG_D("  vision_target <x> <y> [conf]     # Input target pixel coordinates\n");
    LOG_D("  vision_lost                      # Mark target as lost\n\n");
    
    LOG_D("🔧 PID Configuration:\n");
    LOG_D("  vision_pid <yaw|pitch> <kp> <ki> <kd> # Set PID parameters\n");
    LOG_D("  vision_kp <yaw|pitch> <value>        # Set kp only\n");
    LOG_D("  vision_ki <yaw|pitch> <value>        # Set ki only\n");
    LOG_D("  vision_kd <yaw|pitch> <value>        # Set kd only\n");
    LOG_D("  vision_pid_show                      # Show current PID parameters\n");
    LOG_D("  vision_pid_preset <name>             # Apply preset parameters\n");
    LOG_D("  vision_pid_tune <command>            # Interactive tuning mode\n");
    LOG_D("  vision_limits <max_out> <max_int>    # Set PID output limits\n");
    LOG_D("  vision_params <dead> <err> <timeout> # Set tracking parameters\n\n");
    
    LOG_D("🎯 Target Offset Control:\n");
    LOG_D("  vision_offset <x> <y>            # Set target offset (pixels)\n");
    LOG_D("  vision_offset_enable <on|off>    # Enable/disable offset\n");
    LOG_D("  vision_offset_show               # Show current offset settings\n");
    LOG_D("  vision_offset_preset <name>      # Apply offset presets\n");
    LOG_D("  vision_offset_test               # Test offset functionality\n\n");
    
    LOG_D("🔄 Gyro Compensation Control:\n");
    LOG_D("  vision_gyro_enable <on|off>         # Enable/disable gyro compensation\n");
    LOG_D("  vision_gyro_params [y_gain p_gain thresh] # Set/show gyro parameters\n");
    LOG_D("  vision_gyro_preset <preset>         # Apply gyro compensation presets\n");
    LOG_D("  vision_gyro_test                    # Test gyro compensation\n\n");
    
    LOG_D("� Auto Search Control:\n");
    LOG_D("  vision_auto_search_params <params>  # Set search parameters\n");
    LOG_D("  vision_auto_search_enable <on|off>  # Enable/disable auto search\n");
    LOG_D("  vision_auto_search_show             # Show search settings\n");
    LOG_D("  vision_auto_search_start            # Manually start search\n");
    LOG_D("  vision_auto_search_stop             # Stop current search\n");
    LOG_D("  vision_auto_search_test             # Test auto search\n\n");
    
    LOG_D("�📊 Status & Testing:\n");
    LOG_D("  vision_status                    # Show detailed status\n");
    LOG_D("  vision_test demo                 # Run automatic demo\n");
    LOG_D("  vision_test manual               # Manual testing mode\n\n");
    
    LOG_D("📐 Coordinate System:\n");
    LOG_D("  • Origin (0,0) at top-left corner\n");
    LOG_D("  • X-axis: left to right (0 to camera_width)\n");
    LOG_D("  • Y-axis: top to bottom (0 to camera_height)\n");
    LOG_D("  • Center: (camera_width/2, camera_height/2)\n\n");
    
    LOG_D("🎮 Tracking Modes:\n");
    LOG_D("  • MANUAL: Single adjustment per target input\n");
    LOG_D("  • CONTINUOUS: Real-time continuous tracking\n");
    LOG_D("  • LOCK_ON: High-precision lock-on mode\n\n");
    
    LOG_D("💡 Usage Tips:\n");
    LOG_D("  • Start with default PID parameters, then fine-tune\n");
    LOG_D("  • Use higher kp for faster response, higher kd for stability\n");
    LOG_D("  • Adjust deadzone to avoid oscillation around target\n");
    LOG_D("  • Monitor vision_status for PID errors and outputs\n");
    LOG_D("  • Use target offset to aim ahead of moving targets\n");
    LOG_D("  • Positive X offset = aim right, Positive Y offset = aim down\n");
    LOG_D("  • Set confidence threshold for auto-search behavior\n");
    LOG_D("  • Auto-search locks pitch at 0° and rotates yaw to find targets\n\n");
    
    LOG_D("⚠️ Prerequisites:\n");
    LOG_D("  • Laser gimbal must be initialized first (laser_init)\n");
    LOG_D("  • Camera parameters should match your actual camera\n");
    LOG_D("  • FOV (Field of View) affects angle calculation accuracy\n");
    
    LOG_D("📋 Auto-Search Commands:\n");
    LOG_D("  vision_auto_search_config     - Configure auto-search parameters\n");
    LOG_D("  vision_auto_search_enable     - Enable/disable auto-search\n");
    LOG_D("  vision_auto_search_threshold  - Set confidence threshold\n");
    LOG_D("  vision_auto_search_speed      - Set search rotation speed\n");
    LOG_D("  vision_auto_search_cooldown   - Set search cooldown time\n");
    LOG_D("  vision_auto_search_status     - Show auto-search status\n");
    LOG_D("  vision_auto_search_test       - Test auto-search manually\n\n");
}
MSH_CMD_EXPORT(vision_help, Show vision tracking help and usage guide);

/* ======================== 高级PID调节命令 ======================== */

/**
 * @brief 单独设置kp参数
 */
static void vision_kp(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: vision_kp <axis> <kp_value>\n");
        LOG_D("Example: vision_kp yaw 1.2\n");
        LOG_D("  axis: yaw or pitch\n");
        LOG_D("  kp_value: proportional coefficient\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float kp = atof(argv[2]);
    
    if (rt_strcmp(argv[1], "yaw") == 0)
    {
        // 获取当前参数，只修改kp
        float current_ki = g_vision_tracker.pid_yaw.controller.parameter.ki;
        float current_kd = g_vision_tracker.pid_yaw.controller.parameter.kd;
        rt_err_t result = vision_tracker_set_yaw_pid(&g_vision_tracker, kp, current_ki, current_kd);
        if (result == RT_EOK)
        {
            LOG_D("✅ Yaw kp set to %.3f (ki=%.3f, kd=%.3f)\n", kp, current_ki, current_kd);
        }
        else
        {
            LOG_D("❌ Failed to set Yaw kp: %d\n", result);
        }
    }
    else if (rt_strcmp(argv[1], "pitch") == 0)
    {
        float current_ki = g_vision_tracker.pid_pitch.controller.parameter.ki;
        float current_kd = g_vision_tracker.pid_pitch.controller.parameter.kd;
        rt_err_t result = vision_tracker_set_pitch_pid(&g_vision_tracker, kp, current_ki, current_kd);
        if (result == RT_EOK)
        {
            LOG_D("✅ Pitch kp set to %.3f (ki=%.3f, kd=%.3f)\n", kp, current_ki, current_kd);
        }
        else
        {
            LOG_D("❌ Failed to set Pitch kp: %d\n", result);
        }
    }
    else
    {
        LOG_D("❌ Invalid axis: %s (use 'yaw' or 'pitch')\n", argv[1]);
    }
}
MSH_CMD_EXPORT(vision_kp, Set PID kp parameter: vision_kp <yaw|pitch> <kp_value>);

/**
 * @brief 单独设置ki参数
 */
static void vision_ki(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: vision_ki <axis> <ki_value>\n");
        LOG_D("Example: vision_ki yaw 0.05\n");
        LOG_D("  axis: yaw or pitch\n");
        LOG_D("  ki_value: integral coefficient\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float ki = atof(argv[2]);
    
    if (rt_strcmp(argv[1], "yaw") == 0)
    {
        float current_kp = g_vision_tracker.pid_yaw.controller.parameter.kp;
        float current_kd = g_vision_tracker.pid_yaw.controller.parameter.kd;
        rt_err_t result = vision_tracker_set_yaw_pid(&g_vision_tracker, current_kp, ki, current_kd);
        if (result == RT_EOK)
        {
            LOG_D("✅ Yaw ki set to %.3f (kp=%.3f, kd=%.3f)\n", ki, current_kp, current_kd);
        }
        else
        {
            LOG_D("❌ Failed to set Yaw ki: %d\n", result);
        }
    }
    else if (rt_strcmp(argv[1], "pitch") == 0)
    {
        float current_kp = g_vision_tracker.pid_pitch.controller.parameter.kp;
        float current_kd = g_vision_tracker.pid_pitch.controller.parameter.kd;
        rt_err_t result = vision_tracker_set_pitch_pid(&g_vision_tracker, current_kp, ki, current_kd);
        if (result == RT_EOK)
        {
            LOG_D("✅ Pitch ki set to %.3f (kp=%.3f, kd=%.3f)\n", ki, current_kp, current_kd);
        }
        else
        {
            LOG_D("❌ Failed to set Pitch ki: %d\n", result);
        }
    }
    else
    {
        LOG_D("❌ Invalid axis: %s (use 'yaw' or 'pitch')\n", argv[1]);
    }
}
MSH_CMD_EXPORT(vision_ki, Set PID ki parameter: vision_ki <yaw|pitch> <ki_value>);

/**
 * @brief 单独设置kd参数
 */
static void vision_kd(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: vision_kd <axis> <kd_value>\n");
        LOG_D("Example: vision_kd yaw 0.1\n");
        LOG_D("  axis: yaw or pitch\n");
        LOG_D("  kd_value: derivative coefficient\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float kd = atof(argv[2]);
    
    if (rt_strcmp(argv[1], "yaw") == 0)
    {
        float current_kp = g_vision_tracker.pid_yaw.controller.parameter.kp;
        float current_ki = g_vision_tracker.pid_yaw.controller.parameter.ki;
        rt_err_t result = vision_tracker_set_yaw_pid(&g_vision_tracker, current_kp, current_ki, kd);
        if (result == RT_EOK)
        {
            LOG_D("✅ Yaw kd set to %.3f (kp=%.3f, ki=%.3f)\n", kd, current_kp, current_ki);
        }
        else
        {
            LOG_D("❌ Failed to set Yaw kd: %d\n", result);
        }
    }
    else if (rt_strcmp(argv[1], "pitch") == 0)
    {
        float current_kp = g_vision_tracker.pid_pitch.controller.parameter.kp;
        float current_ki = g_vision_tracker.pid_pitch.controller.parameter.ki;
        rt_err_t result = vision_tracker_set_pitch_pid(&g_vision_tracker, current_kp, current_ki, kd);
        if (result == RT_EOK)
        {
            LOG_D("✅ Pitch kd set to %.3f (kp=%.3f, ki=%.3f)\n", kd, current_kp, current_ki);
        }
        else
        {
            LOG_D("❌ Failed to set Pitch kd: %d\n", result);
        }
    }
    else
    {
        LOG_D("❌ Invalid axis: %s (use 'yaw' or 'pitch')\n", argv[1]);
    }
}
MSH_CMD_EXPORT(vision_kd, Set PID kd parameter: vision_kd <yaw|pitch> <kd_value>);

/**
 * @brief 显示当前PID参数
 */
static void vision_pid_show(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    LOG_D("📊 Current PID Parameters:\n");
    LOG_D("┌─────────┬─────────┬─────────┬─────────┐\n");
    LOG_D("│  Axis   │   kp    │   ki    │   kd    │\n");
    LOG_D("├─────────┼─────────┼─────────┼─────────┤\n");
    LOG_D("│   Yaw   │ %7.3f │ %7.3f │ %7.3f │\n", 
          g_vision_tracker.pid_yaw.controller.parameter.kp,
          g_vision_tracker.pid_yaw.controller.parameter.ki,
          g_vision_tracker.pid_yaw.controller.parameter.kd);
    LOG_D("│  Pitch  │ %7.3f │ %7.3f │ %7.3f │\n", 
          g_vision_tracker.pid_pitch.controller.parameter.kp,
          g_vision_tracker.pid_pitch.controller.parameter.ki,
          g_vision_tracker.pid_pitch.controller.parameter.kd);
    LOG_D("└─────────┴─────────┴─────────┴─────────┘\n");
    
    LOG_D("🔧 PID Limits:\n");
    LOG_D("  • Max Output: %.1f°/s\n", g_vision_tracker.pid_yaw.controller.parameter.out_limit);
    LOG_D("  • Max Integral: %.1f\n", g_vision_tracker.pid_yaw.controller.parameter.integral_limit);
    
    LOG_D("📈 Current Status:\n");
    LOG_D("  • Yaw Error: %.2f, Output: %.2f\n", 
          g_vision_tracker.pid_yaw.error,
          g_vision_tracker.pid_yaw.output);
    LOG_D("  • Pitch Error: %.2f, Output: %.2f\n", 
          g_vision_tracker.pid_pitch.error,
          g_vision_tracker.pid_pitch.output);
}
MSH_CMD_EXPORT(vision_pid_show, Show current PID parameters and status);

/**
 * @brief PID预设参数
 */
static void vision_pid_preset(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: vision_pid_preset <preset_name>\n");
        LOG_D("Available presets:\n");
        LOG_D("  • default   - Default balanced parameters\n");
        LOG_D("  • fast      - Fast response (higher kp)\n");
        LOG_D("  • smooth    - Smooth response (lower kp, higher kd)\n");
        LOG_D("  • precise   - High precision (balanced with ki)\n");
        LOG_D("  • demo      - Demo/testing parameters\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    rt_err_t result_yaw = RT_ERROR, result_pitch = RT_ERROR;
    
    if (rt_strcmp(argv[1], "default") == 0)
    {
        result_yaw = vision_tracker_set_yaw_pid(&g_vision_tracker, 0.8f, 0.02f, 0.15f);
        result_pitch = vision_tracker_set_pitch_pid(&g_vision_tracker, 0.8f, 0.02f, 0.15f);
        LOG_D("✅ Applied 'default' PID preset\n");
    }
    else if (rt_strcmp(argv[1], "fast") == 0)
    {
        result_yaw = vision_tracker_set_yaw_pid(&g_vision_tracker, 1.5f, 0.01f, 0.1f);
        result_pitch = vision_tracker_set_pitch_pid(&g_vision_tracker, 1.5f, 0.01f, 0.1f);
        LOG_D("✅ Applied 'fast' PID preset - Quick response\n");
    }
    else if (rt_strcmp(argv[1], "smooth") == 0)
    {
        result_yaw = vision_tracker_set_yaw_pid(&g_vision_tracker, 0.5f, 0.005f, 0.25f);
        result_pitch = vision_tracker_set_pitch_pid(&g_vision_tracker, 0.5f, 0.005f, 0.25f);
        LOG_D("✅ Applied 'smooth' PID preset - Smooth movement\n");
    }
    else if (rt_strcmp(argv[1], "precise") == 0)
    {
        result_yaw = vision_tracker_set_yaw_pid(&g_vision_tracker, 1.0f, 0.05f, 0.2f);
        result_pitch = vision_tracker_set_pitch_pid(&g_vision_tracker, 1.0f, 0.05f, 0.2f);
        LOG_D("✅ Applied 'precise' PID preset - High precision\n");
    }
    else if (rt_strcmp(argv[1], "demo") == 0)
    {
        result_yaw = vision_tracker_set_yaw_pid(&g_vision_tracker, 0.6f, 0.01f, 0.1f);
        result_pitch = vision_tracker_set_pitch_pid(&g_vision_tracker, 0.6f, 0.01f, 0.1f);
        LOG_D("✅ Applied 'demo' PID preset - Safe for testing\n");
    }
    else
    {
        LOG_D("❌ Unknown preset: %s\n", argv[1]);
        LOG_D("Available: default, fast, smooth, precise, demo\n");
        return;
    }

    if (result_yaw == RT_EOK && result_pitch == RT_EOK)
    {
        LOG_D("🎯 Preset applied successfully. Use 'vision_pid_show' to view parameters.\n");
    }
    else
    {
        LOG_D("❌ Failed to apply preset. Yaw: %d, Pitch: %d\n", result_yaw, result_pitch);
    }
}
MSH_CMD_EXPORT(vision_pid_preset, Apply PID parameter presets: vision_pid_preset <default|fast|smooth|precise|demo>);

/**
 * @brief 实时PID调优模式
 */
static void vision_pid_tune(int argc, char **argv)
{
    if (argc < 2)
    {
        LOG_D("Usage: vision_pid_tune <command> [args...]\n");
        LOG_D("Commands:\n");
        LOG_D("  • start         - Start tuning mode\n");
        LOG_D("  • stop          - Stop tuning mode\n");
        LOG_D("  • step <axis> <param> <delta> - Adjust parameter by delta\n");
        LOG_D("  • reset         - Reset to default parameters\n");
        LOG_D("\nExample: vision_pid_tune step yaw kp 0.1\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    if (rt_strcmp(argv[1], "start") == 0)
    {
        LOG_D("🎛️ PID Tuning Mode Started\n");
        LOG_D("📝 Current parameters:\n");
        vision_pid_show(0, NULL);
        LOG_D("\n💡 Quick commands:\n");
        LOG_D("  • vision_kp yaw 1.0     - Set Yaw kp\n");
        LOG_D("  • vision_ki pitch 0.02  - Set Pitch ki\n");
        LOG_D("  • vision_kd yaw 0.15    - Set Yaw kd\n");
        LOG_D("  • vision_pid_show       - Show current values\n");
        LOG_D("  • vision_target 320 240 - Test with center target\n");
    }
    else if (rt_strcmp(argv[1], "stop") == 0)
    {
        LOG_D("🔚 PID Tuning Mode Stopped\n");
        LOG_D("📊 Final parameters:\n");
        vision_pid_show(0, NULL);
    }
    else if (rt_strcmp(argv[1], "step") == 0 && argc == 5)
    {
        char *axis = argv[2];
        char *param = argv[3];
        float delta = atof(argv[4]);
        
        float current_kp, current_ki, current_kd;
        bool is_yaw = (rt_strcmp(axis, "yaw") == 0);
        
        if (is_yaw)
        {
            current_kp = g_vision_tracker.pid_yaw.controller.parameter.kp;
            current_ki = g_vision_tracker.pid_yaw.controller.parameter.ki;
            current_kd = g_vision_tracker.pid_yaw.controller.parameter.kd;
        }
        else if (rt_strcmp(axis, "pitch") == 0)
        {
            current_kp = g_vision_tracker.pid_pitch.controller.parameter.kp;
            current_ki = g_vision_tracker.pid_pitch.controller.parameter.ki;
            current_kd = g_vision_tracker.pid_pitch.controller.parameter.kd;
        }
        else
        {
            LOG_D("❌ Invalid axis: %s\n", axis);
            return;
        }
        
        if (rt_strcmp(param, "kp") == 0)
        {
            float new_kp = current_kp + delta;
            if (new_kp < 0) new_kp = 0;
            if (is_yaw)
                vision_tracker_set_yaw_pid(&g_vision_tracker, new_kp, current_ki, current_kd);
            else
                vision_tracker_set_pitch_pid(&g_vision_tracker, new_kp, current_ki, current_kd);
            LOG_D("🔧 %s kp: %.3f → %.3f (Δ%.3f)\n", axis, current_kp, new_kp, delta);
        }
        else if (rt_strcmp(param, "ki") == 0)
        {
            float new_ki = current_ki + delta;
            if (new_ki < 0) new_ki = 0;
            if (is_yaw)
                vision_tracker_set_yaw_pid(&g_vision_tracker, current_kp, new_ki, current_kd);
            else
                vision_tracker_set_pitch_pid(&g_vision_tracker, current_kp, new_ki, current_kd);
            LOG_D("🔧 %s ki: %.3f → %.3f (Δ%.3f)\n", axis, current_ki, new_ki, delta);
        }
        else if (rt_strcmp(param, "kd") == 0)
        {
            float new_kd = current_kd + delta;
            if (new_kd < 0) new_kd = 0;
            if (is_yaw)
                vision_tracker_set_yaw_pid(&g_vision_tracker, current_kp, current_ki, new_kd);
            else
                vision_tracker_set_pitch_pid(&g_vision_tracker, current_kp, current_ki, new_kd);
            LOG_D("🔧 %s kd: %.3f → %.3f (Δ%.3f)\n", axis, current_kd, new_kd, delta);
        }
        else
        {
            LOG_D("❌ Invalid parameter: %s (use kp, ki, or kd)\n", param);
        }
    }
    else if (rt_strcmp(argv[1], "reset") == 0)
    {
        vision_tracker_set_yaw_pid(&g_vision_tracker, 0.8f, 0.02f, 0.15f);
        vision_tracker_set_pitch_pid(&g_vision_tracker, 0.8f, 0.02f, 0.15f);
        LOG_D("🔄 PID parameters reset to default values\n");
    }
    else
    {
        LOG_D("❌ Invalid command: %s\n", argv[1]);
    }
}
MSH_CMD_EXPORT(vision_pid_tune, Interactive PID tuning: vision_pid_tune <start|stop|step|reset> [args...]);

/* ======================== 目标偏移控制命令 ======================== */

/**
 * @brief 设置目标偏移量
 */
static void vision_offset(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_D("Usage: vision_offset <x_offset> <y_offset>\n");
        LOG_D("Example: vision_offset 10.0 -5.0\n");
        LOG_D("  x_offset: X轴偏移(像素，正值向右)\n");
        LOG_D("  y_offset: Y轴偏移(像素，正值向下)\n");
        LOG_D("Note: 使用 vision_offset_enable 启用偏移功能\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float offset_x = atof(argv[1]);
    float offset_y = atof(argv[2]);

    rt_err_t result = vision_tracker_set_offset(&g_vision_tracker, offset_x, offset_y);
    if (result == RT_EOK)
    {
        LOG_D("✅ Target offset set: X=%.1fpx, Y=%.1fpx\n", offset_x, offset_y);
        LOG_D("💡 Use 'vision_offset_enable on' to activate offset\n");
    }
    else
    {
        LOG_D("❌ Failed to set target offset: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_offset, Set target offset: vision_offset <x_offset> <y_offset>);

/**
 * @brief 启用/禁用目标偏移
 */
static void vision_offset_enable(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: vision_offset_enable <on|off>\n");
        LOG_D("Example: vision_offset_enable on\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    bool enable = (rt_strcmp(argv[1], "on") == 0);
    
    rt_err_t result = vision_tracker_enable_offset(&g_vision_tracker, enable);
    if (result == RT_EOK)
    {
        LOG_D("Target offset: %s\n", enable ? "ENABLED ✅" : "DISABLED ❌");
        
        if (enable)
        {
            float offset_x, offset_y;
            bool enabled;
            if (vision_tracker_get_offset(&g_vision_tracker, &offset_x, &offset_y, &enabled) == RT_EOK)
            {
                LOG_D("Current offset: X=%.1fpx, Y=%.1fpx\n", offset_x, offset_y);
            }
        }
    }
    else
    {
        LOG_D("❌ Failed to %s target offset: %d\n", enable ? "enable" : "disable", result);
    }
}
MSH_CMD_EXPORT(vision_offset_enable, Enable/disable target offset: vision_offset_enable <on|off>);

/**
 * @brief 显示当前偏移设置
 */
static void vision_offset_show(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float offset_x, offset_y;
    bool enabled;
    
    rt_err_t result = vision_tracker_get_offset(&g_vision_tracker, &offset_x, &offset_y, &enabled);
    if (result == RT_EOK)
    {
        LOG_D("📍 Target Offset Status:\n");
        LOG_D("┌──────────┬──────────┬──────────┐\n");
        LOG_D("│  Status  │  X Offset│  Y Offset│\n");
        LOG_D("├──────────┼──────────┼──────────┤\n");
        LOG_D("│ %8s │ %8.1f │ %8.1f │\n", 
              enabled ? "ENABLED" : "DISABLED", offset_x, offset_y);
        LOG_D("└──────────┴──────────┴──────────┘\n");
        
        if (enabled)
        {
            LOG_D("🎯 Cloud gimbal will aim at target + offset\n");
            LOG_D("   • Target at (320, 240) → Actual aim: (%.1f, %.1f)\n", 
                  320.0f + offset_x, 240.0f + offset_y);
        }
        else
        {
            LOG_D("⚪ Offset disabled - gimbal aims directly at target\n");
        }
    }
    else
    {
        LOG_D("❌ Failed to get offset settings: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_offset_show, Show current target offset settings);

/**
 * @brief 偏移预设命令
 */
static void vision_offset_preset(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: vision_offset_preset <preset_name>\n");
        LOG_D("Available presets:\n");
        LOG_D("  • center    - No offset (0, 0)\n");
        LOG_D("  • left      - Aim to left (-20, 0)\n");
        LOG_D("  • right     - Aim to right (20, 0)\n");
        LOG_D("  • up        - Aim upward (0, -15)\n");
        LOG_D("  • down      - Aim downward (0, 15)\n");
        LOG_D("  • topleft   - Top-left corner (-15, -10)\n");
        LOG_D("  • topright  - Top-right corner (15, -10)\n");
        LOG_D("  • bottomleft - Bottom-left corner (-15, 10)\n");
        LOG_D("  • bottomright- Bottom-right corner (15, 10)\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float offset_x = 0, offset_y = 0;
    const char* preset_name = argv[1];
    
    if (rt_strcmp(preset_name, "center") == 0)
    {
        offset_x = 0.0f; offset_y = 0.0f;
    }
    else if (rt_strcmp(preset_name, "left") == 0)
    {
        offset_x = -20.0f; offset_y = 0.0f;
    }
    else if (rt_strcmp(preset_name, "right") == 0)
    {
        offset_x = 20.0f; offset_y = 0.0f;
    }
    else if (rt_strcmp(preset_name, "up") == 0)
    {
        offset_x = 0.0f; offset_y = -15.0f;
    }
    else if (rt_strcmp(preset_name, "down") == 0)
    {
        offset_x = 0.0f; offset_y = 15.0f;
    }
    else if (rt_strcmp(preset_name, "topleft") == 0)
    {
        offset_x = -15.0f; offset_y = -10.0f;
    }
    else if (rt_strcmp(preset_name, "topright") == 0)
    {
        offset_x = 15.0f; offset_y = -10.0f;
    }
    else if (rt_strcmp(preset_name, "bottomleft") == 0)
    {
        offset_x = -15.0f; offset_y = 10.0f;
    }
    else if (rt_strcmp(preset_name, "bottomright") == 0)
    {
        offset_x = 15.0f; offset_y = 10.0f;
    }
    else
    {
        LOG_D("❌ Unknown preset: %s\n", preset_name);
        return;
    }

    rt_err_t result = vision_tracker_set_offset(&g_vision_tracker, offset_x, offset_y);
    if (result == RT_EOK)
    {
        vision_tracker_enable_offset(&g_vision_tracker, true);
        LOG_D("✅ Applied '%s' offset preset: (%.1f, %.1f)\n", preset_name, offset_x, offset_y);
        LOG_D("🎯 Offset automatically enabled\n");
    }
    else
    {
        LOG_D("❌ Failed to apply preset: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_offset_preset, Apply offset presets: vision_offset_preset <center|left|right|up|down|topleft|topright|bottomleft|bottomright>);

/**
 * @brief 偏移测试命令
 */
static void vision_offset_test(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    LOG_D("🧪 Testing Target Offset Functionality\n");
    
    /* 启用追踪 */
    vision_tracker_set_mode(&g_vision_tracker, VISION_TRACK_MODE_CONTINUOUS);
    vision_tracker_enable(&g_vision_tracker, true);
    
    /* 设置中心目标 */
    float center_x = 320.0f, center_y = 240.0f;
    LOG_D("1. Setting target at center: (%.0f, %.0f)\n", center_x, center_y);
    vision_tracker_input_target(&g_vision_tracker, center_x, center_y, 0.9f);
    rt_thread_mdelay(1000);
    
    /* 测试不同偏移 */
    float test_offsets[][2] = {
        {10.0f, 0.0f},    // 右偏移
        {-10.0f, 0.0f},   // 左偏移
        {0.0f, 10.0f},    // 下偏移
        {0.0f, -10.0f},   // 上偏移
        {0.0f, 0.0f}      // 回到中心
    };
    
    const char* test_names[] = {"Right", "Left", "Down", "Up", "Center"};
    
    for (int i = 0; i < 5; i++)
    {
        LOG_D("%d. Testing %s offset: (%.1f, %.1f)\n", 
              i+2, test_names[i], test_offsets[i][0], test_offsets[i][1]);
        
        vision_tracker_set_offset(&g_vision_tracker, test_offsets[i][0], test_offsets[i][1]);
        vision_tracker_enable_offset(&g_vision_tracker, true);
        
        /* 输入相同的目标，但会应用不同偏移 */
        vision_tracker_input_target(&g_vision_tracker, center_x, center_y, 0.9f);
        rt_thread_mdelay(800);
    }
    
    /* 禁用偏移 */
    vision_tracker_enable_offset(&g_vision_tracker, false);
    LOG_D("✅ Offset test completed. Offset disabled.\n");
    LOG_D("💡 Use 'vision_offset_show' to check final settings\n");
}
MSH_CMD_EXPORT(vision_offset_test, Test target offset functionality);

/* ======================== 自动搜索控制命令 ======================== */

/**
 * @brief 设置自动搜索参数
 */
static void vision_auto_search_params(int argc, char **argv)
{
    if (argc != 6)
    {
        LOG_D("Usage: vision_auto_search_params <threshold> <speed> <pitch> <max_time> <cooldown>\n");
        LOG_D("Example: vision_auto_search_params 0.5 30.0 0.0 12000 5000\n");
        LOG_D("  threshold: 可信度阈值(0.0-1.0)\n");
        LOG_D("  speed: yaw搜索速度(度/秒)\n");
        LOG_D("  pitch: 搜索时pitch锁定角度(度)\n");
        LOG_D("  max_time: 最大搜索时间(毫秒)\n");
        LOG_D("  cooldown: 搜索后冷却时间(毫秒)\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float threshold = atof(argv[1]);
    float speed = atof(argv[2]);
    float pitch = atof(argv[3]);
    uint32_t max_time = atoi(argv[4]);
    uint32_t cooldown = atoi(argv[5]);

    rt_err_t result = vision_tracker_set_auto_search_params(&g_vision_tracker, threshold, speed, pitch, max_time, cooldown);
    if (result == RT_EOK)
    {
        LOG_D("✅ Auto search params set: threshold=%.2f, speed=%.1f°/s, pitch=%.1f°, max_time=%dms, cooldown=%dms\n",
              threshold, speed, pitch, max_time, cooldown);
    }
    else
    {
        LOG_D("❌ Failed to set auto search parameters: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_auto_search_params, Set auto search parameters: vision_auto_search_params <threshold> <speed> <pitch> <max_time> <cooldown>);

/**
 * @brief 启用/禁用自动搜索
 */
static void vision_auto_search_enable(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: vision_auto_search_enable <on|off>\n");
        LOG_D("Example: vision_auto_search_enable on\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    bool enable = (rt_strcmp(argv[1], "on") == 0);
    
    rt_err_t result = vision_tracker_enable_auto_search(&g_vision_tracker, enable);
    if (result == RT_EOK)
    {
        LOG_D("Auto search: %s\n", enable ? "ENABLED ✅" : "DISABLED ❌");
    }
    else
    {
        LOG_D("❌ Failed to %s auto search: %d\n", enable ? "enable" : "disable", result);
    }
}
MSH_CMD_EXPORT(vision_auto_search_enable, Enable/disable auto search: vision_auto_search_enable <on|off>);

/**
 * @brief 设置自动搜索冷却时间
 */
static void vision_auto_search_cooldown(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_D("Usage: vision_auto_search_cooldown <time_ms>\n");
        LOG_D("Example: vision_auto_search_cooldown 5000\n");
        LOG_D("  time_ms: 冷却时间(毫秒)\n");
        return;
    }

    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    uint32_t cooldown = atoi(argv[1]);
    
    /* 获取当前参数 */
    float threshold, speed, pitch;
    uint32_t old_cooldown;
    bool enabled;
    
    rt_err_t result = vision_tracker_get_auto_search_params(&g_vision_tracker, &threshold, &speed, &pitch, &old_cooldown, &enabled);
    if (result == RT_EOK)
    {
        /* 更新参数 */
        result = vision_tracker_set_auto_search_params(&g_vision_tracker, threshold, speed, pitch, g_vision_tracker.max_search_time, cooldown);
        if (result == RT_EOK)
        {
            LOG_D("✅ Auto search cooldown updated: %dms → %dms\n", old_cooldown, cooldown);
        }
        else
        {
            LOG_D("❌ Failed to set cooldown: %d\n", result);
        }
    }
    else
    {
        LOG_D("❌ Failed to get current parameters: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_auto_search_cooldown, Set auto search cooldown time: vision_auto_search_cooldown <time_ms>);

/**
 * @brief 显示自动搜索设置
 */
static void vision_auto_search_show(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    float threshold, speed, pitch;
    uint32_t cooldown;
    bool enabled;
    
    rt_err_t result = vision_tracker_get_auto_search_params(&g_vision_tracker, &threshold, &speed, &pitch, &cooldown, &enabled);
    if (result == RT_EOK)
    {
        LOG_D("🔍 Auto Search Settings:\n");
        LOG_D("┌──────────────┬─────────────┬─────────────┬─────────────┬─────────────┐\n");
        LOG_D("│    Status    │  Threshold  │   Speed     │    Pitch    │  Cooldown   │\n");
        LOG_D("├──────────────┼─────────────┼─────────────┼─────────────┼─────────────┤\n");
        LOG_D("│ %12s │ %11.2f │ %9.1f°/s │ %9.1f°   │ %9dms │\n", 
              enabled ? "ENABLED" : "DISABLED", threshold, speed, pitch, cooldown);
        LOG_D("└──────────────┴─────────────┴─────────────┴─────────────┴─────────────┘\n");
        
        LOG_D("📋 Max Search Time: %d ms (%.1f seconds)\n", 
              g_vision_tracker.max_search_time, 
              g_vision_tracker.max_search_time / 1000.0f);
        
        if (enabled)
        {
            LOG_D("💡 When target confidence < %.2f, gimbal will:\n", threshold);
            LOG_D("   • Lock pitch to %.1f°\n", pitch);
            LOG_D("   • Rotate yaw at %.1f°/s searching for target\n", speed);
            LOG_D("   • Stop when confidence ≥ %.2f or timeout\n", threshold);
            LOG_D("   • Wait %dms cooldown before next auto-search\n", cooldown);
            
            /* 显示当前冷却状态 */
            if (g_vision_tracker.last_search_end_time > 0)
            {
                uint32_t current_time = rt_tick_get_millisecond();
                uint32_t time_since_last = current_time - g_vision_tracker.last_search_end_time;
                if (time_since_last < cooldown)
                {
                    uint32_t remaining = cooldown - time_since_last;
                    LOG_D("⏰ Current Status: COOLDOWN (remaining: %dms)\n", remaining);
                }
                else
                {
                    LOG_D("✅ Current Status: READY\n");
                }
            }
            else
            {
                LOG_D("✅ Current Status: READY (never searched)\n");
            }
        }
    }
    else
    {
        LOG_D("❌ Failed to get auto search settings: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_auto_search_show, Show auto search settings);

/**
 * @brief 手动触发自动搜索
 */
static void vision_auto_search_start(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    rt_err_t result = vision_tracker_start_auto_search(&g_vision_tracker);
    if (result == RT_EOK)
    {
        LOG_D("🔍 Auto search started manually\n");
        LOG_D("💡 Use 'vision_status' to monitor search progress\n");
    }
    else
    {
        LOG_D("❌ Failed to start auto search: %d\n", result);
        LOG_D("   • Make sure vision tracking is enabled\n");
        LOG_D("   • Make sure auto search is enabled\n");
    }
}
MSH_CMD_EXPORT(vision_auto_search_start, Manually start auto search);

/**
 * @brief 停止自动搜索
 */
static void vision_auto_search_stop(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    rt_err_t result = vision_tracker_stop_auto_search(&g_vision_tracker);
    if (result == RT_EOK)
    {
        LOG_D("⏹️ Auto search stopped\n");
    }
    else
    {
        LOG_D("❌ Failed to stop auto search: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_auto_search_stop, Stop auto search);

/**
 * @brief 自动搜索测试
 */
static void vision_auto_search_test(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    LOG_D("🧪 Testing Auto Search Functionality\n");
    
    /* 启用追踪 */
    vision_tracker_set_mode(&g_vision_tracker, VISION_TRACK_MODE_CONTINUOUS);
    vision_tracker_enable(&g_vision_tracker, true);
    vision_tracker_enable_auto_search(&g_vision_tracker, true);
    
    /* 设置测试参数 */
    vision_tracker_set_auto_search_params(&g_vision_tracker, 0.5f, 45.0f, 0.0f, 8000, 5000);
    
    LOG_D("1. 输入高可信度目标 (应正常追踪)\n");
    vision_tracker_input_target(&g_vision_tracker, 320, 240, 0.9f);
    rt_thread_mdelay(2000);
    
    LOG_D("2. 输入低可信度目标 (应触发自动搜索)\n");
    vision_tracker_input_target(&g_vision_tracker, 320, 240, 0.3f);
    rt_thread_mdelay(3000);
    
    LOG_D("3. 在搜索中输入高可信度目标 (应停止搜索)\n");
    vision_tracker_input_target(&g_vision_tracker, 400, 200, 0.8f);
    rt_thread_mdelay(2000);
    
    LOG_D("4. 再次输入低可信度目标测试\n");
    vision_tracker_input_target(&g_vision_tracker, 100, 100, 0.2f);
    rt_thread_mdelay(2000);
    
    /* 手动停止搜索 */
    vision_tracker_stop_auto_search(&g_vision_tracker);
    
    LOG_D("✅ Auto search test completed\n");
    LOG_D("💡 Use 'vision_status' to check final state\n");
}
MSH_CMD_EXPORT(vision_auto_search_test, Test auto search functionality);

/* ======================== 陀螺仪补偿测试命令 ======================== */

/**
 * @brief 启用/禁用视觉追踪器的陀螺仪补偿
 */
static void vision_gyro_enable(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    if (argc != 2)
    {
        LOG_D("📖 Usage: vision_gyro_enable <on|off>\n");
        LOG_D("   Enable or disable gyroscope compensation for vision tracking\n");
        return;
    }

    bool enable;
    if (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0)
    {
        enable = true;
    }
    else if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "0") == 0)
    {
        enable = false;
    }
    else
    {
        LOG_D("❌ Invalid parameter: %s (use 'on' or 'off')\n", argv[1]);
        return;
    }

    rt_err_t result = vision_tracker_enable_gyro_compensation(&g_vision_tracker, enable);
    if (result == RT_EOK)
    {
        LOG_D("✅ Vision gyro compensation %s\n", enable ? "enabled" : "disabled");
    }
    else
    {
        LOG_D("❌ Failed to %s vision gyro compensation: %d\n", 
              enable ? "enable" : "disable", result);
    }
}
MSH_CMD_EXPORT(vision_gyro_enable, Enable/disable vision gyro compensation: vision_gyro_enable <on|off>);

/**
 * @brief 设置视觉追踪器的陀螺仪补偿参数
 */
static void vision_gyro_params(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    if (argc == 1)
    {
        /* 显示当前参数 */
        bool enabled;
        float yaw_gain, pitch_gain, threshold;
        
        rt_err_t result = vision_tracker_get_gyro_compensation_params(&g_vision_tracker,
                                                                     &enabled, &yaw_gain, 
                                                                     &pitch_gain, &threshold);
        if (result == RT_EOK)
        {
            LOG_D("📊 Current Vision Gyro Compensation Parameters:\n");
            LOG_D("   Status: %s\n", enabled ? "Enabled" : "Disabled");
            LOG_D("   Yaw Gain: %.3f\n", yaw_gain);
            LOG_D("   Pitch Gain: %.3f\n", pitch_gain);
            LOG_D("   Motion Threshold: %.1f°/s\n", threshold);
        }
        else
        {
            LOG_D("❌ Failed to get gyro compensation parameters: %d\n", result);
        }
        return;
    }

    if (argc != 4)
    {
        LOG_D("📖 Usage: vision_gyro_params <yaw_gain> <pitch_gain> <threshold>\n");
        LOG_D("   yaw_gain: Yaw axis compensation gain (0.0-2.0)\n");
        LOG_D("   pitch_gain: Pitch axis compensation gain (0.0-2.0)\n");
        LOG_D("   threshold: Motion threshold in deg/s (0.0-100.0)\n");
        LOG_D("   Use 'vision_gyro_params' to show current values\n");
        return;
    }

    float yaw_gain = atof(argv[1]);
    float pitch_gain = atof(argv[2]);
    float threshold = atof(argv[3]);

    rt_err_t result = vision_tracker_set_gyro_compensation_params(&g_vision_tracker,
                                                                 yaw_gain, pitch_gain, threshold);
    if (result == RT_EOK)
    {
        LOG_D("✅ Vision gyro compensation parameters set:\n");
        LOG_D("   Yaw Gain: %.3f\n", yaw_gain);
        LOG_D("   Pitch Gain: %.3f\n", pitch_gain);
        LOG_D("   Motion Threshold: %.1f°/s\n", threshold);
    }
    else
    {
        LOG_D("❌ Failed to set gyro compensation parameters: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_gyro_params, Set/show vision gyro compensation parameters: vision_gyro_params [yaw_gain pitch_gain threshold]);

/**
 * @brief 预设陀螺仪补偿参数
 */
static void vision_gyro_preset(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    if (argc != 2)
    {
        LOG_D("📖 Usage: vision_gyro_preset <preset_name>\n");
        LOG_D("   Available presets:\n");
        LOG_D("   - default: Standard compensation (0.3, 0.3, 2.0)\n");
        LOG_D("   - aggressive: Strong compensation (0.8, 0.8, 1.0)\n");
        LOG_D("   - gentle: Mild compensation (0.1, 0.1, 5.0)\n");
        LOG_D("   - yaw_only: Only yaw compensation (0.5, 0.0, 2.0)\n");
        LOG_D("   - pitch_only: Only pitch compensation (0.0, 0.5, 2.0)\n");
        LOG_D("   - off: Disable compensation (0.0, 0.0, 100.0)\n");
        return;
    }

    float yaw_gain, pitch_gain, threshold;

    if (strcmp(argv[1], "default") == 0)
    {
        yaw_gain = 0.3f; pitch_gain = 0.3f; threshold = 2.0f;
    }
    else if (strcmp(argv[1], "aggressive") == 0)
    {
        yaw_gain = 0.8f; pitch_gain = 0.8f; threshold = 1.0f;
    }
    else if (strcmp(argv[1], "gentle") == 0)
    {
        yaw_gain = 0.1f; pitch_gain = 0.1f; threshold = 5.0f;
    }
    else if (strcmp(argv[1], "yaw_only") == 0)
    {
        yaw_gain = 0.5f; pitch_gain = 0.0f; threshold = 2.0f;
    }
    else if (strcmp(argv[1], "pitch_only") == 0)
    {
        yaw_gain = 0.0f; pitch_gain = 0.5f; threshold = 2.0f;
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        yaw_gain = 0.0f; pitch_gain = 0.0f; threshold = 100.0f;
    }
    else
    {
        LOG_D("❌ Unknown preset: %s\n", argv[1]);
        LOG_D("💡 Use 'vision_gyro_preset' to see available presets\n");
        return;
    }

    rt_err_t result = vision_tracker_set_gyro_compensation_params(&g_vision_tracker,
                                                                 yaw_gain, pitch_gain, threshold);
    if (result == RT_EOK)
    {
        LOG_D("✅ Applied '%s' preset:\n", argv[1]);
        LOG_D("   Yaw Gain: %.3f\n", yaw_gain);
        LOG_D("   Pitch Gain: %.3f\n", pitch_gain);
        LOG_D("   Motion Threshold: %.1f°/s\n", threshold);
    }
    else
    {
        LOG_D("❌ Failed to apply preset: %d\n", result);
    }
}
MSH_CMD_EXPORT(vision_gyro_preset, Apply vision gyro compensation presets: vision_gyro_preset <default|aggressive|gentle|yaw_only|pitch_only|off>);

/**
 * @brief 测试视觉追踪器的陀螺仪补偿
 */
static void vision_gyro_test(int argc, char **argv)
{
    if (!g_vision_initialized)
    {
        LOG_D("❌ Vision tracker not initialized! Run 'vision_init' first.\n");
        return;
    }

    LOG_D("🧪 Testing Vision Gyro Compensation\n");
    LOG_D("════════════════════════════════════\n");

    /* 启用视觉追踪 */
    vision_tracker_set_mode(&g_vision_tracker, VISION_TRACK_MODE_CONTINUOUS);
    vision_tracker_enable(&g_vision_tracker, true);

    /* 测试1: 默认参数 */
    LOG_D("1️⃣ Testing with default parameters...\n");
    vision_tracker_enable_gyro_compensation(&g_vision_tracker, true);
    vision_tracker_set_gyro_compensation_params(&g_vision_tracker, 0.3f, 0.3f, 2.0f);
    
    /* 输入目标并观察一段时间 */
    vision_tracker_input_target(&g_vision_tracker, 320, 240, 0.8f);
    LOG_D("   Target input, observe compensation for 3 seconds...\n");
    rt_thread_mdelay(3000);

    /* 测试2: 强补偿 */
    LOG_D("2️⃣ Testing with aggressive compensation...\n");
    vision_tracker_set_gyro_compensation_params(&g_vision_tracker, 0.8f, 0.8f, 1.0f);
    vision_tracker_input_target(&g_vision_tracker, 400, 200, 0.8f);
    LOG_D("   Aggressive mode, observe for 3 seconds...\n");
    rt_thread_mdelay(3000);

    /* 测试3: 禁用补偿 */
    LOG_D("3️⃣ Testing without compensation...\n");
    vision_tracker_enable_gyro_compensation(&g_vision_tracker, false);
    vision_tracker_input_target(&g_vision_tracker, 200, 300, 0.8f);
    LOG_D("   Compensation disabled, observe for 3 seconds...\n");
    rt_thread_mdelay(3000);

    /* 恢复默认设置 */
    vision_tracker_enable_gyro_compensation(&g_vision_tracker, true);
    vision_tracker_set_gyro_compensation_params(&g_vision_tracker, 0.3f, 0.3f, 2.0f);

    LOG_D("✅ Vision gyro compensation test completed\n");
    LOG_D("💡 Use 'vision_gyro_params' to check current settings\n");
    LOG_D("💡 Move the gimbal during test to observe compensation effects\n");
}
MSH_CMD_EXPORT(vision_gyro_test, Test vision gyro compensation functionality);
