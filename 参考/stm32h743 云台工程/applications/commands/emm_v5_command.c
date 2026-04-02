#include "emm_v5_command.h"

#include <rtthread.h>
#include <rtdevice.h>
#include "ulog.h"
#include "drv_emm_v5.h"

#ifdef RT_USING_FINSH

static void emm_v5_commands(int argc, char **argv)
{
    if (argc < 2)	//仅输入注册命令的情况，argc[0]=emm_v5_commands
    {
        LOG_D("EMM_V5_COMMANDS:");
        LOG_D("  emm_v5 enable        - Enable Motor");
			  LOG_D("  emm_v5 disable       - Disable Motor");
        LOG_D("  emm_v5 init         - Initialize motor only");
        LOG_D("  emm_v5 deinit       - Deinitialize motor");
        LOG_D("  emm_v5 stop         - Stop motor");
        LOG_D("  emm_v5 speed <val>  - Speed control (dps)");
        LOG_D("  emm_v5 torque <val> - Torque control");
        LOG_D("  emm_v5 pos <angle> [speed] - Position control");
        LOG_D("  emm_v5 pos2 <angle> <speed> - Position control2");
        LOG_D("  emm_v5 single1 <dir> <angle> - Single pos control1");
        LOG_D("  emm_v5 single2 <dir> <angle> [speed] - Single pos control2");
        LOG_D("  emm_v5 inc1 <angle> - Increment control1");
        LOG_D("  emm_v5 inc2 <angle> [speed] - Increment control2");
        LOG_D("  emm_v5 status1      - Read motor status1");
        LOG_D("  emm_v5 status2      - Read motor status2");
        LOG_D("  emm_v5 status3      - Read motor status3");
        LOG_D("  emm_v5 stats        - Show communication statistics");
        LOG_D("  emm_v5 motor_on     - Turn motor on");
        LOG_D("  emm_v5 motor_off    - Turn motor off");
        LOG_D("  emm_v5 motor_stop   - Stop motor immediately");
        LOG_D("  emm_v5 brake <cmd>  - Brake control (0=lock,1=release,16=read)");
        LOG_D("  emm_v5 clear_err    - Clear error flags");
        return;
    }

    if (rt_strcmp(argv[1], "enable") == 0)
    {
        //ms4010_simple_test_start();
			if(argc==2)
			{
					Emm_V5_En_Control(&stepper_motor_1,1,0);
					Emm_V5_En_Control(&stepper_motor_2,1,0);
					Emm_V5_En_Control(&stepper_motor_3,1,0);
					Emm_V5_En_Control(&stepper_motor_4,1,0);
			}
			else
			{
					if(rt_strcmp(argv[2], "1")==0)
					{
						Emm_V5_En_Control(&stepper_motor_1,1,0);
					}
					else if(rt_strcmp(argv[2], "2")==0)
					{
						Emm_V5_En_Control(&stepper_motor_2,1,0);
					}
					else if(rt_strcmp(argv[2], "3")==0)
					{
						Emm_V5_En_Control(&stepper_motor_3,1,0);
					}
					else if(rt_strcmp(argv[2], "4")==0)
					{
						Emm_V5_En_Control(&stepper_motor_4,1,0);
					}
					LOG_D("Emm_V5_Enable successful");
			}
    }
		 if (rt_strcmp(argv[1], "disable") == 0)
    {
        //ms4010_simple_test_start();
			if(argc==2)
			{
					Emm_V5_En_Control(&stepper_motor_1,0,0);
					Emm_V5_En_Control(&stepper_motor_2,0,0);
					Emm_V5_En_Control(&stepper_motor_3,0,0);
					Emm_V5_En_Control(&stepper_motor_4,0,0);
			}
			else
			{
					if(rt_strcmp(argv[2], "1")==0)
					{
						Emm_V5_En_Control(&stepper_motor_1,0,0);
					}
					else if(rt_strcmp(argv[2], "2")==0)
					{
						Emm_V5_En_Control(&stepper_motor_2,0,0);
					}
					else if(rt_strcmp(argv[2], "3")==0)
					{
						Emm_V5_En_Control(&stepper_motor_3,0,0);
					}
					else if(rt_strcmp(argv[2], "4")==0)
					{
						Emm_V5_En_Control(&stepper_motor_4,0,0);
					}
					LOG_D("Emm_V5_Disable successful");
			}
    }
    else if (rt_strcmp(argv[1], "init") == 0)
    {

			LOG_D("init undefinition");
    }
    else if (rt_strcmp(argv[1], "deinit") == 0)
    {
//        rt_err_t result = ms4010_deinit(&simple_motor);
//        LOG_D("MS4010 deinit: %s", result == RT_EOK ? "SUCCESS" : "FAILED");
			LOG_D("deinit undefinition");
    }

    else if (rt_strcmp(argv[1], "stop") == 0)
    {
//        rt_err_t result = ms4010_motor_stop(&simple_motor);
//        LOG_D("Stop motor: %s", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "speed") == 0)
    {
				if(argc==2)
				{
					LOG_D("id:1,emmv5_speed:target:%d,real:%d",stepper_motor_1.stepper_motor_target_speed,stepper_motor_1.stepper_motor_speed);
					LOG_D("id:2,emmv5_speed:target:%d,real:%d",stepper_motor_2.stepper_motor_target_speed,stepper_motor_2.stepper_motor_speed);
					LOG_D("id:3,emmv5_speed:target:%d,real:%d",stepper_motor_3.stepper_motor_target_speed,stepper_motor_3.stepper_motor_speed);
					LOG_D("id:4,emmv5_speed:target:%d,real:%d",stepper_motor_4.stepper_motor_target_speed,stepper_motor_4.stepper_motor_speed);
				}
				else 
				{
					if(rt_strcmp(argv[2],"1")==0)
					{
							LOG_D("id:1,emmv5_speed:target:%d,real:%d",stepper_motor_1.stepper_motor_target_speed,stepper_motor_1.stepper_motor_speed);
					}
					else if(rt_strcmp(argv[2],"2")==0)
					{
							LOG_D("id:2,emmv5_speed:target:%d,real:%d",stepper_motor_2.stepper_motor_target_speed,stepper_motor_2.stepper_motor_speed);
					}
					else if(rt_strcmp(argv[2],"3")==0)
					{
							LOG_D("id:3,emmv5_speed:target:%d,real:%d",stepper_motor_3.stepper_motor_target_speed,stepper_motor_3.stepper_motor_speed);
					}
					else if(rt_strcmp(argv[2],"4")==0)
					{
							LOG_D("id:4,emmv5_speed:target:%d,real:%d",stepper_motor_4.stepper_motor_target_speed,stepper_motor_4.stepper_motor_speed);
					}
				}
//        rt_int32_t speed = atoi(argv[2]);
//        rt_err_t result = ms4010_speed_control(&simple_motor, speed);
//        LOG_D("Speed control %d dps: %s", speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }

    else if (rt_strcmp(argv[1], "pos") == 0)
    {
        if (argc==2)
        {
					LOG_D("id:1,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_1.stepper_motor_target_angle,stepper_motor_1.stepper_motor_angle);
					LOG_D("id:2,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_2.stepper_motor_target_angle,stepper_motor_2.stepper_motor_angle);
					LOG_D("id:3,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_3.stepper_motor_target_angle,stepper_motor_3.stepper_motor_angle);
					LOG_D("id:4,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_4.stepper_motor_target_angle,stepper_motor_4.stepper_motor_angle);
        }
				else
				{
						if(rt_strcmp(argv[2],"1")==0)
						{
								LOG_D("id:1,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_1.stepper_motor_target_angle,stepper_motor_1.stepper_motor_angle);
						}
						else if(rt_strcmp(argv[2],"2")==0)
						{
								LOG_D("id:2,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_2.stepper_motor_target_angle,stepper_motor_2.stepper_motor_angle);
						}
						else if(rt_strcmp(argv[2],"3")==0)
						{
								LOG_D("id:3,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_3.stepper_motor_target_angle,stepper_motor_3.stepper_motor_angle);
						}
						else if(rt_strcmp(argv[2],"4")==0)
						{
								LOG_D("id:4,emmv5_pos:target:%.3f,real:%.3f",stepper_motor_4.stepper_motor_target_angle,stepper_motor_4.stepper_motor_angle);
						}
				}

    }

    else if (rt_strcmp(argv[1], "single1") == 0)
    {
        if (argc < 4)
        {
            LOG_D("Usage: ms4010_simple single1 <dir> <angle>");
            LOG_D("  dir: 0=clockwise, 1=counter-clockwise");
            LOG_D("  angle: target angle in 0.01 degree units (0-35999)");
            return;
        }
//        rt_uint8_t direction = atoi(argv[2]);
//        rt_uint16_t angle = atoi(argv[3]);
//        rt_err_t result = ms4010_single_position_control1(&simple_motor, direction, angle);
//        LOG_D("Single position control1 dir=%d, angle=%d: %s", direction, angle, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "single2") == 0)
    {
        if (argc < 4)
        {
            LOG_D("Usage: ms4010_simple single2 <dir> <angle> [max_speed]");
            LOG_D("  dir: 0=clockwise, 1=counter-clockwise");
            LOG_D("  angle: target angle in 0.01 degree units (0-35999)");
            LOG_D("  max_speed: optional max speed (default 18000)");
            return;
        }
//        rt_uint8_t direction = atoi(argv[2]);
//        rt_uint16_t angle = atoi(argv[3]);
//        rt_uint32_t max_speed = (argc >= 5) ? atoi(argv[4]) : 18000;
//        rt_err_t result = ms4010_single_position_control2(&simple_motor, direction, angle, max_speed);
//        LOG_D("Single position control2 dir=%d, angle=%d @ max_speed %u: %s", direction, angle, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "inc1") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple inc1 <angle>");
            LOG_D("  angle: angle increment in 0.01 degree units");
            return;
        }
//        rt_int32_t angle_increment = atoi(argv[2]);
//        rt_err_t result = ms4010_increment_position_control1(&simple_motor, angle_increment);
//        LOG_D("Increment position control1 %d (0.01°): %s", angle_increment, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "inc2") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple inc2 <angle> [max_speed]");
            LOG_D("  angle: angle increment in 0.01 degree units");
            LOG_D("  max_speed: optional max speed (default 18000)");
            return;
        }
//        rt_int32_t angle_increment = atoi(argv[2]);
//        rt_uint32_t max_speed = (argc >= 4) ? atoi(argv[3]) : 18000;
//        rt_err_t result = ms4010_increment_position_control2(&simple_motor, angle_increment, max_speed);
//        LOG_D("Increment position control2 %d (0.01°) @ max_speed %u: %s", angle_increment, max_speed, result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "status1") == 0)
    {
//        ms4010_status1_t status1;
//        rt_err_t result = ms4010_read_status1(&simple_motor, &status1);
//        if (result == RT_EOK)
//        {
//            LOG_D("Motor Status1:");
//            LOG_D("  Temperature: %d°C", status1.temperature);
//            LOG_D("  Voltage: %.2fV", status1.voltage * 0.01f);
//            LOG_D("  Current: %.2fA", status1.current * 0.01f);
//            LOG_D("  Motor State: 0x%02X", status1.motor_state);
//            LOG_D("  Error State: 0x%02X", status1.error_state);
//        }
//        else
//        {
//            LOG_D("Failed to read status1: %d", result);
//        }
    }
    else if (rt_strcmp(argv[1], "status2") == 0)
    {
//        ms4010_status_t status2;
//        rt_err_t result = ms4010_get_status(&simple_motor, &status2);
//        if (result == RT_EOK)
//        {
//            LOG_D("Motor Status2:");
//            LOG_D("  Temperature: %d°C", status2.temperature);
//            LOG_D("  Power/Torque: %d", status2.power_or_torque);
//            LOG_D("  Speed: %d dps", status2.speed);
//            LOG_D("  Encoder: %u", status2.encoder);
//        }
//        else
//        {
//            LOG_D("Failed to read status2: %d", result);
//        }
    }
    else if (rt_strcmp(argv[1], "status3") == 0)
    {
//        ms4010_status3_t status3;
//        rt_err_t result = ms4010_read_status3(&simple_motor, &status3);
//        if (result == RT_EOK)
//        {
//            LOG_D("Motor Status3:");
//            LOG_D("  Temperature: %d°C", status3.temperature);
//            LOG_D("  Current A: %d", status3.current_a);
//            LOG_D("  Current B: %d", status3.current_b);
//            LOG_D("  Current C: %d", status3.current_c);
//        }
//        else
//        {
//            LOG_D("Failed to read status3: %d", result);
//        }
    }
    else if (rt_strcmp(argv[1], "stats") == 0)
    {
//        rt_uint32_t send_count, error_count;
//        ms4010_get_statistics(&simple_motor, &send_count, &error_count);
//        LOG_D("Communication Statistics:");
//        LOG_D("  Total Send: %u", send_count);
//        LOG_D("  Total Error: %u", error_count);
//        LOG_D("  Success Rate: %.1f%%", 
//                   send_count > 0 ? ((float)(send_count - error_count) / send_count * 100) : 0);
    }
    else if (rt_strcmp(argv[1], "motor_on") == 0)
    {
//        rt_err_t result = ms4010_motor_on(&simple_motor);
//        LOG_D("Motor on: %s", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "motor_off") == 0)
    {
//        rt_err_t result = ms4010_motor_off(&simple_motor);
//        LOG_D("Motor off: %s", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "motor_stop") == 0)
    {
//        rt_err_t result = ms4010_motor_stop(&simple_motor);
//        LOG_D("Motor stop: %s", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else if (rt_strcmp(argv[1], "brake") == 0)
    {
        if (argc < 3)
        {
            LOG_D("Usage: ms4010_simple brake <cmd>");
            LOG_D("  cmd: 0=lock, 1=release, 16=read status");
            return;
        }
//        rt_uint8_t brake_cmd = atoi(argv[2]);
//        rt_uint8_t brake_status = 0;
//        rt_err_t result = ms4010_brake_control(&simple_motor, brake_cmd, &brake_status);
//        if (brake_cmd == MS4010_BRAKE_READ_STATUS)
//        {
//            LOG_D("Brake status read: %s, status=0x%02X", result == RT_EOK ? "SUCCESS" : "FAILED", brake_status);
//        }
//        else
//        {
//            LOG_D("Brake control cmd=%d: %s", brake_cmd, result == RT_EOK ? "SUCCESS" : "FAILED");
//        }
    }
    else if (rt_strcmp(argv[1], "clear_err") == 0)
    {
//        rt_err_t result = ms4010_clear_error(&simple_motor);
//        LOG_D("Clear error: %s", result == RT_EOK ? "SUCCESS" : "FAILED");
    }
    else
    {
        LOG_D("Unknown command: %s", argv[1]);
    }
}
MSH_CMD_EXPORT(emm_v5_commands, emm_v5_commands);

#endif