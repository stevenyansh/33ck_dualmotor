
#include <stdint.h>
#include <stdbool.h>

#include <libq.h>

#include "board_service.h"
#include "mc1_init.h"
#include "mc1_service.h"
#include "tricycle_control.h"
#include "mc2_init.h"
#include "mc2_service.h"
#include "mc1_user_params.h" 
#include "MC1_AutoHold.h"
#include "MC2_AutoHold.h"




unsigned char sim_alm_port;
unsigned int system_state_flag;

unsigned char lock_off_count;
unsigned char lock_on_count;
unsigned char hall_a_status;
unsigned char blanking_count;
unsigned char move_direction;
unsigned char motor_alm_status_lock;
unsigned char hall_moved_count;
unsigned char running_lock;
unsigned char speed_low_lock_count;
unsigned char lock_hall_table[30];
unsigned char lock_hall_status;

unsigned int motor_block_count;

unsigned int at_lock_cd_count;
unsigned int at_lock_on_count;

unsigned char hall_f_tab_mc1[8] = {0,3,6,2,5,1,4,0};
unsigned char hall_r_tab_mc1[8] = {0,5,3,1,6,4,2,0};

signed int lock_current_limit_val = 0;
signed int lock_current_limit_target = 0;
unsigned int lock_current_inc_count;
unsigned int lock_current_dec_count;
unsigned int lock_current_inc_mod = 1;
unsigned int lock_current_dec_mod = 1;
unsigned int lock_current_inc_val = 10;
unsigned int lock_current_dec_val = 10;

unsigned char motor_fist_lock_flag = 0;
unsigned char motor_last_shock_flag = 0;
unsigned int motor_shock_count = 0;
void exalm_chk(void)
{
//	
//    sim_alm_port = 1;
// 	if(sim_alm_port)
    if(PARK_PORT == 0)
	{
		if(system_state_flag & LOCK_ON_FLAG)
		{
			lock_off_count = 0;
		}
		else
		{
			if(lock_on_count >= 25)	//init alarm lock
			{
				hall_a_status = hall_last_a;
                hall_a_status_mc2 = hall_last_b;
				hall_moved_count = 0;
                hall_moved_count_mc2 = 0;
				move_direction = 0;
                move_direction_mc2 = 0;
                motor_fist_lock_flag = 0;
                motor_last_shock_flag = 0;
                motor_shock_count = 0;
                motor_fist_lock_flag_mc2 = 0;
                motor_last_shock_flag_mc2 = 0;
                motor_shock_count_mc2 = 0;
				system_state_flag |= LOCK_ON_FLAG;
                motor_alm_status_lock = 1;
			}
			else
				lock_on_count ++;
		}
	}
	else			//alarm over
	{
		if(system_state_flag & LOCK_ON_FLAG)
		{
			if(lock_off_count >= 25)
			{
				motor_alm_status_lock = 0;
				system_state_flag &= (unsigned int)(~LOCK_ON_FLAG);
			}
			else
				lock_off_count ++;
		}
		else
			lock_on_count = 0;
	}
	
	if(system_state_flag & LOCK_ON_FLAG)
	{
		if(motor_alm_status_lock == 1)
		{
			lock_hall_status = hall_last_a;
			if(hall_moved_count == 0)
			{
				if(lock_hall_status != hall_a_status)
				{
					hall_moved_count ++;
					lock_hall_table[hall_moved_count] = lock_hall_status;
					if(lock_hall_status == hall_f_tab_mc1[hall_a_status])
						move_direction = 1;
					else if(lock_hall_status == hall_r_tab_mc1[hall_a_status])
						move_direction = 0;
				}
			}
			else
			{
				if(lock_hall_status != lock_hall_table[(hall_moved_count)])
				{
					
					if(move_direction == 1)
					{
						if(lock_hall_status == hall_f_tab_mc1[(lock_hall_table[(hall_moved_count)])])
						{
							hall_moved_count ++;
				
						}
						else if(lock_hall_status == hall_r_tab_mc1[(lock_hall_table[(hall_moved_count)])])
						{
							if(hall_moved_count > 0)
							{
								hall_moved_count --;
							}
							else
								move_direction = 0;
						}
						lock_hall_table[hall_moved_count] = lock_hall_status;					
					}
					else
					{
						if(lock_hall_status == hall_r_tab_mc1[(lock_hall_table[(hall_moved_count)])])
						{
							hall_moved_count ++;
			
						}
						else if(lock_hall_status == hall_f_tab_mc1[(lock_hall_table[(hall_moved_count)])])
						{
							if(hall_moved_count > 0)
							{
								hall_moved_count --;
							}
							else
								move_direction = 1;
						}
						lock_hall_table[hall_moved_count] = lock_hall_status;	
					}
				}
                if(hall_moved_count > 25)
                    hall_moved_count = 25;
			}

            if((system_state_flag & LOCK_CD_FLAG) == 0)
            {
                if(motor_fist_lock_flag > 3)
                {
                    at_lock_cd_count = 0;
                    system_state_flag |= LOCK_CD_FLAG;
                }
            }
            else
            {
                if(at_lock_cd_count >= 24000)
                {
                    system_state_flag &= (unsigned int)(~LOCK_CD_FLAG);
                    hall_moved_count = 0;
                    hall_moved_count_mc2 = 0;
                    motor_last_shock_flag = 0;
                    motor_fist_lock_flag = 0;
                    motor_last_shock_flag_mc2 = 0;
                    motor_fist_lock_flag_mc2 = 0;                    
                }
                else
                    at_lock_cd_count ++;
            }
		}	
	}
}


void lock_current_deal(void)
{
  	if(hall_moved_count < 4)
    {
		lock_current_limit_target = 0;
    }
	else
	{
        if(motor_fist_lock_flag == 0)
        {
            motor_last_shock_flag = move_direction;
            motor_fist_lock_flag = 1;
        }
        else
        {
            if(move_direction != motor_last_shock_flag)
            {
                if(motor_fist_lock_flag <= 10)
                    motor_fist_lock_flag ++;
                motor_last_shock_flag = move_direction;
            }
        }
        lock_current_limit_target = __builtin_mulss((hall_moved_count - 4),Q_LOCK_CURRENT_STEP);

		if(lock_current_limit_target > Q_LOCK_CURRENT_MAX)
		  	lock_current_limit_target = Q_LOCK_CURRENT_MAX;
	}
	
	if(lock_current_limit_target > lock_current_limit_val)
	{
	  	if(lock_current_inc_count > lock_current_inc_mod)
		{
		  	lock_current_inc_count = 0;
			lock_current_limit_val += lock_current_inc_val;
		}
		else
		  	lock_current_inc_count ++;
	}
	else if(lock_current_limit_target < lock_current_limit_val)
	{
	  	if(lock_current_dec_count > lock_current_dec_mod)
		{
		  	lock_current_dec_count = 0;
			lock_current_limit_val -= lock_current_dec_val;
			if(lock_current_limit_val < 0)
			  	lock_current_limit_val = 0;
		}
		else
		  	lock_current_dec_count ++;
	}
}
