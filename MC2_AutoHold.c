
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


unsigned char hall_a_status_mc2;
unsigned char move_direction_mc2;
unsigned char hall_moved_count_mc2;

unsigned char lock_hall_table_mc2[30];
unsigned char lock_hall_status_mc2;

unsigned int at_lock_cd_count_mc2;
unsigned int at_lock_on_count_mc2;

unsigned char hall_f_tab_mc2[8] = {0,3,6,2,5,1,4,0};
unsigned char hall_r_tab_mc2[8] = {0,5,3,1,6,4,2,0};

signed int lock_current_limit_val_mc2 = 0;
signed int lock_current_limit_target_mc2 = 0;
unsigned int lock_current_inc_count_mc2;
unsigned int lock_current_dec_count_mc2;

unsigned char motor_fist_lock_flag_mc2 = 0;
unsigned char motor_last_shock_flag_mc2 = 0;
unsigned int motor_shock_count_mc2 = 0;


void exalm_chk_mc2(void)
{
    if(motor_alm_status_lock == 1)
    {
        lock_hall_status_mc2 = hall_last_b;
        if(hall_moved_count_mc2 == 0)
        {
            if(lock_hall_status_mc2 != hall_a_status_mc2)
            {
                hall_moved_count_mc2 ++;
                lock_hall_table_mc2[hall_moved_count_mc2] = lock_hall_status_mc2;
                if(lock_hall_status_mc2 == hall_f_tab_mc2[hall_a_status_mc2])
                    move_direction_mc2 = 1;
                else if(lock_hall_status_mc2 == hall_r_tab_mc2[hall_a_status_mc2])
                    move_direction_mc2 = 0;
            }
        }
        else
        {
            if(lock_hall_status_mc2 != lock_hall_table_mc2[(hall_moved_count_mc2)])
            {

                if(move_direction_mc2 == 1)
                {
                    if(lock_hall_status_mc2 == hall_f_tab_mc2[(lock_hall_table_mc2[(hall_moved_count_mc2)])])
                    {
                        hall_moved_count_mc2 ++;

                    }
                    else if(lock_hall_status_mc2 == hall_r_tab_mc2[(lock_hall_table_mc2[(hall_moved_count_mc2)])])
                    {
                        if(hall_moved_count_mc2 > 0)
                        {
                            hall_moved_count_mc2 --;
                        }
                        else
                            move_direction_mc2 = 0;
                    }
                    lock_hall_table_mc2[hall_moved_count_mc2] = lock_hall_status_mc2;					
                }
                else
                {
                    if(lock_hall_status_mc2 == hall_r_tab_mc2[(lock_hall_table_mc2[(hall_moved_count_mc2)])])
                    {
                        hall_moved_count_mc2 ++;

                    }
                    else if(lock_hall_status_mc2 == hall_f_tab_mc2[(lock_hall_table_mc2[(hall_moved_count_mc2)])])
                    {
                        if(hall_moved_count_mc2 > 0)
                        {
                            hall_moved_count_mc2 --;
                        }
                        else
                            move_direction_mc2 = 1;
                    }
                    lock_hall_table_mc2[hall_moved_count_mc2] = lock_hall_status_mc2;	
                }
            }
            if(hall_moved_count_mc2 > 25)
                hall_moved_count_mc2 = 25;
        }

        if((system_state_flag & LOCK_CD_FLAG) == 0)
        {
            if(motor_fist_lock_flag_mc2 > 3)
            {
                at_lock_cd_count = 0;
                system_state_flag |= LOCK_CD_FLAG;
            }
        }
	}
}


void lock_current_deal_mc2(void)
{
  	if(hall_moved_count_mc2 < 4)
    {
		lock_current_limit_target_mc2 = 0;
    }
	else
	{
        if(motor_fist_lock_flag_mc2 == 0)
        {
            motor_last_shock_flag_mc2 = move_direction_mc2;
            motor_fist_lock_flag_mc2 = 1;
        }
        else
        {
            if(move_direction_mc2 != motor_last_shock_flag_mc2)
            {
                if(motor_fist_lock_flag_mc2 <= 10)
                    motor_fist_lock_flag_mc2 ++;
                motor_last_shock_flag_mc2 = move_direction_mc2;
            }
        }
        lock_current_limit_target_mc2 = __builtin_mulss((hall_moved_count_mc2 - 4),Q_LOCK_CURRENT_STEP);

		if(lock_current_limit_target_mc2 > Q_LOCK_CURRENT_MAX)
		  	lock_current_limit_target_mc2 = Q_LOCK_CURRENT_MAX;
	}
	
	if(lock_current_limit_target_mc2 > lock_current_limit_val_mc2)
	{
	  	if(lock_current_inc_count_mc2 > lock_current_inc_mod)
		{
		  	lock_current_inc_count_mc2 = 0;
			lock_current_limit_val_mc2 += lock_current_inc_val;
		}
		else
		  	lock_current_inc_count_mc2 ++;
	}
	else if(lock_current_limit_target_mc2 < lock_current_limit_val_mc2)
	{
	  	if(lock_current_dec_count_mc2 > lock_current_dec_mod)
		{
		  	lock_current_dec_count_mc2 = 0;
			lock_current_limit_val_mc2 -= lock_current_dec_val;
			if(lock_current_limit_val_mc2 < 0)
			  	lock_current_limit_val_mc2 = 0;
		}
		else
		  	lock_current_dec_count_mc2 ++;
	}
}
