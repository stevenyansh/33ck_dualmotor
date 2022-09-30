/* 
 * File:   AutoHold_MC1.h
 * Author: A16842
 *
 * Created on April 7, 2021, 1:41 PM
 */

#ifndef AUTOHOLD_MC2_H
#define	AUTOHOLD_MC2_H

#ifdef	__cplusplus
extern "C" {
#endif

void exalm_chk_mc2(void);
void lock_current_deal_mc2(void);

extern unsigned char move_direction_mc2;
extern signed int lock_current_limit_val_mc2;
extern unsigned char hall_a_status_mc2;
extern unsigned char hall_moved_count_mc2;

extern unsigned char motor_fist_lock_flag_mc2;
extern unsigned char motor_last_shock_flag_mc2;
extern unsigned int motor_shock_count_mc2;

#ifdef	__cplusplus
}
#endif

#endif	/* AUTOHOLD_MC1_H */

