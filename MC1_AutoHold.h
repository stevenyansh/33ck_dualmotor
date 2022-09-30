/* 
 * File:   AutoHold_MC1.h
 * Author: A16842
 *
 * Created on April 7, 2021, 1:41 PM
 */

#ifndef AUTOHOLD_MC1_H
#define	AUTOHOLD_MC1_H

#ifdef	__cplusplus
extern "C" {
#endif

void exalm_chk(void);
void lock_current_deal(void);

#define LOCK_ON_FLAG	0x0080
#define LOCK_CD_FLAG	0x0100

#define Q_LOCK_CURRENT_MAX 6000
#define Q_LOCK_CURRENT_STEP 400

extern unsigned int system_state_flag;
extern signed int lock_current_limit_val;
extern unsigned char move_direction;
extern unsigned char motor_alm_status_lock;
extern unsigned int lock_current_inc_mod;
extern unsigned int lock_current_dec_mod;
extern unsigned int lock_current_inc_val;
extern unsigned int lock_current_dec_val;

extern unsigned char motor_fist_lock_flag;
extern unsigned char motor_last_shock_flag;
extern unsigned int motor_shock_count;

extern unsigned int at_lock_cd_count;

#ifdef	__cplusplus
}
#endif

#endif	/* AUTOHOLD_MC1_H */

