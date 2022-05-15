/***************************************************************************************
  vivo fingerprint team created @ 2021/206/18
***************************************************************************************/
#define pr_fmt(fmt)		"[FP_KERN] " KBUILD_MODNAME ": " fmt

#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/input.h>

#define EVENT_TYPE_DOWN    0
#define EVENT_TYPE_UP      1
#define EVENT_TYPE_MOVE    2

#define RESERVED_LENGTH 8

struct touch_event {
	int panel;						/* 0-main  1-other */
	int fid;                        /* Finger ID */
	int type;                       /* 0 - Down, 1 - Move, 2 - Up, */
	int x;                          /*Coordinate X*/
	int y;                          /*Coordinate y*/
	int inside;                     /*the touch area inside fp sensor */
	int outside;                    /*the touch area outside fp sensor */
	int major_axis;                 /*the major axis axis of touch area*/
	int minor_axis;                 /*the minor axis of touch arear */
	int pressure;                   /*the pressure*/
	int signal;                     /*mean value of touch*/
	int reserved[RESERVED_LENGTH];  /*reserved data*/
};

extern int vts_notifier_chain_register(struct notifier_block *nb);
extern int vts_notifier_chain_unregister(struct notifier_block *nb);
extern void fp_sendmsg(void *message, int message_len);

void touch_event_filter(unsigned int type, unsigned int code, int value)
{
	if (type == EV_KEY && code == KEY_FINGERPRINT_WAKE) {
		struct touch_event data;
		if (value == 1) {
			data.type = EVENT_TYPE_DOWN;
			pr_info("finger press down [type:%d, code:%d, value:%d]", type, code, value);
		} else {
			data.type = EVENT_TYPE_UP;
			pr_info("finger press up[type:%d, code:%d, value:%d]", type, code, value);
		}
		fp_sendmsg((void *)&data, sizeof(struct touch_event));
	}
}
int touch_event_notifiy(struct notifier_block *self, unsigned long action, void *data)
{
	struct touch_event *event = (struct touch_event *)data;
	fp_sendmsg((void *)data, sizeof(struct touch_event));
	pr_info("panel %d, fid:%d, type:%d, x:%d,y:%d", event->panel, event->fid,  event->type, event->x, event->y);
	return NOTIFY_OK;
}

struct notifier_block touch_event_notifier = {
	.notifier_call = touch_event_notifiy,
};

/*将notifier_block节点加到touch_notifier_list链表中*/
int register_touch_event_notifier(void)
{
	return vts_notifier_chain_register(&touch_event_notifier);
}

int unregister_touch_event_notifier(void)
{
	return vts_notifier_chain_unregister(&touch_event_notifier);
}
