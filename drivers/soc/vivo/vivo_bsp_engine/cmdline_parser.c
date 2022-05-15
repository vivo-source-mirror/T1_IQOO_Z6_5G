#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/bug.h>
#include <asm/setup.h>

#include "cmdline_parser.h"

#define FINAL_CMDLINE_SIZE 4096

#define VGLOBAL_FLAG_DECLARE(x) \
	unsigned int x; \
	EXPORT_SYMBOL(x);

#define VGLOBAL_PARAM_FLAG_DECLARE(x) \
	unsigned int x; \
	EXPORT_SYMBOL(x); \
	module_param(x, int, 0444);

struct cmdline_pattern {
	const char *pat;
	unsigned int *flag;
	char *value_string;
	unsigned int value_size;
};

extern char *qpnp_pon_get_pon_reason(void);
extern char *qpnp_pon_get_poff_reason(void);

static struct device_node *of_dtb_chosen;
static char final_cmdline[FINAL_CMDLINE_SIZE];

VGLOBAL_FLAG_DECLARE(bsp_test_mode)
VGLOBAL_PARAM_FLAG_DECLARE(is_debug_mode)
VGLOBAL_FLAG_DECLARE(is_fg_reboot)
VGLOBAL_FLAG_DECLARE(power_off_charging_mode)
VGLOBAL_FLAG_DECLARE(recoverymode)
VGLOBAL_FLAG_DECLARE(update_firmware)
VGLOBAL_FLAG_DECLARE(force_ssr_related)
VGLOBAL_FLAG_DECLARE(is_vivolog_flag)
VGLOBAL_FLAG_DECLARE(is_atboot_eslp0)
VGLOBAL_FLAG_DECLARE(is_atboot_eslp1)
VGLOBAL_FLAG_DECLARE(is_atboot_eslp2)
VGLOBAL_FLAG_DECLARE(is_atbbkj)
VGLOBAL_FLAG_DECLARE(is_atboot)
VGLOBAL_FLAG_DECLARE(is_atCmd_boot)
VGLOBAL_FLAG_DECLARE(is_normal_mode)
VGLOBAL_FLAG_DECLARE(vivo_em_mode)
VGLOBAL_FLAG_DECLARE(ex_fg_support)
VGLOBAL_FLAG_DECLARE(ex_fg_power_on_i2c_try_success)
VGLOBAL_FLAG_DECLARE(enable_slave_charger)
VGLOBAL_FLAG_DECLARE(battery_cout_value)
VGLOBAL_FLAG_DECLARE(fg_soc_really)
VGLOBAL_FLAG_DECLARE(ex_fg_i2c_hand_step)
VGLOBAL_FLAG_DECLARE(ex_fg_i2c_error_counter)
VGLOBAL_FLAG_DECLARE(ex_fg_ffc_support)
VGLOBAL_FLAG_DECLARE(ex_fg_soc)
VGLOBAL_FLAG_DECLARE(ex_fg_state)
VGLOBAL_FLAG_DECLARE(ex_fg_clk_gpio)
VGLOBAL_FLAG_DECLARE(ex_fg_sda_gpio)
VGLOBAL_FLAG_DECLARE(ffc_charge_mode)

static char pmic_status[32];
static unsigned int pmic_status_exist;

static struct cmdline_pattern cmdline_pattern_table[] = {
	{.pat = "boot_bsptmode=1", .flag = &bsp_test_mode},
	{.pat = "boot_bsptmode=1", .flag = &is_debug_mode},
	{.pat = "fg_reboot=1", .flag = &is_fg_reboot},
	{.pat = "androidboot.mode=charger", .flag = &power_off_charging_mode},
	{.pat = "recoverymode=1", .flag = &recoverymode},
	{.pat = "update_firmware=1", .flag = &update_firmware},
	{.pat = "ssr_related=1", .flag = &force_ssr_related},
	{.pat = "vivolog_flag=", .flag = &is_vivolog_flag},
	{.pat = "console-atcmd=vivo_atcmd_eslp", .flag = &is_atboot},
	{.pat = "console-atcmd=vivo_atcmd_eslp0", .flag = &is_atboot_eslp0},
	{.pat = "console-atcmd=vivo_atcmd_eslp1", .flag = &is_atboot_eslp1},
	{.pat = "console-atcmd=vivo_atcmd_eslp2", .flag = &is_atboot_eslp2},
	{.pat = "console-atcmd=vivo_atcmd_bkkj", .flag = &is_atbbkj},
	{.pat = "console-atcmd=vivo_atcmd", .flag = &is_atCmd_boot},
	{.pat = "console-atcmd=null", .flag = &is_normal_mode},
	{.pat = "console-mode=vivo_em_mode", .flag = &vivo_em_mode},
	{.pat = "pmic_status=", .flag = &pmic_status_exist,
	 .value_string = pmic_status, .value_size = sizeof(pmic_status)},
};

static void print_cmdline(const char *cmdline)
{
	const char *prefix_comline = "Cmdline: ";
	unsigned int cmdline_len = strlen(cmdline);
	unsigned int printed_len = 0;
	unsigned int n = COMMAND_LINE_SIZE / 1024;

	printed_len = printk(KERN_INFO "%s%s\n", prefix_comline, cmdline);
	printed_len -= strlen(prefix_comline);
	while (printed_len < cmdline_len && printed_len < COMMAND_LINE_SIZE && n--)
		printed_len += printk(KERN_INFO "%s\n", &cmdline[printed_len]);
}

static void filter_cmdline(const char *cmdline)
{
	const char *str = NULL;
	size_t cmdline_len = strlen(cmdline);
	int i;

	for (i = 0; i < ARRAY_SIZE(cmdline_pattern_table); i++) {
		str = strnstr(cmdline, cmdline_pattern_table[i].pat, cmdline_len);
		if (!str || !cmdline_pattern_table[i].flag)
			continue;
		str += strlen(cmdline_pattern_table[i].pat) - 1;
		if (str > cmdline + cmdline_len - 1)
			continue;
		if (*str == '=') {
			if (cmdline_pattern_table[i].value_string) {
				char *p = NULL;

				strlcpy(cmdline_pattern_table[i].value_string, ++str,
						cmdline_pattern_table[i].value_size);
				p = strnchr(cmdline_pattern_table[i].value_string,
						cmdline_pattern_table[i].value_size, ' ');
				if (p)
					*p = '\0';
				*cmdline_pattern_table[i].flag = 1;
			} else
				sscanf(++str, "%d", cmdline_pattern_table[i].flag);
		} else
			*cmdline_pattern_table[i].flag = 1;
		printk(KERN_DEBUG "%ps: %d %s\n", cmdline_pattern_table[i].flag,
				*cmdline_pattern_table[i].flag,
				cmdline_pattern_table[i].value_string ? : "");
	}
}

static void append_cmdline(void)
{
	const char *pon_reason = qpnp_pon_get_pon_reason();
	const char *poff_reason = qpnp_pon_get_poff_reason();
	char command[256];

	snprintf(command, sizeof(command), " androidboot.bootreason=%s/%s/%s",
			pon_reason, poff_reason, pmic_status);
	strlcat(final_cmdline, command, sizeof(final_cmdline));
}

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, final_cmdline);
	seq_putc(m, '\n');
	return 0;
}

static void reconstruct_cmdline(void)
{
	remove_proc_entry("cmdline", NULL);
	proc_create_single("cmdline", 0, NULL, cmdline_proc_show);
}

static void cmdline_merge(const char *cmdline_orig)
{
	const char *cmdline_ex = NULL;

	WARN(strlen(cmdline_orig) > 1800, "cmdline exceeds 1800 bytes!");
	strlcpy(final_cmdline, cmdline_orig, sizeof(final_cmdline));

	if (!of_property_read_string(of_dtb_chosen, "bootargs_ex", &cmdline_ex)) {
		if (cmdline_ex) {
			WARN(strlen(cmdline_ex) > 1800, "cmdline_ex exceeds 1800 bytes!");
			strlcat(final_cmdline, " ", sizeof(final_cmdline));
			strlcat(final_cmdline, cmdline_ex, sizeof(final_cmdline));
			printk(KERN_INFO "append cmdline_ex done.\n");
		}
	}
}

int cmdline_parser_init(void)
{
	const char *cmdline = NULL;

	of_dtb_chosen = of_find_node_opts_by_path("/chosen", NULL);
	if (!of_dtb_chosen) {
		printk(KERN_ERR "/chosen cannot be found\n");
		return -EINVAL;
	}

	if (of_property_read_string(of_dtb_chosen, "bootargs", &cmdline)) {
		printk(KERN_ERR "bootargs read fail\n");
		return -EINVAL;
	}

	if (!cmdline) {
		printk(KERN_ERR "cmdline is null\n");
		return -EFAULT;
	}

	cmdline_merge(cmdline);
	filter_cmdline(final_cmdline);
	append_cmdline();
	print_cmdline(final_cmdline);
	reconstruct_cmdline();

	return 0;
}

