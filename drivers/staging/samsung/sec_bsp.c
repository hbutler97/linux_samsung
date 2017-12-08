/* sec_bsp.c
 *
 * Copyright (C) 2014 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/sec_sysfs.h>
#include <linux/sec_bsp.h>

struct boot_event {
	const char *string;
	unsigned int time;
	int freq[2];
	int online;
	int temp[4];
	int order;
};

static struct boot_event boot_initcall[] = {
	{"early",},
	{"core",},
	{"postcore",},
	{"arch",},
	{"subsys",},
	{"fs",},
	{"device",},
	{"late",},
};

static struct boot_event boot_events[] = {
	{"!@Boot: start init process", },
	{"!@Boot: beginofpreloadResources()", },
	{"!@Boot: beginofpreloadClasses()", },
	{"!@Boot: End of preloadResources()", },
	{"!@Boot: EndofpreloadClasses()", },
	{"!@Boot: Entered the Android system server!", },
	{"!@Boot: Start PackageManagerService", },
	{"!@Boot: End PackageManagerService", },
	{"!@Boot: Loop forever", },
	{"!@Boot: performEnableScreen", },
	{"!@Boot: bootcomplete", },
	{"!@Boot: Enabling Screen!", },
	{"!@Boot: Voice SVC is acquired", },
	{"!@Boot: Data SVC is acquired", },
	{"!@Boot_SVC : RIL_UNSOL_RIL_CONNECTED", },
	{"!@Boot_SVC : setRadioPower on", },
	{"!@Boot_SVC : setUiccSubscription", },
	{"!@Boot_SVC : SIM onAllRecordsLoaded", },
	{"!@Boot_SVC : RUIM onAllRecordsLoaded", },
	{"!@Boot_SVC : setupDataCall", },
};

static int prevIndex;
static u32 sec_mct_offset;

extern void get_cpuinfo_cur_freq(int *, int *);
extern void get_exynos_thermal_curr_temp(int *);

void print_format(struct boot_event *data,
	struct seq_file *m, int index, int delta)
{
	seq_printf(m, "%-50s\t%5u\t%5u\t%5d %4d %4d L%d%d%d%d B%d%d%d%d %2d %2d %2d %2d\n",
		data[index].string,
		data[index].time + sec_mct_offset,
		data[index].time, delta,
		data[index].freq[0]/1000,
		data[index].freq[1]/1000,
		data[index].online&1,
		(data[index].online>>1)&1,
		(data[index].online>>2)&1,
		(data[index].online>>3)&1,
		(data[index].online>>4)&1,
		(data[index].online>>5)&1,
		(data[index].online>>6)&1,
		(data[index].online>>7)&1,
		data[index].temp[0],
		data[index].temp[1],
		data[index].temp[2],
		data[index].temp[3]
		);
}

static int sec_boot_stat_proc_show(struct seq_file *m, void *v)
{
	unsigned int i = 0;
	int delta = 0;
	int prev = 0;

	seq_printf(m, "boot event\t\t\t\t\t\ttime"
				"\t\tdelta f_0  f_1  little big  B  L  G  I\n");
	seq_printf(m, "----------------------------------------------------------"
				"----------------------------------------------------\n");
	seq_printf(m, "BOOTLOADER - KERNEL\n");
	seq_printf(m, "----------------------------------------------------------"
				"----------------------------------------------------\n");
	seq_printf(m, "mct was initialized in bl2\t\t\t\t%5u\t%5u\t%5u\n", 0, 0, 0);
	seq_printf(m, "arch_timer_init in kernel\t\t\t\t%5u\t%5u\t%5u\n", sec_mct_offset, 0, sec_mct_offset);

	print_format(boot_initcall, m, 0, boot_initcall[0].time);

	for (i = 1; i < ARRAY_SIZE(boot_initcall); i++) {
		delta = boot_initcall[i].time - boot_initcall[i - 1].time;
		print_format(boot_initcall, m, i, delta);
	}

	seq_printf(m, "----------------------------------------------------------"
				"----------------------------------------------------\n");
	seq_printf(m, "FRAMEWORK\n");
	seq_printf(m, "----------------------------------------------------------"
				"----------------------------------------------------\n");

	i = 0;

	delta = boot_events[0].time;
	print_format(boot_events, m, 0, delta);
	prev = i;
	i = boot_events[i].order;

	while (boot_events[i].string != NULL && i!=0) {
		delta = boot_events[i].time - boot_events[prev].time;
		print_format(boot_events, m, i, delta);
		prev = i;
		i = boot_events[i].order;
	}
	return 0;
}


static int sec_boot_stat_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_boot_stat_proc_show, NULL);
}

static const struct file_operations sec_boot_stat_proc_fops = {
	.open    = sec_boot_stat_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

void sec_boot_stat_get_mct(u32 rate) {
	void __iomem *addr = ioremap(0x101C0100, SZ_4K);

	sec_mct_offset = readl_relaxed(addr) / (rate /1000);
}

void sec_boot_stat_add_initcall(const char *s)
{
	int i = 0;
	unsigned long long t = 0;

	for (i = 0; i < ARRAY_SIZE(boot_initcall); i++) {
		if (!strcmp(s, boot_initcall[i].string)) {
			t = local_clock();
			do_div(t, 1000000);
			boot_initcall[i].time = (unsigned int)t;
			break;
		}
	}
}

void sec_boot_stat_add(const char *c)
{
	int i = 0;
	unsigned long long t = 0;

	for (i = 0; i < ARRAY_SIZE(boot_events); i++) {
		if (!strcmp(c, boot_events[i].string)) {
			if (boot_events[i].time == 0) {
				boot_events[prevIndex].order=i;
				prevIndex=i;
				t = local_clock();
				do_div(t, 1000000);
				boot_events[i].time = (unsigned int)t;
				get_cpuinfo_cur_freq(boot_events[i].freq, &boot_events[i].online);
				get_exynos_thermal_curr_temp(boot_events[i].temp);
			}
			break;
		}
	}
}

static struct device *sec_bsp_dev;

static ssize_t store_boot_stat(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long long t = 0;

	if (!strncmp(buf, "!@Boot: start init process", 26)) {
		t = local_clock();
		do_div(t, 1000000);
		boot_events[0].time = (unsigned int)t;
		get_cpuinfo_cur_freq(boot_events[0].freq, &boot_events[0].online);
		get_exynos_thermal_curr_temp(boot_events[0].temp);
	}

	return count;
}

static DEVICE_ATTR(boot_stat, 0220, NULL, store_boot_stat);

static int __init sec_bsp_init(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create("boot_stat", S_IRUGO, NULL,
				&sec_boot_stat_proc_fops);
	if (!entry)
		return -ENOMEM;
#if 0
	boot_events[SYSTEM_START_LK].time = bootloader_start;
	boot_events[SYSTEM_LK_LOGO_DISPLAY].time = bootloader_display;
	boot_events[SYSTEM_END_LK].time = bootloader_end;
#endif
	sec_bsp_dev = sec_device_create(NULL, "bsp");
	BUG_ON(!sec_bsp_dev);
	if (IS_ERR(sec_bsp_dev))
		pr_err("%s:Failed to create devce\n", __func__);

	if (device_create_file(sec_bsp_dev, &dev_attr_boot_stat) < 0)
		pr_err("%s: Failed to create device file\n", __func__);

	return 0;
}

module_init(sec_bsp_init);
