// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 chickendrop89 <chickendrop89@gmail.com>
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h> 
#include <linux/cpu.h> 
#include <linux/cpufreq.h> 
#include <linux/workqueue.h> 
#include <linux/power_supply.h> 
#include <linux/kobject.h> 
#include <linux/sysfs.h> 
#include <linux/suspend.h>
#include <linux/topology.h>

/*
 * this is here to prevent thrashing/lagging if the device is
 * consecutively going from charging to discharging (and again).
 *
 * A good example of this event is when the charging cable is damaged,
 * and the charging is interrupted when the angle changes.
 * 
 * (Default: ratelimit to 5 seconds minimum between changes)
*/
#define RATELIMIT_INTERVAL_MS (5 * 1000)

/* 
 * because the BMS/fuel gauge drivers are loaded later in the userspace,
 * probe power supply for "battery", and delay this module until they are ready. 
 * 
 * (Default: probe PS every 1 minute until they are availiable)
*/
#define PS_PROBE_INTERVAL_MS (60 * 1000)
#define POWER_SAVING_GOVERNOR CONFIG_CPUFREQ_PSM_GOVERNOR

static atomic_t driver_enabled __read_mostly = ATOMIC_INIT(1);
static atomic_t low_battery __read_mostly = ATOMIC_INIT(0);

static bool system_suspended __read_mostly = false;
static bool notifier_registered = false;
static bool governor_stored = false;

static unsigned int low_battery_threshold __read_mostly =
	CONFIG_CPUFREQ_PSM_THRESHOLD;

static struct {
	struct notifier_block pm;
	struct notifier_block psy;
} notifiers;

static struct {
	int num;
	cpumask_t *cpu_masks;
	unsigned int *max_freqs;
	char *governors;
} clusters;

static struct {
    struct delayed_work probe;
    struct work_struct psy;
    struct workqueue_struct *wq;
} works __cacheline_aligned;

static struct kobject *psm_kobj;
static unsigned long last_change_time;

static DEFINE_MUTEX(governor_mutex);
static DEFINE_SPINLOCK(time_lock);

/**/

static void limit_cpu_freq(struct cpufreq_policy *policy, bool restore)
{
    struct cpufreq_frequency_table *pos;
    unsigned int target_freq, closest_freq, closest_diff = UINT_MAX;
    int cluster_id;
    
    /* get correct cluster id based on policy cpus */
    for (cluster_id = 0; cluster_id < clusters.num; cluster_id++) 
    {
        if (cpumask_intersects(policy->cpus, &clusters.cpu_masks[cluster_id]))
            break;
    }

    /* validate cluster id */
    if (cluster_id >= clusters.num) 
    {
        pr_err("invalid cluster ID for CPU%d\n", policy->cpu);
        return;
    }

    if (restore) 
    {
        if (clusters.max_freqs && clusters.max_freqs[cluster_id]) 
        {
            policy->max = clusters.max_freqs[cluster_id];
            policy->cpuinfo.max_freq = clusters.max_freqs[cluster_id];
            pr_info("cluster %d: Restoring maximum frequency to %u kHz\n",
                   cluster_id, clusters.max_freqs[cluster_id]);
        } else {
            pr_err("no stored frequency for cluster %d\n", cluster_id);
        }
        return;
    }

    /* store current max freq before limiting */
    if (clusters.max_freqs) 
        clusters.max_freqs[cluster_id] = policy->max;

    /* throttle cpu frequencies by half to save power */
    target_freq = policy->cpuinfo.max_freq / 2;

    /* beacause our devices have a discrete set of availiable frequency steps */
    /* we have to select a availiable frequency that is closest to target_freq */
    cpufreq_for_each_valid_entry(pos, policy->freq_table) 
    {
        unsigned int diff;

        if (pos->frequency > target_freq)
            diff = pos->frequency - target_freq;
        else
            diff = target_freq - pos->frequency;

        if (diff < closest_diff) 
        {
            closest_diff = diff;
            closest_freq = pos->frequency;
        }
    }

    pr_info("cluster %d: Limiting maximum frequency to %u kHz\n", 
            cluster_id, closest_freq);
    policy->max = closest_freq;
    policy->cpuinfo.max_freq = closest_freq;
}

static void set_cpu_governor(const char *governor, bool save_current)
{
    struct cpufreq_policy *policy;
    static bool freqs_stored = false;
    int cluster_id;

    mutex_lock(&governor_mutex);

    /* iterate through clusters */
    for (cluster_id = 0; cluster_id < clusters.num; cluster_id++) 
    {
        unsigned int cpu = cpumask_first(&clusters.cpu_masks[cluster_id]);  

        policy = cpufreq_cpu_get(cpu);
        if (!policy)
            continue;

        if (save_current) 
        {
            if (!freqs_stored)
                limit_cpu_freq(policy, false);
            
            if (!governor_stored)
                strncpy(&clusters.governors[cluster_id * CPUFREQ_NAME_LEN],
                        policy->governor->name, CPUFREQ_NAME_LEN);
        } else {
            const char *restore_governor;
            limit_cpu_freq(policy, true);
    
            restore_governor = governor ? governor : 
                &clusters.governors[cluster_id * CPUFREQ_NAME_LEN];
            governor = restore_governor;
        }

        strncpy(policy->governor->name, governor, CPUFREQ_NAME_LEN);
        cpufreq_update_policy(cpu);
        cpufreq_cpu_put(policy);
    }

    if (save_current)
        freqs_stored = governor_stored = true;
    else
        freqs_stored = governor_stored = false;

    mutex_unlock(&governor_mutex);
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
    struct power_supply *psy = ptr;

    if (unlikely(system_suspended))
        return NOTIFY_OK;

    if (likely(strcmp(psy->desc->name, "battery") == 0)) 
    {
        if (!work_pending(&works.psy))
            queue_work(system_wq, &works.psy);
    }

    return NOTIFY_OK;
}

static void psy_work_func(struct work_struct *work)
{
    bool current_low_battery;
    bool discharging = false;
    unsigned long now = jiffies;
    unsigned long flags;

    struct power_supply *psy;
    union power_supply_propval val;
    int capacity;
    int ret;

    if (unlikely(system_suspended))
        return;

    psy = power_supply_get_by_name("battery");
    if (!psy)
        return;

    ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (unlikely(ret)) 
    {
        pr_err("failed to get battery capacity: %d\n", ret);
        goto put_psy;
    }
    capacity = val.intval;

    ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
    if (unlikely(ret)) 
    {
        pr_err("failed to get battery status: %d\n", ret);
        goto put_psy;
    }
    if (likely(val.intval == POWER_SUPPLY_STATUS_DISCHARGING))
        discharging = true;

    current_low_battery = (discharging && capacity <= low_battery_threshold);

    spin_lock_irqsave(&time_lock, flags);

    /* check if enough time has passed since last change */
    if (time_before(now, last_change_time + msecs_to_jiffies(RATELIMIT_INTERVAL_MS))) 
    {
        spin_unlock_irqrestore(&time_lock, flags);

        /* queue work to check again after rate limit expires */
        queue_work(system_wq, &works.psy);
        goto put_psy;
    }

    spin_unlock_irqrestore(&time_lock, flags);

    if (unlikely(current_low_battery && !atomic_read(&low_battery))) 
    {
        pr_info("battery level %d%% below threshold:%d%% and discharging, setting governor to %s\n",
                capacity, low_battery_threshold, POWER_SAVING_GOVERNOR);

        spin_lock_irqsave(&time_lock, flags);
        atomic_set(&low_battery, 1);
        last_change_time = now;
        spin_unlock_irqrestore(&time_lock, flags);

        set_cpu_governor(POWER_SAVING_GOVERNOR, true);
    } else if (unlikely(!current_low_battery && atomic_read(&low_battery))) 
    {
        pr_info("battery level %d%% above threshold:%d%% or charging, restoring governor to %s\n",
                capacity, low_battery_threshold, &clusters.governors[0]);

        spin_lock_irqsave(&time_lock, flags);
        atomic_set(&low_battery, 0);
        last_change_time = now;
        spin_unlock_irqrestore(&time_lock, flags);

        set_cpu_governor(&clusters.governors[0], false);
    }

put_psy:
    power_supply_put(psy);
}

static int start_battery_monitoring(void)
{
    int ret = 0;
    
    if (!notifier_registered) 
    {
        notifiers.psy.notifier_call = psy_changed;
        ret = power_supply_reg_notifier(&notifiers.psy);
        if (ret == 0)
            notifier_registered = true;
    }
    return ret;
}

static void probe_power_supply(struct work_struct *work)
{
    struct power_supply *psy;
    int ret;

    if (unlikely(system_suspended))
        return;

    psy = power_supply_get_by_name("battery");
    if (!psy)
    {
        if (likely(works.wq && !system_suspended)) 
        {
            mod_delayed_work(works.wq, &works.probe,
            msecs_to_jiffies(PS_PROBE_INTERVAL_MS));
        }
        return;
    }

    ret = start_battery_monitoring();
    power_supply_put(psy);

    if (ret == 0) 
        cancel_delayed_work(&works.probe);
}

static int power_suspend_notifier(struct notifier_block *nb,
    unsigned long event, void *dummy)
{
    switch (event) {
    case PM_SUSPEND_PREPARE:
        system_suspended = true;

        /* prevent new work from being queued */
        if (works.wq) 
            cancel_delayed_work(&works.probe);

        return NOTIFY_OK;

    case PM_POST_SUSPEND:
        system_suspended = false;
        /* only requeue if driver is enabled */
        if (works.wq && atomic_read(&driver_enabled)) 
        {
            queue_delayed_work(works.wq, &works.probe, 
                msecs_to_jiffies(PS_PROBE_INTERVAL_MS));
        }
        return NOTIFY_OK;

    default:
        return NOTIFY_DONE;
    }
}

/**/

static ssize_t driver_enabled_show(struct kobject *kobj,
    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&driver_enabled));
}

static ssize_t driver_enabled_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	bool enabled;
	int ret;

	ret = kstrtobool(buf, &enabled);
	if (ret)
		return ret;

	if (enabled != atomic_read(&driver_enabled)) 
    {
		if (!enabled) 
        {
			pr_info("disabled\n");
			atomic_set(&driver_enabled, 0);

			/* unregister notifier when disabling */
            if (notifier_registered) 
            {
		        power_supply_unreg_notifier(&notifiers.psy);
                notifier_registered = false;
            }

			/* restore original governor */
			if (atomic_read(&low_battery)) 
            {
				pr_info("restoring governor to %s due to state change\n",
					&clusters.governors[0]);
				set_cpu_governor(&clusters.governors[0], false);
				atomic_set(&low_battery, 0);
			}
		} else {
			pr_info("enabled\n");
            atomic_set(&driver_enabled, 1);

			/* re-register notifier when enabling */
            if (!notifier_registered) 
            {
			    ret = power_supply_reg_notifier(&notifiers.psy);
			    if (ret) 
                {
			    	pr_err("failed to register power supply notifier: %d\n", ret);
			    	return ret;
			    }
                notifier_registered = true;
		    }
	    }
    }
	return count;
}

static ssize_t low_battery_threshold_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%u\n", low_battery_threshold);
}

static ssize_t low_battery_threshold_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (unlikely(ret))
		return ret;

	if (unlikely(val > 100))
		return -EINVAL;

	pr_info("changed threshold to %u\n", val);
	low_battery_threshold = val;
	return count;
}

static struct kobj_attribute driver_enabled_attr =
	__ATTR(enabled, 0644, driver_enabled_show, driver_enabled_store);

static struct kobj_attribute low_battery_threshold_attr =
	__ATTR(low_battery_threshold, 0644, low_battery_threshold_show,
	low_battery_threshold_store);

static struct attribute *psm_attrs[] = 
{
	&driver_enabled_attr.attr,
	&low_battery_threshold_attr.attr,
	NULL,
};

static struct attribute_group psm_attr_group = 
{
	.attrs = psm_attrs,
};

/**/

static int __init psm_init(void)
{
    int ret, i;
    unsigned int cpu;
    cpumask_t prev_cpus;

    /* count clusters and allocate storage */
    cpumask_clear(&prev_cpus);
    
    for_each_possible_cpu(cpu) 
    {
        if (!cpumask_test_cpu(cpu, &prev_cpus)) 
        {
            clusters.num++;
            cpumask_copy(&prev_cpus, topology_core_cpumask(cpu));
        }
    }

    /* debug number of clusters */
    pr_debug("%d cluster(s)\n", clusters.num);

    clusters.cpu_masks = kcalloc(clusters.num, sizeof(cpumask_t), GFP_KERNEL);
    clusters.max_freqs = kcalloc(clusters.num, sizeof(unsigned int), GFP_KERNEL);
    if (!clusters.cpu_masks || !clusters.max_freqs) 
    {
        kfree(clusters.cpu_masks);
        kfree(clusters.max_freqs);
        return -ENOMEM;
    }

    /* map cpus to clusters */
    i = 0;
    cpumask_clear(&prev_cpus);
    for_each_possible_cpu(cpu)
    {
        if (!cpumask_test_cpu(cpu, &prev_cpus)) 
        {
            cpumask_copy(&clusters.cpu_masks[i], topology_core_cpumask(cpu));
            cpumask_or(&prev_cpus, &prev_cpus, &clusters.cpu_masks[i]);
            i++;
        }
    }

    /* allocate storage for governors */
    clusters.governors = kzalloc(clusters.num * CPUFREQ_NAME_LEN, GFP_KERNEL);
    if (!clusters.governors)
    {
        kfree(clusters.cpu_masks);
        kfree(clusters.max_freqs);
        return -ENOMEM;
    }

    /* create sysfs entries */
    psm_kobj =
	    kobject_create_and_add("power_saving_mode", kernel_kobj);
    if (!psm_kobj)
	    return -ENOMEM;

    ret = sysfs_create_group(psm_kobj, &psm_attr_group);
    if (ret) 
    {
	    kobject_put(psm_kobj);
	    return ret;
    }

    /* create probe workqueue */
    works.wq = alloc_workqueue("battery_probe_wq", 
        /* wq_power_efficient instead of wq_unbound */
        WQ_FREEZABLE | WQ_POWER_EFFICIENT | WQ_MEM_RECLAIM, 1);

    if (!works.wq)
    {
	    sysfs_remove_group(psm_kobj, &psm_attr_group);
	    kobject_put(psm_kobj);
	    return -ENOMEM;
    }

    /* start probing for power supply */
    INIT_DELAYED_WORK(&works.probe, probe_power_supply);
    queue_delayed_work(works.wq, &works.probe, 0);

    /* suspend notifier */
    notifiers.pm.notifier_call = power_suspend_notifier;
    ret = register_pm_notifier(&notifiers.pm);
    if (ret) 
    {
        pr_err("failed to register PM notifier: %d\n", ret);
        destroy_workqueue(works.wq);
	    sysfs_remove_group(psm_kobj, &psm_attr_group);
	    kobject_put(psm_kobj);
	    return ret;
    }

    INIT_WORK(&works.psy, psy_work_func);
    pr_info("init\n");

    return 0;
}

static void __exit psm_exit(void)
{
    /* prevent new work */
    atomic_set(&driver_enabled, 0);
    system_suspended = true;

    /* Ensure synchronization */
    mutex_lock(&governor_mutex);
    unregister_pm_notifier(&notifiers.pm);
    
    if (notifier_registered)
    {
        power_supply_unreg_notifier(&notifiers.psy);
        notifier_registered = false;
    }

    if (works.wq) 
    {
        cancel_delayed_work_sync(&works.probe);
        cancel_work_sync(&works.psy);
        destroy_workqueue(works.wq);
        works.wq = NULL;
    }

    /* restore governors if stored*/
    if (governor_stored)
        set_cpu_governor(NULL, false);

    kfree(clusters.governors);
    kfree(clusters.cpu_masks);
    kfree(clusters.max_freqs);

    if (psm_kobj) 
    {
	    sysfs_remove_group(psm_kobj, &psm_attr_group);
	    kobject_put(psm_kobj);
    }

    pr_info("exit\n");
    mutex_unlock(&governor_mutex);
}

/**/

module_init(psm_init);
module_exit(psm_exit);

module_param(low_battery_threshold, uint, 0644);
MODULE_PARM_DESC(low_battery_threshold, "Low battery level threshold");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("chickendrop89");
MODULE_DESCRIPTION("Power-saving mode");
