/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Task isolation support
 *
 * Authors:
 *   Chris Metcalf <cmetcalf@mellanox.com>
 *   Alex Belits <abelits@marvell.com>
 *   Yuri Norov <ynorov@marvell.com>
 */
#ifndef _LINUX_ISOLATION_H
#define _LINUX_ISOLATION_H

//#include <stdarg.h>
#include <linux/errno.h>
#include <linux/cpumask.h>
#include <linux/percpu.h>
#include <linux/irqflags.h>
#include <linux/prctl.h>
#include <linux/types.h>

struct task_struct;

#ifdef CONFIG_TASK_ISOLATION

int task_isolation_message(int cpu, int level, bool supp, const char *fmt, ...);

#define pr_task_isol_emerg(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_EMERG, false, fmt, ##__VA_ARGS__)
#define pr_task_isol_alert(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_ALERT, false, fmt, ##__VA_ARGS__)
#define pr_task_isol_crit(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_CRIT, false, fmt, ##__VA_ARGS__)
#define pr_task_isol_err(cpu, fmt, ...)				\
	task_isolation_message(cpu, LOGLEVEL_ERR, false, fmt, ##__VA_ARGS__)
#define pr_task_isol_warn(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_WARNING, false, fmt, ##__VA_ARGS__)
#define pr_task_isol_notice(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_NOTICE, false, fmt, ##__VA_ARGS__)
#define pr_task_isol_info(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_INFO, false, fmt, ##__VA_ARGS__)
#define pr_task_isol_debug(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_DEBUG, false, fmt, ##__VA_ARGS__)

#define pr_task_isol_emerg_supp(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_EMERG, true, fmt, ##__VA_ARGS__)
#define pr_task_isol_alert_supp(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_ALERT, true, fmt, ##__VA_ARGS__)
#define pr_task_isol_crit_supp(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_CRIT, true, fmt, ##__VA_ARGS__)
#define pr_task_isol_err_supp(cpu, fmt, ...)				\
	task_isolation_message(cpu, LOGLEVEL_ERR, true, fmt, ##__VA_ARGS__)
#define pr_task_isol_warn_supp(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_WARNING, true, fmt, ##__VA_ARGS__)
#define pr_task_isol_notice_supp(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_NOTICE, true, fmt, ##__VA_ARGS__)
#define pr_task_isol_info_supp(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_INFO, true, fmt, ##__VA_ARGS__)
#define pr_task_isol_debug_supp(cpu, fmt, ...)			\
	task_isolation_message(cpu, LOGLEVEL_DEBUG, true, fmt, ##__VA_ARGS__)

#define BIT_LL_TASK_ISOLATION	(0)
#define FLAG_LL_TASK_ISOLATION	(1 << BIT_LL_TASK_ISOLATION)

DECLARE_PER_CPU(unsigned long, ll_isol_flags);
extern cpumask_var_t task_isolation_map;

/**
 * task_isolation_request() - prctl hook to request task isolation
 * @flags:	Flags from <linux/prctl.h> PR_TASK_ISOLATION_xxx.
 *
 * This is called from the generic prctl() code for PR_TASK_ISOLATION.
 *
 * Return: Returns 0 when task isolation enabled, otherwise a negative
 * errno.
 */
extern int task_isolation_request(unsigned int flags);

/**
 * task_isolation_kernel_enter() - clear low-level task isolation flag
 *
 * This should be called immediately after entering kernel.
 */
static __always_inline void task_isolation_kernel_enter(void)
{
	unsigned long flags;

	/*
	 * This function runs on a CPU that ran isolated task.
	 *
	 * We don't want this CPU running code from the rest of kernel
	 * until other CPUs know that it is no longer isolated.
	 * When CPU is running isolated task until this point anything
	 * that causes an interrupt on this CPU must end up calling this
	 * before touching the rest of kernel. That is, this function or
	 * fast_task_isolation_cpu_cleanup() or stop_isolation() calling
	 * it. If any interrupt, including scheduling timer, arrives, it
	 * will still end up here early after entering kernel.
	 * From this point interrupts are disabled until all CPUs will see
	 * that this CPU is no longer running isolated task.
	 *
	 * See also fast_task_isolation_cpu_cleanup().
	 */
	if ((this_cpu_read(ll_isol_flags) & FLAG_LL_TASK_ISOLATION) == 0)
		return;

	local_irq_save(flags);

	/* Clear low-level flags */
	this_cpu_write(ll_isol_flags, 0);

	/*
	 * If something happened that requires a barrier that would
	 * otherwise be called from remote CPUs by CPU kick procedure,
	 * this barrier runs instead of it. After this barrier, CPU
	 * kick procedure would see the updated ll_isol_flags, so it
	 * will run its own IPI to trigger a barrier.
	 */
	smp_mb();
	/*
	 * Synchronize instructions -- this CPU was not kicked while
	 * in isolated mode, so it might require synchronization.
	 * There might be an IPI if kick procedure happened and
	 * ll_isol_flags was already updated while it assembled a CPU
	 * mask. However if this did not happen, synchronize everything
	 * here.
	 */
	instr_sync();
	local_irq_restore(flags);
}

extern void task_isolation_cpu_cleanup(void);
/**
 * task_isolation_start() - attempt to actually start task isolation
 *
 * This function should be invoked as the last thing prior to returning to
 * user space if TIF_TASK_ISOLATION is set in the thread_info flags.  It
 * will attempt to quiesce the core and enter task-isolation mode.  If it
 * fails, it will reset the system call return value to an error code that
 * indicates the failure mode.
 */
extern void task_isolation_start(void);

/**
 * is_isolation_cpu() - check if CPU is intended for running isolated tasks.
 * @cpu:	CPU to check.
 */
static inline bool is_isolation_cpu(int cpu)
{
	return task_isolation_map != NULL &&
		cpumask_test_cpu(cpu, task_isolation_map);
}

/**
 * task_isolation_on_cpu() - check if the cpu is running isolated task
 * @cpu:	CPU to check.
 *
 * Caller is responsible for proper read memory barriers before
 * calling this function.
 */
static inline int task_isolation_on_cpu(int cpu)
{
	return test_bit(BIT_LL_TASK_ISOLATION, &per_cpu(ll_isol_flags, cpu));
}

extern void task_isolation_check_run_cleanup(void);

/**
 * task_isolation_cpumask() - set CPUs currently running isolated tasks
 * @mask:	Mask to modify.
 */
extern void task_isolation_cpumask(struct cpumask *mask);

/**
 * task_isolation_clear_cpumask() - clear CPUs currently running isolated tasks
 * @mask:      Mask to modify.
 */
extern void task_isolation_clear_cpumask(struct cpumask *mask);

/**
 * task_isolation_syscall() - report a syscall from an isolated task
 * @nr:		The syscall number.
 *
 * This routine should be invoked at syscall entry if TIF_TASK_ISOLATION is
 * set in the thread_info flags.  It checks for valid syscalls,
 * specifically prctl() with PR_TASK_ISOLATION, exit(), and exit_group().
 * For any other syscall it will raise a signal and return failure.
 *
 * Return: 0 for acceptable syscalls, -1 for all others.
 */
extern int task_isolation_syscall(int nr);

/**
 * _task_isolation_interrupt() - report an interrupt of an isolated task
 * @fmt:	A format string describing the interrupt
 * @...:	Format arguments, if any.
 *
 * This routine should be invoked at any exception or IRQ if
 * TIF_TASK_ISOLATION is set in the thread_info flags.  It is not necessary
 * to invoke it if the exception will generate a signal anyway (e.g. a bad
 * page fault), and in that case it is preferable not to invoke it but just
 * rely on the standard Linux signal.  The macro task_isolation_syscall()
 * wraps the TIF_TASK_ISOLATION flag test to simplify the caller code.
 */
extern void _task_isolation_interrupt(const char *fmt, ...);
#define task_isolation_interrupt(fmt, ...)				\
	do {								\
		if (current_thread_info()->flags & _TIF_TASK_ISOLATION)	\
			_task_isolation_interrupt(fmt, ## __VA_ARGS__);	\
	} while (0)

/**
 * task_isolation_remote() - report a remote interrupt of an isolated task
 * @cpu:	The remote cpu that is about to be interrupted.
 * @fmt:	A format string describing the interrupt
 * @...:	Format arguments, if any.
 *
 * This routine should be invoked any time a remote IPI or other type of
 * interrupt is being delivered to another cpu. The function will check to
 * see if the target core is running a task-isolation task, and generate a
 * diagnostic on the console if so; in addition, we tag the task so it
 * doesn't generate another diagnostic when the interrupt actually arrives.
 * Generating a diagnostic remotely yields a clearer indication of what
 * happened then just reporting only when the remote core is interrupted.
 *
 */
extern void task_isolation_remote(int cpu, const char *fmt, ...);

/**
 * task_isolation_remote_cpumask() - report interruption of multiple cpus
 * @mask:	The set of remotes cpus that are about to be interrupted.
 * @fmt:	A format string describing the interrupt
 * @...:	Format arguments, if any.
 *
 * This is the cpumask variant of _task_isolation_remote().  We
 * generate a single-line diagnostic message even if multiple remote
 * task-isolation cpus are being interrupted.
 */
extern void task_isolation_remote_cpumask(const struct cpumask *mask,
					  const char *fmt, ...);

/**
 * _task_isolation_signal() - disable task isolation when signal is pending
 * @task:	The task for which to disable isolation.
 *
 * This function generates a diagnostic and disables task isolation; it
 * should be called if TIF_TASK_ISOLATION is set when notifying a task of a
 * pending signal.  The task_isolation_interrupt() function normally
 * generates a diagnostic for events that just interrupt a task without
 * generating a signal; here we need to hook the paths that correspond to
 * interrupts that do generate a signal.  The macro task_isolation_signal()
 * wraps the TIF_TASK_ISOLATION flag test to simplify the caller code.
 */
extern void _task_isolation_signal(struct task_struct *task);
#define task_isolation_signal(task)					\
	do {								\
		if (task_thread_info(task)->flags & _TIF_TASK_ISOLATION) \
			_task_isolation_signal(task);			\
	} while (0)

/**
 * task_isolation_user_exit() - debug all user_exit calls
 *
 * By default, we don't generate an exception in the low-level user_exit()
 * code, because programs lose the ability to disable task isolation: the
 * user_exit() hook will cause a signal prior to task_isolation_syscall()
 * disabling task isolation.  In addition, it means that we lose all the
 * diagnostic info otherwise available from task_isolation_interrupt() hooks
 * later in the interrupt-handling process.  But you may enable it here for
 * a special kernel build if you are having undiagnosed userspace jitter.
 */
static inline void task_isolation_user_exit(void)
{
#ifdef DEBUG_TASK_ISOLATION
	task_isolation_interrupt("user_exit");
#endif
}

#else /* !CONFIG_TASK_ISOLATION */
static inline int task_isolation_request(unsigned int flags) { return -EINVAL; }
static inline void task_isolation_kernel_enter(void) {}
static inline void task_isolation_start(void) { }
static inline bool is_isolation_cpu(int cpu) { return 0; }
static inline int task_isolation_on_cpu(int cpu) { return 0; }
static inline void task_isolation_cpumask(struct cpumask *mask) { }
static inline void task_isolation_clear_cpumask(struct cpumask *mask) { }
static inline void task_isolation_cpu_cleanup(void) { }
static inline void task_isolation_check_run_cleanup(void) { }
static inline int task_isolation_syscall(int nr) { return 0; }
static inline void task_isolation_interrupt(const char *fmt, ...) { }
static inline void task_isolation_remote(int cpu, const char *fmt, ...) { }
static inline void task_isolation_remote_cpumask(const struct cpumask *mask,
						 const char *fmt, ...) { }
static inline void task_isolation_signal(struct task_struct *task) { }
static inline void task_isolation_user_exit(void) { }
#endif

#endif /* _LINUX_ISOLATION_H */
