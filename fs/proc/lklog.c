/***
*
* CONFIG_TINNO_LK_LOG_SUPPORT function codes
*
***/
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/string.h>
#include <linux/slab.h>

const char *lklog_str = NULL;

int __init lklog_dt_scan_chosen(unsigned long node, const char *uname,
				     int depth, void *data)
{
	int l;
	char *p = NULL;
	char *temp;
	pr_debug("search \"chosen\", depth: %d, uname: %s\n", depth, uname);

	if (depth != 1 ||
	    (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	/* Retrieve command line */
	lklog_str = of_get_flat_dt_prop(node, "atag,lklog", &l);
	if(lklog_str) {
        temp = kmalloc(strlen(lklog_str)+1, GFP_KERNEL);
        if(temp) {
            memset(temp,0,strlen(lklog_str)+1);
			strcpy(temp,lklog_str);
	//			 	printk("proc lk log lenght %d\n",strlen(lklog_str));
	//			 	printk("kmsg lk log lenght %d\n",strlen(temp));
				 	   
			pr_debug("--------------- tinno-lk-log-start ----------------------\n");
			while((p = strsep(&temp,"\n")) != NULL)  
                pr_debug("[ lk ] %s\n", p);
			pr_debug("---------------  tinno-lk-log-end  ----------------------\n");

			kfree(temp);
		}
	}
	return 1;
}

static int lklog_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", lklog_str);
	return 0;
}

static int lklog_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, lklog_proc_show, NULL);
}

static const struct file_operations lklog_proc_fops = {
	.open		= lklog_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_lklog_init(void)
{
	of_scan_flat_dt(lklog_dt_scan_chosen, NULL);

	proc_create("lklog", 0, NULL, &lklog_proc_fops);
	return 0;
}

static void __exit proc_lklog_exit(void)
{
	if(lklog_str) {
        kfree(lklog_str);
    }
}

late_initcall(proc_lklog_init);
module_exit(proc_lklog_exit);
