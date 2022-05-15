/*
 * Copyright (C) 2016 vivo Co., Ltd.
 * YangChun <yangchun@vivo.com.cn>
 *
 * This driver is used to export hardware board information for users
 *
**/

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <soc/qcom/socinfo.h>
#include <soc/qcom/smem.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#define BOARD_REV_LEN 16
#define BOARD_NAME_LEN 24
#define VIVO_VENDOR_LEN 8
#define VIVO_CPU_TYPE_LEN 8
#define FREQ_STR_LEN 8
#define INVALID_CPU_FREQ "0"
#define INVALID_CPU_TYPE "unkown"
#define CPU_REVISION_ADDR 0x000A607C
#define VIVO_HW_VERSION_MASK (0xF<<28)
#define VIVO_PM_STATUS_LEN 32

//store in shared memory
struct boardinfo_smem{
	uint32_t board_id;
	char board_rev[BOARD_REV_LEN];
	uint32_t size;
	uint32_t type;
	char board_name[BOARD_NAME_LEN];
	uint32_t ddr_manufacture_id;
	uint32_t lcm_id;
	uint32_t dp_status;	/*add wuyuanhan, dp image load or not.*/
	uint32_t reserved;//make len as a multiple of 8
} *boardinfo_smem;

struct boardinfo_ext{
	char vendor[VIVO_VENDOR_LEN];
	unsigned int cpu_freq;
	char cpu_type[VIVO_CPU_TYPE_LEN];
    char user_cpu_freq[FREQ_STR_LEN];				// max cpu_freq|string|unit GHz|for setting app display to user... 2018/10/25  wuyuanhan, 
	unsigned int core_num;
} *boardinfo_ext;

typedef struct freq_base_map
{
	uint32_t board_id;
	uint32_t act_freq;
	char user_freq[FREQ_STR_LEN];
    char cpu_type[VIVO_CPU_TYPE_LEN];
} freq_base_map_t;

#if 0

Î¬»¤Ô­Ôò:
1¡¢ÓÃ»§ÏÔÊ¾CPUÆµÂÊÀ´Ô´ÓÚcpuÐ¾Æ¬ÊÖ²á /sys/bus/soc/devices/soc1/user_cpu_freq£¬µ¥Î»GHz.
2¡¢Êµ¼ÊCPU¹¤×÷ÆµÂÊ£¬ÒÔ/sys/bus/cpu/devices/cpu7/cpufreq/cpuinfo_max_freq & /sys/bus/soc/devices/soc1/cpu_freq Îª×¼
3¡¢freq_maps Ó³Éä±íÖ§³ÖÒ»¸öboardid£¬Ê¹ÓÃ²»Í¬ÆµÂÊ²»Í¬CPUµÄÇé¿ö¡£
4¡¢Ã¿¸öBoarid ¶¼ÐèÒªÅäÖÃ£¬²»È»¶ÁÈ¡µ½µÄCPUÆµÂÊÎª0£¬CPUÐÍºÅÎª unkonw.
5¡¢map±íÆ¥ÅäÔ­Ôò:
   5.1 Æ¥Åäboardid£¬Î´ÕÒµ½ ÉèÖÃÎÞÐ§ÆµÂÊ"0"
   5.2 Ö»Æ¥Åäµ½boardid£¬Î´ÅäÖÃµ½Êµ¼Ê¹¤×÷ÆµÂÊcpu,»ñÈ¡Ä¬ÈÏÏÔÊ¾cpuÆµÂÊ£¬CPUÐÍºÅÐÅÏ¢
   5.3 Æ¥Åäµ½boardid&ÅäÖÃµ½Êµ¼Ê¹¤×÷ÆµÂÊcpu,¶ÔÓ¦Êµ¼ÊÆµÂÊcpuÆµÂÊµÄÏÔÊ¾CPUÆµÂÊ£¬CPUÐÍºÅÐÅÏ¢

/* -----------------  1.  ¸ÃÆ½Ì¨Éæ¼°µ½µÄËùÓÐCPUÊÖ²á CPU¹¤×÷ÆµÂÊÐÅÏ¢                   ----------------------- */
sm6150:
2xKryo 360 Gold 2.0GHz

sm7150:
2xKryo 360 Gold 2.2GHz

/* -----------------  2.  ¸ÃÆ½Ì¨xbl/sblÏÂ board_id ÐÅÏ¢¿½±´£¬ÐÂÔöboard_idÊ±£¬ÐèÒªÍ¬²½¸üÐÂ  ------------------ */
static vivo_board_id_t board_ids[] = {
{0,"SM6125",},
{1,"TD1901",},




#endif

static freq_base_map_t freq_maps[] = 
{
	{0, 0, "1.95", "439"},//SDM439
	{11, 0, "1.95", "439"},//PD1818
	{12, 0, "1.95", "439"},//VTD1905
	{20, 0, "1.95", "439"},//PD1930
	{24, 0, "1.95", "439"},//PD1930F_EX
	{25, 0, "1.95", "439"},//PD1930BF_EX
	{30, 0, "1.95", "439"},//PD2060F_EX
	{31, 0, "1.95", "439"},//PD2060F_EX_WIFI5G
};

static ssize_t vivo_show_board_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_smem->board_id);
}

static ssize_t vivo_show_board_name(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_smem->board_name);
}
static ssize_t vivo_show_ddr_manufacture_id(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n",boardinfo_smem->ddr_manufacture_id);
}
static ssize_t vivo_show_vendor(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->vendor);
}
static ssize_t vivo_show_cpu_freq(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_ext->cpu_freq);
}
static ssize_t vivo_show_cpu_type(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->cpu_type);
}
static ssize_t vivo_show_user_cpu_freq(struct device *dev,struct device_attribute *attr,char *buf)
{ 
    return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->user_cpu_freq);
}

static ssize_t vivo_show_core_num(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_ext->core_num);
}

static struct device_attribute vivo_vendor = __ATTR(vendor, S_IRUGO, vivo_show_vendor,  NULL);

static struct device_attribute vivo_board_id = __ATTR(board_id, S_IRUGO,vivo_show_board_id, NULL);

static struct device_attribute vivo_board_name = __ATTR(board_name, S_IRUGO,vivo_show_board_name, NULL);

static struct device_attribute vivo_ddrinfo = __ATTR(ddrinfo, S_IRUGO, vivo_show_ddr_manufacture_id,  NULL);

static struct device_attribute vivo_cpu_freq = __ATTR(cpu_freq, S_IRUGO,vivo_show_cpu_freq, NULL);

static struct device_attribute vivo_cpu_user_freq = __ATTR(user_cpu_freq, S_IRUGO,vivo_show_user_cpu_freq, NULL);

static struct device_attribute vivo_cpu_type = __ATTR(cpu_type, S_IRUGO, vivo_show_cpu_type, NULL);	

static struct device_attribute vivo_core_num = __ATTR(core_num, S_IRUGO, vivo_show_core_num, NULL);			

static void __init populate_soc_sysfs_files(struct device *vivo_soc_device)
{
	device_create_file(vivo_soc_device, &vivo_board_id);
	device_create_file(vivo_soc_device, &vivo_board_name);
	device_create_file(vivo_soc_device, &vivo_vendor);
	device_create_file(vivo_soc_device, &vivo_ddrinfo);
	device_create_file(vivo_soc_device, &vivo_cpu_freq);
    device_create_file(vivo_soc_device, &vivo_cpu_user_freq);
	device_create_file(vivo_soc_device, &vivo_cpu_type);
	device_create_file(vivo_soc_device, &vivo_core_num);
	return;
}
	
static int vivo_get_max_freq(unsigned int cpu_id){
	int max_freq = 0;
	int cur_freq = 0;
	int i = 0 ;
	struct cpufreq_frequency_table *table = NULL;
	struct cpufreq_policy *policy = NULL;
	policy = cpufreq_cpu_get(cpu_id);
	if(policy == NULL){
		return 0;
	}
	table = cpufreq_frequency_get_table(policy->cpu);
	cpufreq_cpu_put(policy);
	if(table == NULL){
		pr_err("vivo get frequency of CPU%u fail\n",cpu_id);
		return 0;
	}
	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		cur_freq = table[i].frequency;
		if (cur_freq == CPUFREQ_ENTRY_INVALID)
			continue;
		if(cur_freq > max_freq){
			max_freq = cur_freq;
		}
		//pr_err("CPU%u:table[%d]=%d\n",cpu_id,i,cur_freq);
	}
	return max_freq;
}


static unsigned int vivo_get_cpu_freq(void ){
	int cpu_id = 0;
	unsigned int max_freq = 0;
	unsigned int cur_freq = 0;
	int num_cpus = num_possible_cpus();
	for(cpu_id = 0;cpu_id < num_cpus;cpu_id++){
		if(cpu_online(cpu_id)){
			cur_freq = vivo_get_max_freq(cpu_id);
			if(cur_freq > max_freq){
				max_freq = cur_freq;
			}
		}
	}
	
	return max_freq;
}

static void get_user_cpu_freq_and_type(void ){
    
	int i = 0;
	int default_index = -1;
 
	for (i = 0; i < (sizeof (freq_maps) / sizeof (freq_maps[0])); i++) {
		if (freq_maps[i].board_id == boardinfo_smem->board_id) {
			if ((freq_maps[i].act_freq == boardinfo_ext->cpu_freq) && (freq_maps[i].act_freq != 0)) {				
				pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[i].user_freq); 
				strncpy(boardinfo_ext->user_cpu_freq, freq_maps[i].user_freq, FREQ_STR_LEN);
                strncpy(boardinfo_ext->cpu_type, freq_maps[i].cpu_type, VIVO_CPU_TYPE_LEN);
                return;
			} else if (freq_maps[i].act_freq == 0) {
			    default_index = i;
			}
		}
	}
	
	if (default_index >= 0) {
		pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[i].user_freq);
        strncpy(boardinfo_ext->user_cpu_freq, freq_maps[default_index].user_freq, VIVO_CPU_TYPE_LEN);
        strncpy(boardinfo_ext->cpu_type, freq_maps[default_index].cpu_type, VIVO_CPU_TYPE_LEN);
		return;
	} else {		
		pr_err("vivo board_info: error: Need to set cpu max freq for user!!!\n"); 
		/* 未找到有效boardid时，强制panic,防止未配置CPU频率的版本意外流�?  */
		panic("Can not find a valid board id!");
		return;
	}
}

static void vivo_boardinfo_ext_init(void){
	
	boardinfo_ext = kzalloc(sizeof(*boardinfo_ext), GFP_KERNEL);
	if (!boardinfo_ext) {
		pr_err("boardinfo_ext alloc failed!\n");
		return;
	}
    //cpu max frequency
    boardinfo_ext->cpu_freq = vivo_get_cpu_freq();
    
	//user frequency & type of cpu
	get_user_cpu_freq_and_type();
	//core number
	boardinfo_ext->core_num = num_possible_cpus();
	
	//vendor
	strncpy(boardinfo_ext->vendor,"vivo",VIVO_VENDOR_LEN); //vivo
	pr_err("vivo cpu_freq:%u user_cpu_freq:%sGHz core_num=%u cpu_type=%s\n",boardinfo_ext->cpu_freq,
            boardinfo_ext->user_cpu_freq,
			boardinfo_ext->core_num,
			boardinfo_ext->cpu_type);
}
static int __init vivo_boardinfo_init_sysfs(void)
{
	struct device *vivo_soc_device;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;

	if (!boardinfo_smem) {
		pr_err("No boardinfo found!\n");
		return -ENODEV;
	}

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		pr_err("Soc Device alloc failed!\n");
		return -ENOMEM;
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		kfree(soc_dev_attr);
		pr_err("Soc device register failed\n");
		return -EIO;
	}
		
	vivo_soc_device = soc_device_to_device(soc_dev);
	
	populate_soc_sysfs_files(vivo_soc_device);
	
	//extra information init
	vivo_boardinfo_ext_init();
	
	return 0;
}
late_initcall(vivo_boardinfo_init_sysfs);

char *get_bbk_board_name(void)
{
	if (boardinfo_smem) 
		return boardinfo_smem->board_name;
	return NULL;
}

EXPORT_SYMBOL(get_bbk_board_name);

static void vivo_boardinfo_print(void)
{
	pr_info("board_id=%d, board_version=%s, type=%d, board_name:%s\n",
		boardinfo_smem->board_id, boardinfo_smem->board_rev, 
		boardinfo_smem->type, boardinfo_smem->board_name);
}

int __init vivo_boardinfo_init(void)
{
	static bool boardinfo_init_done;
	unsigned size;
	if (boardinfo_init_done)
		return 0;

	boardinfo_smem = smem_get_entry(SMEM_ID_VENDOR0, &size, 0, SMEM_ANY_HOST_FLAG);

	if (IS_ERR_OR_NULL(boardinfo_smem))
		BUG_ON("Can't find SMEM_ID_VENDOR0 for vivo boardinfo_smem!\n");

	vivo_boardinfo_print();
	boardinfo_init_done = true;
	return 0;
}
subsys_initcall(vivo_boardinfo_init);
