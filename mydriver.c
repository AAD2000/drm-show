#include "mylib.h"
#include <linux/device.h>
#include <drm/drm.h>
#include <drm/drm_ioctl.h>
#include <linux/platform_device.h>
#include <drm/drm_drv.h>

#include <linux/mod_devicetable.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <drm/drm_gem.h>
#include <drm/drm_mm.h>
#include <drm/drm_gem_cma_helper.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/pci.h>


#include <drm/drm_mode.h>






enum drm_zocl_execbuf_state {
  DRM_ZOCL_EXECBUF_STATE_COMPLETE = 0,
  DRM_ZOCL_EXECBUF_STATE_RUNNING,
  DRM_ZOCL_EXECBUF_STATE_SUBMITTED,
  DRM_ZOCL_EXECBUF_STATE_QUEUED,
  DRM_ZOCL_EXECBUF_STATE_ERROR,
  DRM_ZOCL_EXECBUF_STATE_ABORT,
};

struct drm_zocl_exec_metadata {
	enum drm_zocl_execbuf_state state;
	unsigned int                index;
};

struct drm_zocl_bo {
	union {
		struct drm_gem_cma_object       cma_base;
		struct {
			struct drm_gem_object         gem_base;
			struct page                 **pages;
			struct sg_table              *sgt;
			void                         *vmapping;
			uint64_t                      uaddr;
		};
	};
	struct drm_mm_node            *mm_node;
	struct drm_zocl_exec_metadata  metadata;
	unsigned int                   mem_index;
	uint32_t                       flags;
	unsigned int                   user_flags;
};

int
zocl_create_bo_ioctl(struct drm_device *dev, void *data, struct drm_file *filp)
{
	printk("CALLED IOCTL");
	return 0;
}


static const struct drm_ioctl_desc zocl_ioctls[] = {
	DRM_IOCTL_DEF_DRV(ZOCL_CREATE_BO, zocl_create_bo_ioctl,
			DRM_AUTH|DRM_UNLOCKED|DRM_RENDER_ALLOW)
};

struct drm_gem_object *
zocl_gem_create_object(struct drm_device *dev, size_t size)
{
	struct drm_zocl_bo *bo = kzalloc(sizeof(struct drm_zocl_bo), GFP_KERNEL);

	return (&bo->gem_base);
}

static const struct file_operations zocl_driver_fops = {
	.open           = drm_open,
	.read           = drm_read,
	.unlocked_ioctl = drm_ioctl,
	.release        = drm_release,
};


static struct drm_driver mydrm = {

	.driver_features           = DRIVER_GEM | DRIVER_RENDER,


	.gem_create_object         = zocl_gem_create_object,

	.ioctls                    = zocl_ioctls,
	.num_ioctls                = 1,
	.fops                      = &zocl_driver_fops,

};




static int my_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{

   struct drm_device *drm;
      printk("===== DRM PLATFORM PROBE =====");

      drm = drm_dev_alloc(&mydrm, &pdev->dev);


      int res2 = drm_dev_register(drm, 0);

      if(res2 == 0){
          printk("----- SUCCESS -----%s----%s",mydrm.name, drm->unique);
      } else {
          printk("----- ERROR -----");
      }

      return 0;

}

static void my_pci_remove(struct pci_dev *pdev)
{
}

static const struct pci_device_id my_pci_table[] =  { {
     PCI_DEVICE(PCI_ANY_ID,PCI_ANY_ID)
     }, { /* end: all zeroes */ }
};

static struct pci_driver my_pci_driver = {
 .name =  "my-drm",
 .id_table = my_pci_table,
 .probe = my_pci_probe,
 .remove = my_pci_remove,
};






static int __init etx_driver_init(void)
{
  int i =pci_register_driver(&my_pci_driver);
  printk("%d", i);
  return 0;

}

/*
** Module exit function
*/
static void __exit etx_driver_exit(void){
  pci_unregister_driver(&my_pci_driver);
}


module_init(etx_driver_init);
module_exit(etx_driver_exit);

//drm_module_pci_driver(hibmc_pci_driver);

MODULE_DEVICE_TABLE(pci, my_pci_table);
MODULE_AUTHOR("RongrongZou <zourongrong@huawei.com>");
MODULE_DESCRIPTION("DRM Driver for Hisilicon Hibmc");
MODULE_LICENSE("GPL v2");
