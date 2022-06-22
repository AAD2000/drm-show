#include "../common/mylib.h"
#include "__error.h"
#include <linux/device.h>
#include <linux/iommu.h>
#include <linux/pagemap.h>
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
#include "my_xrt_mem.h"



enum drm___execbuf_state {
  DRM___EXECBUF_STATE_COMPLETE = 0,
  DRM___EXECBUF_STATE_RUNNING,
  DRM___EXECBUF_STATE_SUBMITTED,
  DRM___EXECBUF_STATE_QUEUED,
  DRM___EXECBUF_STATE_ERROR,
  DRM___EXECBUF_STATE_ABORT,
};



struct drm___exec_metadata {
	enum drm___execbuf_state state;
	unsigned int                index;
};

struct drm___bo {
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
	struct drm___exec_metadata  metadata;
	unsigned int                   mem_index;
	uint32_t                       flags;
	unsigned int                   user_flags;
};

#define __BO_FLAGS_CACHEABLE		(1 << 24)
#define __BO_FLAGS_HOST_BO		(1 << 25)
#define __BO_FLAGS_COHERENT		(1 << 26)
#define __BO_FLAGS_SVM		(1 << 27)
#define __BO_FLAGS_USERPTR		(1 << 28)
#define __BO_FLAGS_CMA		(1 << 29)
#define __BO_FLAGS_P2P		(1 << 30)
#define __BO_FLAGS_EXECBUF		(1 << 31)

/* BO types we support */
#define __BO_NORMAL	(XRT_DRV_BO_HOST_MEM | XRT_DRM_SHMEM | \
			XRT_DRV_BO_DRM_ALLOC)
#define __BO_EXECBUF	(__BO_NORMAL)
#define __BO_CACHE	(__BO_NORMAL | XRT_CACHEABLE)
#define __BO_USERPTR	(XRT_USER_MEM | XRT_DRV_BO_USER_ALLOC)
#define __BO_SVM	(XRT_DRV_BO_HOST_MEM | XRT_DRM_SHMEM | \
			XRT_DRV_BO_DRM_ALLOC)
#define __BO_PL_DDR	(XRT_DEVICE_MEM)
#define __BO_HOST_BO	(XRT_DRV_BO_HOST_MEM)
#define __BO_IMPORT	(XRT_DRM_IMPORT | XRT_DRV_BO_HOST_MEM)

struct drm___dev {
	struct drm_device       *ddev;
	struct fpga_manager     *fpga_mgr;
	struct __ert_dev     *ert;
	struct iommu_domain	*domain;
	phys_addr_t              host_mem;
	resource_size_t          host_mem_len;
	/* Record start address, this is only for MPSoC as PCIe platform */
	phys_addr_t		 res_start;
	unsigned int		 cu_num;
	unsigned int             irq[MAX_CU_NUM];
	struct sched_exec_core  *exec;
	/* _ driver memory list head */
	struct list_head	 zm_list_head;
	struct drm_mm           *zm_drm_mm;    /* DRM MM node for PL-DDR */
	struct mutex		 mm_lock;
	struct mutex		 aie_lock;

	struct list_head	 ctx_list;

	struct addr_aperture	*apertures;
	unsigned int		 num_apts;

	struct kds_sched	 kds;
	struct platform_device	*cu_pldev[MAX_CU_NUM];

	/*
	 * This RW lock is to protect the sysfs nodes exported
	 * by _ driver. Currently, all _ attributes exported
	 * to sysfs nodes are protected by a single lock. Any read
	 * functions which not atomically touch those attributes should
	 * hold read lock; And all write functions which not atomically
	 * touch those attributes should hold write lock.
	 */
	rwlock_t		attr_rwlock;

	struct soft_krnl	*soft_kernel;
	struct aie_info		*aie_information;
	struct dma_chan		*zdev_dma_chan;
	struct mailbox		*zdev_mailbox;
	const struct zdev_data	*zdev_data_info;
	struct generic_cu	*generic_cu;
	struct __error	 zdev_error;
	struct __aie		*aie;
	struct __watchdog_dev *watchdog;

	int			 num_pr_slot;
	int			 full_overlay_id;
	struct drm___slot	*pr_slot[MAX_PR_SLOT_NUM];
};

struct drm___dev* zdev = NULL;

static inline struct
drm___bo *to___bo(struct drm_gem_object *bo)
{
	return (struct drm___bo *) bo;
}

void __describe(const struct drm___bo *obj)
{
	size_t size_in_kb = obj->cma_base.base.size / 1024;
	size_t physical_addr = obj->cma_base.paddr;

	DRM_DEBUG("%px: H[0x%zxKB] D[0x%zx]\n",
			obj,
			size_in_kb,
			physical_addr);
}

void __free_userptr_bo(struct drm_gem_object *gem_obj)
{
	/* Do all drm_gem_cma_free_object(bo->base) do, execpt free vaddr */
	struct drm___bo *__bo = to___bo(gem_obj);

	DRM_DEBUG("%s: obj 0x%px", __func__, __bo);
	if (__bo->cma_base.sgt)
		sg_free_table(__bo->cma_base.sgt);

	drm_gem_object_release(gem_obj);

	kfree(&__bo->cma_base);
}

void __free_host_bo(struct drm_gem_object *gem_obj)
{
	struct drm___bo *__bo = to___bo(gem_obj);

	DRM_DEBUG("%s: obj 0x%px", __func__, __bo);

	memunmap(__bo->cma_base.vaddr);

	drm_gem_object_release(gem_obj);

	kfree(&__bo->cma_base);
}
struct drm___mm_stat {
	size_t memory_usage;
	unsigned int bo_count;
};

struct addr_aperture {
	phys_addr_t	addr;
	size_t		size;
	u32		prop;
	int		cu_idx;
	u32		slot_idx;
};

enum __mem_type {
	__MEM_TYPE_CMA		= 0,
	__MEM_TYPE_RANGE_ALLOC	= 1,
	__MEM_TYPE_STREAMING		= 2,
};

/*
 * Memory structure in _ driver. There will be an array of this
 * structure where each element is representing each section in
 * the memory topology in xclbin.
 */
struct __mem {
	u32			zm_mem_idx;
	enum __mem_type	zm_type;
	unsigned int		zm_used;
	u64			zm_base_addr;
	u64			zm_size;
	struct drm___mm_stat zm_stat;
	struct list_head	link;
	struct list_head        zm_list;
};


static struct __mem *
__get_mem_by_mem_index(struct drm___dev *zdev, u32 mem_index)
{
	struct __mem *curr_mem = NULL;
	list_for_each_entry(curr_mem, &zdev->zm_list_head, link)
	    printk("seems good112");

		if (curr_mem->zm_mem_idx == mem_index)
			return curr_mem;

	return NULL;
}

void __update_mem_stat(struct drm___dev *zdev, u64 size, int count,
		uint32_t index)
{
	struct __mem *mem = __get_mem_by_mem_index(zdev, index);
	if (!mem)
		return;

	/*
	 * If the 'bank' passed in is a valid bank and its type is
	 * PL-DDR or LPDDR , we update that bank usage. Otherwise, we go
	 * through our bank list and find the CMA bank to update
	 * its usage.
	 */
	if (mem->zm_type != __MEM_TYPE_RANGE_ALLOC) {
		struct __mem *curr_mem = NULL;
		list_for_each_entry(curr_mem, &zdev->zm_list_head, link) {
			if (curr_mem->zm_used &&
			    curr_mem->zm_type == __MEM_TYPE_CMA) {
				mem = curr_mem;
				break;
			}
		}
	}

	write_lock(&zdev->attr_rwlock);
	mem->zm_stat.memory_usage +=
	    (count > 0) ?  size : -size;
	mem->zm_stat.bo_count += count;
	write_unlock(&zdev->attr_rwlock);
}



int __iommu_unmap_bo(struct drm_device *dev, struct drm___bo *bo)
{
	struct drm___dev *zdev = dev->dev_private;
	/* If IOMMU map had failed before bo->uaddr will be zero */
	if (bo->uaddr)
		iommu_unmap(zdev->domain, bo->uaddr, bo->gem_base.size);
	return 0;
}

static inline bool
__bo_userptr(const struct drm___bo *bo)
{
	return (bo->flags & __BO_FLAGS_USERPTR);
}

void __free_bo(struct drm_gem_object *obj)
{
	struct drm___bo *__obj;
	struct drm___dev *zdev;
	int npages;

	if (IS_ERR(obj) || !obj)
		return;

	DRM_DEBUG("Freeing BO\n");
	__obj = to___bo(obj);
	zdev = obj->dev->dev_private;

	if (!zdev->domain) {
		__describe(__obj);
		if (__obj->flags & __BO_FLAGS_USERPTR)
			__free_userptr_bo(obj);
		else if (__obj->flags & __BO_FLAGS_HOST_BO)
			__free_host_bo(obj);
		else if (__obj->flags & __BO_FLAGS_CMA) {
			/* Update memory usage statistics */
			__update_mem_stat(zdev, obj->size, -1,
			    __obj->mem_index);
			/* free resources associated with a CMA GEM object */
			drm_gem_cma_free_object(obj);

		} else {
			if (__obj->mm_node) {
				mutex_lock(&zdev->mm_lock);
				drm_mm_remove_node(__obj->mm_node);
				mutex_unlock(&zdev->mm_lock);
				kfree(__obj->mm_node);
				if (__obj->vmapping) {
					memunmap(__obj->vmapping);
					__obj->vmapping = NULL;
				}
				__update_mem_stat(zdev, obj->size, -1,
				    __obj->mem_index);
			}
			/* release GEM buffer object resources */
			drm_gem_object_release(obj);
			kfree(__obj);
		}

		return;
	}

	npages = obj->size >> PAGE_SHIFT;
	/* release GEM buffer object resources */
	drm_gem_object_release(obj);

	if (__obj->vmapping)
		vunmap(__obj->vmapping);
	__obj->vmapping = NULL;

	__iommu_unmap_bo(obj->dev, __obj);
	if (__obj->pages) {
		if (__bo_userptr(__obj)) {
			release_pages(__obj->pages, npages);
			kvfree(__obj->pages);
		} else {
			drm_gem_put_pages(obj, __obj->pages, false, false);

			/* Update memory usage statistics */
			__update_mem_stat(zdev, obj->size, -1,
			    __obj->mem_index);
		}
	}
	if (__obj->sgt)
		sg_free_table(__obj->sgt);
	__obj->sgt = NULL;
	__obj->pages = NULL;
	kfree(__obj);
}

static vm_fault_t __bo_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct drm_gem_object *obj = vma->vm_private_data;
	struct drm___bo *bo = to___bo(obj);
	struct drm___dev *zdev = obj->dev->dev_private;
	struct page *page;
	pgoff_t offset;
	int err;

	if (!zdev->domain)
		return 0;

	if (!bo->pages)
		return VM_FAULT_SIGBUS;

	offset = ((unsigned long)vmf->address - vma->vm_start) >> PAGE_SHIFT;
	page = bo->pages[offset];

	err = vm_insert_page(vma, (unsigned long)vmf->address, page);
	switch (err) {
	case -EAGAIN:
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
	case -EBUSY:
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	}
	return VM_FAULT_SIGBUS;
}

const struct vm_operations_struct __bo_vm_ops = {
	.fault = __bo_fault,
	.open  = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

const struct drm_gem_object_funcs __gem_object_funcs = {
	.free = __free_bo,
	.vm_ops = &__bo_vm_ops,
	.get_sg_table = drm_gem_cma_get_sg_table,
	.vmap = drm_gem_cma_vmap,
	.export = drm_gem_prime_export,
};

static struct drm___bo *
__create_cma_mem(struct drm_device *dev, size_t size)
{
	struct drm_gem_cma_object *cma_obj;
	struct drm___bo *bo;

	/* Allocate from CMA buffer */
	cma_obj = drm_gem_cma_create(dev, size);
	if (IS_ERR(cma_obj))
		return ERR_PTR(-ENOMEM);

  printk("cma obj is null %d",cma_obj == NULL);

	bo = to___bo(&cma_obj->base);

	return bo;
}

#define	GET_MEM_INDEX(x)	((x) & 0xFFFF)

static struct drm___bo *
__create_range_mem(struct drm_device *dev, size_t size, struct __mem *mem)
{
	struct drm___dev *zdev = dev->dev_private;
	struct drm___bo *bo = NULL;
	struct __mem *head_mem = mem;
	int err = -ENOMEM;

	bo = kzalloc(sizeof(struct drm___bo), GFP_KERNEL);
	if (IS_ERR(bo))
		return ERR_PTR(-ENOMEM);

	bo->gem_base.funcs = &__gem_object_funcs;
	err = drm_gem_object_init(dev, &bo->gem_base, size);
	if (err) {
		kfree(bo);
		return ERR_PTR(err);
	}

	bo->mm_node = kzalloc(sizeof(struct drm_mm_node),
			GFP_KERNEL);
	if (IS_ERR(bo->mm_node)) {
		drm_gem_object_release(&bo->gem_base);
		kfree(bo);
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&zdev->mm_lock);
	do {
		if (mem->zm_type == __MEM_TYPE_CMA) {
			struct drm___bo *cma_bo =
				__create_cma_mem(dev, size);
			if (!IS_ERR(cma_bo)) {
				/* Get the memory from CMA memory region */
				mutex_unlock(&zdev->mm_lock);
				kfree(bo->mm_node);
				drm_gem_object_release(&bo->gem_base);
				kfree(bo);
				cma_bo->flags |= __BO_FLAGS_CMA;
				return cma_bo;
			}
			DRM_WARN("Memory allocated from CMA region"
				" whereas requested for reserved memory region\n");
		}
		else {
			err = drm_mm_insert_node_in_range(zdev->zm_drm_mm,
				bo->mm_node, size, PAGE_SIZE, 0,
				mem->zm_base_addr,
				mem->zm_base_addr + mem->zm_size, 0);
			if (!err) {
				/* Got memory from this Range memory manager */
				break;
			}
		}

		/* No memory left to this memory manager.
		 * Try to allocate from similer memory manger link list
		 */
		mem = list_entry(mem->zm_list.next, typeof(*mem), zm_list);

	} while (&mem->zm_list != &head_mem->zm_list);

	if (err) {
		DRM_ERROR("Fail to allocate BO: size %ld\n",
				(long)size);
		mutex_unlock(&zdev->mm_lock);
		kfree(bo->mm_node);
		drm_gem_object_release(&bo->gem_base);
		kfree(bo);
		return ERR_PTR(-ENOMEM);
	}

	mutex_unlock(&zdev->mm_lock);

	/*
	 * Set up a kernel mapping for direct BO access.
	 * We don't have to fail BO allocation if we can
	 * not establish the kernel mapping. We just can not
	 * access BO directly from kernel.
	 */
	bo->vmapping = memremap(bo->mm_node->start, size, MEMREMAP_WC);

	err = drm_gem_create_mmap_offset(&bo->gem_base);
	if (err) {
		DRM_ERROR("Fail to create BO mmap offset.\n");
		__free_bo(&bo->gem_base);
		return ERR_PTR(err);
	}

	return bo;
}

static struct drm___bo *
__create_bo(struct drm_device *dev, uint64_t unaligned_size, u32 user_flags)
{
  printk("drm _ create bo");
	size_t size = PAGE_ALIGN(unaligned_size);
	struct drm___bo *bo = NULL;
	int err = 0;

	if (!size)
		return ERR_PTR(-EINVAL);

	if (zdev->domain) {
		bo = kzalloc(sizeof(*bo), GFP_KERNEL);
		if (!bo)
			return ERR_PTR(-ENOMEM);

		bo->gem_base.funcs = &__gem_object_funcs;
		err = drm_gem_object_init(dev, &bo->gem_base, size);
//		if (err < 0)
//			goto free;
	} else if (user_flags & __BO_FLAGS_CMA) {
		bo = __create_cma_mem(dev, size);
	} else {
		/* We are allocating from a separate mem Index, i.e. PL-DDR or LPDDR */
		unsigned int mem_index = GET_MEM_INDEX(user_flags);
		struct __mem *mem = __get_mem_by_mem_index(zdev, mem_index);
		if (mem == NULL)
			return ERR_PTR(-ENOMEM);

		if (!mem->zm_used || mem->zm_type != __MEM_TYPE_RANGE_ALLOC)
			return ERR_PTR(-EINVAL);

		//bo = __create_range_mem(dev, size, mem);
	}
  printk("all ok");
	//if (user_flags & __BO_FLAGS_EXECBUF) {
	//	bo->flags = __BO_FLAGS_EXECBUF;
	//	bo->metadata.state = DRM___EXECBUF_STATE_ABORT;
	//}

	return bo;
//free:
//	kfree(bo);
//	return ERR_PTR(err);
}

#define __DRM_GEM_OBJECT_PUT_UNLOCKED drm_gem_object_put


static struct drm___bo *
__create_svm_bo(struct drm_device *dev, void *data, struct drm_file *filp)
{
	struct drm___create_bo *args = data;
	struct drm___bo *bo = NULL;
	size_t bo_size = 0;
	int ret = 0;

	printk("__create_svm_bo");

	if ((args->flags & __BO_FLAGS_COHERENT) ||
			(args->flags & __BO_FLAGS_CMA))
		return ERR_PTR(-EINVAL);

	args->flags |= __BO_FLAGS_SVM;
	if (!(args->flags & __BO_FLAGS_SVM))
		return ERR_PTR(-EINVAL);

	bo = __create_bo(dev, args->size, args->flags);

  printk("hello");
	printk("%d", bo == NULL);
	bo->flags |= __BO_FLAGS_SVM;
	bo->mem_index = GET_MEM_INDEX(args->flags);

	if (IS_ERR(bo)) {
		DRM_DEBUG("object creation failed\n");
		return bo;
	}
	bo->pages = drm_gem_get_pages(&bo->gem_base);
	if (IS_ERR(bo->pages)) {
		ret = PTR_ERR(bo->pages);
		goto out_free;
	}

	bo_size = bo->gem_base.size;
	bo->sgt = drm_prime_pages_to_sg(dev, bo->pages, bo_size >> PAGE_SHIFT);
	if (IS_ERR(bo->sgt))
		goto out_free;

	bo->vmapping = vmap(bo->pages, bo->gem_base.size >> PAGE_SHIFT, VM_MAP,
			pgprot_writecombine(PAGE_KERNEL));

	if (!bo->vmapping) {
		ret = -ENOMEM;
		goto out_free;
	}

	ret = drm_gem_create_mmap_offset(&bo->gem_base);
	if (ret < 0)
		goto out_free;

	ret = drm_gem_handle_create(filp, &bo->gem_base, &args->handle);
	if (ret < 0)
		goto out_free;

	__describe(bo);
	__DRM_GEM_OBJECT_PUT_UNLOCKED(&bo->gem_base);

	/* Update memory usage statistics */
	__update_mem_stat(dev->dev_private, args->size, 1, bo->mem_index);

	return bo;

out_free:
	__free_bo(&bo->gem_base);
	return ERR_PTR(ret);
}













static inline uint32_t __convert_bo_uflags(uint32_t uflags)
{
	uint32_t zflags = 0;

	/*
	 * Keep the bank index and remove all flags, except EXECBUF and
	 * CACHEABLE.
	 */
	if (uflags & XCL_BO_FLAGS_EXECBUF)
		zflags |= __BO_FLAGS_EXECBUF;

	if (uflags & XCL_BO_FLAGS_CACHEABLE)
		zflags |= __BO_FLAGS_CACHEABLE;

	zflags |= (uflags & 0xFFFF);

	return zflags;
}

int __create_bo_ioctl(struct drm_device *dev, void *data, struct drm_file *filp)
{
	printk("CALLED IOCTL");
	int ret = 0;
  struct drm___create_bo *args = data;
  struct drm___bo *bo = NULL;
  struct __mem *mem = NULL;
  unsigned int mem_index = 0;
  uint32_t user_flags = args->flags;

  printk("[drm]  create bo");

  args->flags = __convert_bo_uflags(args->flags);
  if (zdev->domain) {
    printk  ("drm _ converted bo uflags2");

  	bo = __create_svm_bo(dev, data, filp);
  	  printk("drm _ converted bo uflags3");

  	if (IS_ERR(bo))
  		return PTR_ERR(bo);
  	bo->user_flags = user_flags;
  	return 0;
  }

  printk("drm _ converted bo uflags4");


  mem_index = GET_MEM_INDEX(args->flags);
  return 0;

  //mem = __get_mem_by_mem_index(zdev, mem_index);

  	/* Always allocate EXECBUF from CMA */
  if (args->flags & __BO_FLAGS_EXECBUF)
  	args->flags |= __BO_FLAGS_CMA;
  else {
  	/*
  	 * For specified valid DDR bank, we only mark CMA flags
  	 * if the bank type is CMA, non-CMA type bank will use
  	 * PL-DDR; For any other cases (invalid bank index), we
  	 * allocate from CMA by default.
  	 */
  	if (mem && mem->zm_used) {
  		if (mem->zm_type == __MEM_TYPE_CMA)
  			args->flags |= __BO_FLAGS_CMA;
  	} else {
  		//DRM_WARN("Allocating BO from CMA for invalid or unused memory index[%d]\n",
  				//mem_index);
  		args->flags |= __BO_FLAGS_CMA;
  	}
  }

  if (!(args->flags & __BO_FLAGS_CACHEABLE)) {
  	/* If cacheable is not set, make sure we set COHERENT. */
  	args->flags |= __BO_FLAGS_COHERENT;
  } else if (!(args->flags & __BO_FLAGS_CMA)) {
  	/*
  	 * We do not support allocating cacheable BO from PL-DDR or
  	 * LPDDR
  	 */
  	DRM_WARN("Cache is not supported and turned off for PL-DDR or LPDDR\n");
  	args->flags &= ~__BO_FLAGS_CACHEABLE;
  }

  bo = __create_bo(dev, args->size, args->flags);


  if (IS_ERR(bo)) {
  	DRM_DEBUG("object creation failed\n");
  	return PTR_ERR(bo);
  }

  bo->mem_index = mem_index;
  if (args->flags & __BO_FLAGS_CACHEABLE)
  	bo->flags |= __BO_FLAGS_CACHEABLE;
  else
  	bo->flags |= __BO_FLAGS_COHERENT;

  if (args->flags & __BO_FLAGS_CMA) {
  	bo->flags |= __BO_FLAGS_CMA;
  	ret = drm_gem_handle_create(filp, &bo->cma_base.base,
  	    &args->handle);
  	if (ret) {
  		drm_gem_cma_free_object(&bo->cma_base.base);
  		DRM_DEBUG("handle creation failed\n");
  		return ret;
  	}
  } else {
  	ret = drm_gem_handle_create(filp, &bo->gem_base,
  	    &args->handle);
  	if (ret) {
  		__free_bo(&bo->gem_base);
  		DRM_DEBUG("handle create failed\n");
  		return ret;
  	}
  }

  bo->user_flags = user_flags;
  __describe(bo);
  __DRM_GEM_OBJECT_PUT_UNLOCKED(&bo->cma_base.base);

  /*
   * Update memory usage statistics.
   *
   * Note: We can not use args->size here because it is
   *       the required size while gem object records the
   *       actual size allocated.
   */
  __update_mem_stat(zdev, bo->gem_base.size, 1, bo->mem_index);

  return 0;
}




static const struct drm_ioctl_desc __ioctls[] = {
	DRM_IOCTL_DEF_DRV(__CREATE_BO, __create_bo_ioctl,
			DRM_AUTH|DRM_UNLOCKED|DRM_RENDER_ALLOW)
};

struct drm_gem_object *
__gem_create_object(struct drm_device *dev, size_t size)
{
	struct drm___bo *bo = kzalloc(sizeof(struct drm___bo), GFP_KERNEL);

	return (&bo->gem_base);
}

static const struct file_operations __driver_fops = {
	.open           = drm_open,
	.read           = drm_read,
	.unlocked_ioctl = drm_ioctl,
	.release        = drm_release,
};


static struct drm_driver mydrm = {

	.driver_features           = DRIVER_GEM | DRIVER_RENDER,


	.gem_create_object         = __gem_create_object,

	.ioctls                    = __ioctls,
	.num_ioctls                = 1,
	.fops                      = &__driver_fops,

};




static int my_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	 zdev = devm_kzalloc(&pdev->dev, sizeof(*zdev), GFP_KERNEL);
   struct drm_device *drm;
      printk("===== DRM PCI PROBE =====");

      drm = drm_dev_alloc(&mydrm, &pdev->dev);


      int res2 = drm_dev_register(drm, 0);

      if(res2 == 0){
          printk("----- SUCCESS -----");
      } else {
          printk("----- ERROR -----");
      }

      drm->dev_private = zdev;
      zdev->ddev = drm;



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
