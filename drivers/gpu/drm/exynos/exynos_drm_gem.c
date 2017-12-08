#include <linux/ion.h>
#include <linux/exynos_iovmm.h>
#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_gem.h"

extern struct ion_device *ion_exynos;

int exynos_drm_gem_init(struct drm_device *dev)
{
	struct exynos_drm_private *priv = dev->dev_private;

	priv->gem_priv = kzalloc(sizeof(*priv->gem_priv), GFP_KERNEL);
	if (!priv->gem_priv)
		return -ENOMEM;

	priv->gem_priv->ion_client = ion_client_create(ion_exynos, "exynosgem");
	if (IS_ERR(priv->gem_priv->ion_client)) {
		int ret = PTR_ERR(priv->gem_priv->ion_client);

		DRM_ERROR("Failed to create ION client: %d\n", ret);
		kfree(priv->gem_priv);
		return ret;
	}

	return 0;
}

void exynos_drm_gem_deinit(struct drm_device *dev)
{
	struct exynos_drm_private *priv = dev->dev_private;

	ion_client_destroy(priv->gem_priv->ion_client);
	kfree(priv->gem_priv);
}

int exynos_drm_gem_init_iommu(struct drm_device *dev, struct device *client)
{
	struct exynos_drm_private *devpriv = dev->dev_private;
	struct exynos_drm_gem_private *gempriv = devpriv->gem_priv;
	int ret;

	if (!gempriv) {
		DRM_ERROR("exynos_drm_gem_init() should be called before %s\n",
			  __func__);
		return -EINVAL;
	}

	ret = iovmm_activate(client);
	if (ret) {
		DRM_ERROR("Failed to activate IOVMM of %s", dev_name(client));
		return ret;
	}

	gempriv->iommu_client = client;
	DRM_DEBUG_DRIVER("assigned %s for the default iommu client device\n",
			dev_name(client));

	return 0;
}

void exynos_drm_gem_deinit_iommu(struct drm_device *dev, struct device *client)
{
	struct exynos_drm_private *devpriv = dev->dev_private;
	struct exynos_drm_gem_private *gempriv = devpriv->gem_priv;

	if (client != gempriv->iommu_client) {
		DRM_ERROR("IOMMU client of GEM is %s but deinit of %s tried\n",
			  dev_name(gempriv->iommu_client), dev_name(client));
		return;
	}

	iovmm_deactivate(client);

	gempriv->iommu_client = NULL;
}

static int check_gem_flags(unsigned int flags)
{
	if (flags & ~(EXYNOS_BO_MASK)) {
		DRM_ERROR("Invalid flags %#x\n", flags);
		return -EINVAL;
	}

	return 0;
}

static int exynos_drm_gem_map(struct exynos_drm_gem_obj *exynos_gem_obj,
			      struct device *client)
{
	int ret;

	exynos_gem_obj->attachment = dma_buf_attach(exynos_gem_obj->dmabuf,
						    client);
	if (IS_ERR(exynos_gem_obj->attachment)) {
		DRM_ERROR("Failed to attach dmabuf to %s: %ld\n",
			dev_name(client), PTR_ERR(exynos_gem_obj->attachment));
		return PTR_ERR(exynos_gem_obj->attachment);
	}

	exynos_gem_obj->sgt = dma_buf_map_attachment(exynos_gem_obj->attachment,
						     DMA_TO_DEVICE);
	if (IS_ERR(exynos_gem_obj->sgt)) {
		ret = PTR_ERR(exynos_gem_obj->sgt);
		DRM_ERROR("Failed to map attachment of %s: %d\n",
			  dev_name(client), ret);
		goto err_map;
	}

	if (client) {
		exynos_gem_obj->dma_addr = iovmm_map(client,
					exynos_gem_obj->sgt->sgl, 0,
					exynos_gem_obj->size,
					DMA_TO_DEVICE, IOMMU_READ);
		if (IS_ERR_VALUE(exynos_gem_obj->dma_addr)) {
			DRM_ERROR("Failed to allocate IOVM of %s\n",
				  dev_name(client));
			ret = (int)exynos_gem_obj->dma_addr;
			goto err_dmaaddr;
		}
	} else if (exynos_gem_obj->sgt->orig_nents != 1) {
		DRM_ERROR("Scattered buffer is unavailable without IOMMU\n");
		ret = -EINVAL;
		goto err_dmaaddr;
	} else {
		exynos_gem_obj->dma_addr = sg_phys(exynos_gem_obj->sgt->sgl);
		DRM_DEBUG_DRIVER("no client device for IOMMU exists\n");
	}

	DRM_DEBUG_DRIVER("dma_addr(0x%llx)\n", exynos_gem_obj->dma_addr);

	return 0;

err_dmaaddr:
	dma_buf_unmap_attachment(exynos_gem_obj->attachment,
				 exynos_gem_obj->sgt, DMA_TO_DEVICE);
err_map:
	dma_buf_detach(exynos_gem_obj->dmabuf, exynos_gem_obj->attachment);

	return ret;
}

static void exynos_drm_gem_unmap(struct exynos_drm_gem_obj *exynos_gem_obj,
				 struct device *client)
{
	if (client)
		iovmm_unmap(client, exynos_gem_obj->dma_addr);

	dma_buf_unmap_attachment(exynos_gem_obj->attachment,
				 exynos_gem_obj->sgt, DMA_TO_DEVICE);
	dma_buf_detach(exynos_gem_obj->dmabuf, exynos_gem_obj->attachment);

	DRM_DEBUG_DRIVER("unmapped the dma address and the attachment\n");
}

struct exynos_drm_gem_obj *exynos_drm_gem_alloc(struct drm_device *dev,
						size_t size, unsigned int flags)
{
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct exynos_drm_private *devpriv = dev->dev_private;
	struct exynos_drm_gem_private *gempriv = devpriv->gem_priv;
	unsigned int ion_flags = 0;
	unsigned int heapmask = ION_HEAP_SYSTEM_MASK;
	int ret;

	ret = check_gem_flags(flags);
	if (ret)
		return ERR_PTR(ret);

	exynos_gem_obj = kzalloc(sizeof(*exynos_gem_obj), GFP_KERNEL);
	if (!exynos_gem_obj)
		return ERR_PTR(-ENOMEM);

	exynos_gem_obj->size = size;
	exynos_gem_obj->flags = flags;

	/* no need to release initialized private gem object */
	drm_gem_private_object_init(dev, &exynos_gem_obj->base, size);

	if (!!(flags & EXYNOS_BO_CACHABLE))
		ion_flags = ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC;

	if (!(flags & EXYNOS_BO_NONCONTIG))
		heapmask = ION_HEAP_CARVEOUT_MASK;

	exynos_gem_obj->handle = ion_alloc(gempriv->ion_client, size,
				PAGE_SIZE, heapmask, ion_flags);
	if (IS_ERR(exynos_gem_obj->handle)) {
		DRM_ERROR("ION Failed to alloc %zu bytes with flags %#x/%#x\n",
				size, heapmask, ion_flags);
		ret = PTR_ERR(exynos_gem_obj->handle);
		goto err_alloc;
	}

	exynos_gem_obj->dmabuf = ion_share_dma_buf(gempriv->ion_client,
						   exynos_gem_obj->handle);
	if (IS_ERR(exynos_gem_obj->dmabuf)) {
		ret = PTR_ERR(exynos_gem_obj->dmabuf);
		DRM_ERROR("ION failed to export ION handle to dmabuf: %d\n",
				ret);
		goto err_export;
	}

	ret = exynos_drm_gem_map(exynos_gem_obj, gempriv->iommu_client);
	if (ret)
		goto err_map;

	DRM_DEBUG_DRIVER("allocated %zu bytes with flags %#x\n", size, flags);

	return exynos_gem_obj;
err_map:
	dma_buf_put(exynos_gem_obj->dmabuf);
err_export:
	ion_free(gempriv->ion_client, exynos_gem_obj->handle);
err_alloc:
	kfree(exynos_gem_obj);

	return ERR_PTR(ret);
}

static int exynos_drm_gem_create(struct drm_device *dev, struct drm_file *filep,
				size_t size, unsigned int flags,
				unsigned int *gem_handle)
{
	struct exynos_drm_gem_obj *exynos_gem_obj;
	int ret;

	exynos_gem_obj = exynos_drm_gem_alloc(dev, size, flags);
	if (IS_ERR(exynos_gem_obj))
		return PTR_ERR(exynos_gem_obj);

	ret = drm_gem_handle_create(filep, &exynos_gem_obj->base, gem_handle);
	if (ret) {
		DRM_ERROR("Failed to create a handle of GEM\n");
		exynos_drm_gem_free_object(&exynos_gem_obj->base);
		return ret;
	}

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(&exynos_gem_obj->base);

	return 0;
}

int exynos_drm_gem_create_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct drm_exynos_gem_create *args = data;
	size_t size = PAGE_ALIGN(args->size);
	unsigned int flags = args->flags;
	unsigned int handle;
	int ret;

	ret = exynos_drm_gem_create(dev, file_priv, size, flags, &handle);
	if (ret) {
		DRM_ERROR("Failed to create GEM buffer of %zu bytes\n", size);
		return ret;
	}

	args->handle = handle;

	DRM_DEBUG_DRIVER("created a gem object: handle %u size %zu\n",
			handle, size);

	return 0;
}

int exynos_drm_gem_get_ioctl(struct drm_device *dev, void *data,
				      struct drm_file *file_priv)
{
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct drm_exynos_gem_info *args = data;
	struct drm_gem_object *obj;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		DRM_ERROR("Failed to lookup gem object from handle %u.\n",
				args->handle);
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	exynos_gem_obj = to_exynos_gem_obj(obj);

	args->flags = exynos_gem_obj->flags;
	args->size = exynos_gem_obj->size;

	drm_gem_object_unreference(obj);

	mutex_unlock(&dev->struct_mutex);

	DRM_DEBUG_DRIVER("obtained info of handle %u: flags %#x size %llu\n",
			args->handle, args->flags, args->size);

	return 0;
}

static int exynos_drm_gem_offset(struct drm_device *dev, struct drm_file *filep,
				 unsigned int handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, filep, handle);
	if (!obj) {
		DRM_ERROR("Failed to lookup gem object from handle %u.\n",
				handle);
		ret = -EINVAL;
		goto unlock;
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret) {
		DRM_ERROR("Failed to create mmap fake offset for handle %u\n",
			handle);
		goto out;
	}

	*offset = drm_vma_node_offset_addr(&obj->vma_node);
out:
	drm_gem_object_unreference(obj);
unlock:
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

int exynos_drm_gem_offset_ioctl(struct drm_device *dev, void *data,
				      struct drm_file *file_priv)
{
	struct drm_exynos_gem_mmap_offset *args = data;
	int ret;

	ret = exynos_drm_gem_offset(dev, file_priv,
				  args->handle, &args->offset);
	if (ret)
		return ret;

	DRM_DEBUG_DRIVER("obtained fake mmap offset %llu of handle %u\n",
			args->offset, args->handle);
	return 0;
}

int exynos_drm_gem_mmap_ioctl(struct drm_device *dev, void *data,
				     struct drm_file *file_priv)
{
	struct drm_exynos_gem_mmap *args = data;
	struct drm_gem_object *obj;
	unsigned long addr;
	int ret = 0;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		DRM_ERROR("Failed to lookup gem object.\n");
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	ret = drm_gem_create_mmap_offset(obj);
	mutex_unlock(&dev->struct_mutex);
	if (ret) {
		DRM_ERROR("Failed to create mmap fake offset for handle %u\n",
			args->handle);
		goto out;
	}

	addr = vm_mmap(file_priv->filp, args->addr, args->size,
		args->prot, args->flag, drm_vma_node_offset_addr(&obj->vma_node));
	if (IS_ERR_VALUE(addr)) {
		DRM_ERROR("Failed to mmap the given handle %u of obj %p\n",
				args->handle, obj);
		ret = (int)addr;
		goto out;
	}

	args->addr = addr;

	DRM_DEBUG_DRIVER("ioctl-mmaped the handle %u of size %llu to %#lx\n",
			args->handle, args->size, addr);
out:
	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

void exynos_drm_gem_free_object(struct drm_gem_object *obj)
{
	struct exynos_drm_gem_obj *exynos_gem_obj = to_exynos_gem_obj(obj);
	struct exynos_drm_private *priv = exynos_gem_obj->base.dev->dev_private;
	struct exynos_drm_gem_private *gempriv = priv->gem_priv;

	exynos_drm_gem_unmap(exynos_gem_obj, gempriv->iommu_client);

	if (exynos_gem_obj->kaddr) {
		dma_buf_kunmap(exynos_gem_obj->dmabuf, 0, exynos_gem_obj->kaddr);
		dma_buf_end_cpu_access(exynos_gem_obj->dmabuf, 0,
				       exynos_gem_obj->size, DMA_TO_DEVICE);
	}

	if (exynos_gem_obj->dmabuf) {
		DRM_DEBUG("destroying imported gem object of size %lu\n",
				exynos_gem_obj->size);
		dma_buf_put(exynos_gem_obj->dmabuf);
	}

	if (exynos_gem_obj->handle) {
		DRM_DEBUG("destroying genuine gem object of size %lu\n",
				exynos_gem_obj->size);
		ion_free(gempriv->ion_client, exynos_gem_obj->handle);
	}

	if (obj->import_attach)
		drm_prime_gem_destroy(obj, NULL);

	drm_gem_object_release(&exynos_gem_obj->base);
	kfree(exynos_gem_obj);
}

int exynos_drm_gem_mmap_object(struct exynos_drm_gem_obj *exynos_gem_obj,
			       struct vm_area_struct *vma)
{
	struct exynos_drm_private *priv = exynos_gem_obj->base.dev->dev_private;
	struct exynos_drm_gem_private *gempriv = priv->gem_priv;
	unsigned long addr = vma->vm_start;
	struct sg_table *sgt;
	struct scatterlist *sg;
	int i, ret;

	if (exynos_gem_obj->dmabuf) {
		ret = dma_buf_mmap(exynos_gem_obj->dmabuf, vma, 0);
		if (ret) {
			DRM_ERROR("Failed to mmap imported buffer: %d\n", ret);
			return ret;
		}

		return 0;
	}

	/* Correct vm_page_prot to the attribute of buffer */
	if (exynos_gem_obj->flags & EXYNOS_BO_CACHABLE)
		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);

	/* map all pages here */
	sgt = ion_sg_table(gempriv->ion_client, exynos_gem_obj->handle);
	if (IS_ERR(sgt)) {
		DRM_ERROR("Failed to get sg_table: %ld\n", PTR_ERR(sgt));
		return PTR_ERR(sgt);
	}

	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		unsigned long remainder = vma->vm_end - addr;
		unsigned long len;

		len = min_t(unsigned long, sg->length, remainder);

		ret = remap_pfn_range(vma, addr, page_to_pfn(sg_page(sg)), len,
				vma->vm_page_prot);
		if (ret) {
			DRM_ERROR("Failed to map %lu@%#lx with pfn %lx: %d\n",
				  addr, page_to_pfn(sg_page(sg)), len, ret);
			return ret;
		}

		addr += len;
		if (addr >= vma->vm_end)
			break;

		remainder -= len;
	}

	return 0;
}

int exynos_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	/* NOTE: drm_gem_mmap() always set the vm_page_prot to writecombine */
	ret = drm_gem_mmap(filp, vma);
	if (ret < 0)
		goto err;

	ret = exynos_drm_gem_mmap_object(
				to_exynos_gem_obj(vma->vm_private_data), vma);
	if (ret)
		goto err_mmap;

	DRM_DEBUG_DRIVER("mmaped the offset %lu of size %lu to %#lx\n",
		vma->vm_pgoff, vma->vm_end - vma->vm_start, vma->vm_start);

	return 0;
err_mmap:
		/* release the resources referenced by drm_gem_mmap() */
	drm_gem_vm_close(vma);
err:
	DRM_ERROR("Failed to mmap with offset %lu.\n", vma->vm_pgoff);

	return ret;
}

void * exynos_drm_gem_kaddr(struct exynos_drm_gem_obj *exynos_gem_obj)
{
	if (exynos_gem_obj->kaddr == NULL) {
		int ret;

		ret = dma_buf_begin_cpu_access(exynos_gem_obj->dmabuf,
					0, exynos_gem_obj->size, DMA_TO_DEVICE);
		if (ret) {
			DRM_ERROR("Failed to setup kernel address: %d\n", ret);
			return NULL;
		}

		exynos_gem_obj->kaddr = dma_buf_kmap(exynos_gem_obj->dmabuf, 0);
		if (IS_ERR_OR_NULL(exynos_gem_obj->kaddr)) {
			DRM_ERROR("Failed to obtain kernel address from bo\n");
			dma_buf_end_cpu_access(exynos_gem_obj->dmabuf, 0,
					       exynos_gem_obj->size, DMA_TO_DEVICE);
			exynos_gem_obj->kaddr = NULL;
		}
	}

	return exynos_gem_obj->kaddr;
}

static void exynos_dmabuf_prime_release(void *p)
{
	struct exynos_drm_gem_obj *exynos_gem_obj = to_exynos_gem_obj(p);

	DRM_DEBUG_PRIME("finally released an exported dmabuf with size %lu\n",
			exynos_gem_obj->size);

	drm_gem_object_unreference_unlocked(p);
}

struct dma_buf *exynos_dmabuf_prime_export(struct drm_device *drm_dev,
				struct drm_gem_object *obj, int flags)
{
	struct exynos_drm_gem_obj *exynos_gem_obj = to_exynos_gem_obj(obj);
	struct exynos_drm_private *priv = exynos_gem_obj->base.dev->dev_private;
	struct exynos_drm_gem_private *gempriv = priv->gem_priv;
	struct dma_buf *dmabuf;

	if (!exynos_gem_obj->handle) {
		DRM_ERROR("PRIME imported buffer never be exported again\n");
		return ERR_PTR(-EINVAL);
	}

	drm_gem_object_unreference_unlocked(&exynos_gem_obj->base);
	/*
	 * this expoerted dmabuf is just for prime buffer sharing. It is
	 * different from the exynos_gem_obj->dmabuf that is for the prime
	 * imported buffer or dma buffer management.
	 */
	dmabuf = ion_share_dma_buf_callback(gempriv->ion_client,
					    exynos_gem_obj->handle,
					    obj, exynos_dmabuf_prime_release);
	if (IS_ERR(dmabuf)) {
		DRM_ERROR("Failed to export dmabuf from gem object\n");
	} else {
		DRM_DEBUG_PRIME("exported a buffer with size %zu to dmabuf\n",
				dmabuf->size);
	}

	return dmabuf;
}

struct drm_gem_object *exynos_dmabuf_prime_import(struct drm_device *drm_dev,
				struct dma_buf *dmabuf)
{
	struct exynos_drm_gem_obj *exynos_gem_obj;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	struct exynos_drm_gem_private *gempriv = priv->gem_priv;
	int ret;

	exynos_gem_obj = kzalloc(sizeof(*exynos_gem_obj), GFP_KERNEL);
	if (!exynos_gem_obj)
		return ERR_PTR(-ENOMEM);

	drm_gem_private_object_init(drm_dev,
				&exynos_gem_obj->base, dmabuf->size);

	exynos_gem_obj->size = dmabuf->size;
	exynos_gem_obj->dmabuf = dmabuf;

	ret = exynos_drm_gem_map(exynos_gem_obj, gempriv->iommu_client);
	if (ret) {
		kfree(exynos_gem_obj);
		DRM_ERROR("Failed to import prime buffer\n");
		return ERR_PTR(ret);
	}

	/* FIXME: consider the flag correctly */
	exynos_gem_obj->flags = EXYNOS_BO_NONCONTIG;

	DRM_DEBUG_PRIME("imported dmabuf with size %zu bytes\n", dmabuf->size);

	return &exynos_gem_obj->base;
}

int exynos_drm_gem_dumb_create(struct drm_file *file_priv,
				      struct drm_device *dev,
				      struct drm_mode_create_dumb *args)
{
	unsigned int handle;
	int ret;

	args->pitch = args->width * DIV_ROUND_UP(args->bpp, 8);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	ret = exynos_drm_gem_create(dev, file_priv, args->size,
				    EXYNOS_BO_NONCONTIG, &handle);
	if (ret) {
		DRM_ERROR("Failed to create dumb of %llu bytes (%ux%u/%ubpp)\n",
			  args->size, args->width, args->height, args->bpp);
	} else {
		DRM_DEBUG_KMS("created a dumb: handle %u size %llu %ux%u/%u\n",
				handle, args->size,
				args->width, args->height, args->bpp);
		args->handle = handle;
	}

	return 0;
}

int exynos_drm_gem_dumb_map_offset(struct drm_file *file_priv,
					struct drm_device *dev, uint32_t handle,
					uint64_t *offset)
{
	int ret;

	ret = exynos_drm_gem_offset(dev, file_priv, handle, offset);
	if (!ret)
		DRM_DEBUG_KMS("obtained fake mmap offset %llu from handle %u\n",
				*offset, handle);
	return ret;
}
