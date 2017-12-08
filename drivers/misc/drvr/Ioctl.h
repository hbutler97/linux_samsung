 /* @file
 *
 * Define Data structures and ioctl commands to pass between the driver
 * and the user space.  The definitions serve as the interface between
 * the kernel and user.
 *
 * NOTE: This file is included in source code built for the kernel and user.
 * It must be kept compatible with both environments.
 * In particular, structure definitions can not be changed in one space without
 * recompiling the code in the other space.
 */
 
#ifndef SC_IOCTL_H
#define SC_IOCTL_H

#include <asm/ioctl.h>

#include "sysDefs.h"

#ifndef MAX_PCI_BARS
#define MAX_PCI_BARS 7    // do not count/use the expansion ROM
#endif

#define MAX_DRIVER_NAME_LEN 128
#define MAX_DRIVER_VERSION_LEN 128



typedef char DriverVerStr_t[MAX_DRIVER_NAME_LEN];



/**
 * Information about a device's BAR.
 */
typedef struct
{
	ULONG nBAR;
	ULONG physStartAddr;
	ULONG size;
	bool memMapped;
	USHORT flags;
	UCHAR type;

} PCI_BAR_t;


/**
 * Device Drvier specific information to return to user space.
 * This includes the BAR resources allocated to the device,
 * interrupts, etc.
 * NOTE: to keep backwards compatibility with older applications
 * that were built against v1.0.0.x drivers, do no add any new
 * fields to this structure or the driver will crash during
 * copying info back to user pages because the size will be different
 * and a fault will occur.  Add new fields to the extra info struct.
 */
typedef struct
{
	// Device Memory Access info
	ULONG numBARs;
	PCI_BAR_t BAR[MAX_PCI_BARS];

	UCHAR PCICfgReg[256];

	// Device Interrupt Info
	bool hasInterrupt;
	ULONG intrVector;

} PCIResourceInfo_t;


/**
 * Additional Device Driver specific information to return to user space.
 * This includes the DMA resources allocated to the device, interrupts, etc.
 * This is for the newer PCIeBasic and PCIeSFIF demos and applications.
 */
typedef struct
{
        // Instance and device location info
        ULONG devID;     /**< board number of specific device */

        ULONG  busNum;         /**< PCI bus number board located on */
        USHORT deviceNum;      /**< PCI device number assigned to board */
        USHORT functionNum;    /**< our function number, which is always 0 (no multi-function */
        ULONG   UINumber;      /**< motherboard slot number (not always implemented/valid) */

        // Device DMA Common buffer memory info
        bool hasDmaBuf;        /**< true if DMA buffer has been allocated by driver */
        ULONG DmaBufSize;      /**< size in bytes of said buffer */
        bool DmaAddr64;        /**< true if the address mode is 64 bit (almost always 32 bit) */
        ULONG DmaPhyAddrHi;    /**< Upper 32 bits of 64 bit address for 64 bit mode */
        ULONG DmaPhyAddrLo;    /**< DMA bus address to be programmed into device */


        char DriverName[MAX_DRIVER_NAME_LEN];   /**< version and name compiled into driver */


} ExtraResourceInfo_t;

struct load_img_args {
	int img_index;
	int buff_fd;
};

struct control_args {
	u32 cmd;
	u32 arg1;
	u32 arg2;
	u32 arg3;
};

/**
 * IOCTL Operations.
 * use these defines when performing an ioctl operation to a device.
 */

#define ADIPCIE_MAGIC 'L'

/** This IO_CTL is used to get PCI driver name and version
 * back to user space applications.
 */
#define IOCTL_ADIPCIE_GET_VERSION_INFO _IOR(ADIPCIE_MAGIC, 0, DriverVerStr_t)


/** This IO_CTL is used to get PCI and driver information from the
 * driver back to user space applications.
 */
#define IOCTL_ADIPCIE_GET_RESOURCES    _IOR(ADIPCIE_MAGIC, 1, PCIResourceInfo_t)

/**
 * This IO_CTL is used to set the BAR number that is mapped in with the MMAP command.
 */
#define IOCTL_ADIPCIE_SET_BAR     _IOW(ADIPCIE_MAGIC, 2, int)

/** This IO_CTL is used to get additional information from the
 * driver back to user space applications.  This info is used by the SFIF.
 */
#define IOCTL_ADIPCIE_GET_EXTRA_INFO     _IOR(ADIPCIE_MAGIC, 3, ExtraResourceInfo_t)


/**
 * Controls to support future user space handling of interrupts.
 * Not implemented yet.
 */
#define IOCTL_ADIPCIE_WAIT_FOR_INTERRUPT       _IO(ADIPCIE_MAGIC, 4)
#define IOCTL_ADIPCIE_DISABLE_INTERRUPT        _IO(ADIPCIE_MAGIC, 5)

#define IOCTL_ADIPCIE_LOAD_IMG		_IOR(ADIPCIE_MAGIC, 6, struct load_img_args)
#define IOCTL_ADIPCIE_REQ_IMG		_IOR(ADIPCIE_MAGIC, 7, int)

#define IOCTL_ADIPCIE_MAX_NR 7   // ^^^^^^must match last entry above^^^^^^^

#define SEND_INTERRUPT_0	0x10000
#define SEND_INTERRUPT_1	0x10001
#define SEND_INTERRUPT_2	0x10002
#define SEND_INTERRUPT_3	0x10003
#define SEND_INTERRUPT_4	0x10004
#define SEND_INTERRUPT_5	0x10005
#define SEND_INTERRUPT_6	0x10006
#define SEND_INTERRUPT_7	0x10007

#define CMD_LOAD_IMG	6
#define CMD_REQ_IMG		7

#define ADIPCIE_TIMEOUT	(msecs_to_jiffies(5000))

#define ADIPCIE_DEBUG

extern void exynos_pcie_wifion(int ch_num);
extern struct ion_device *ion_exynos;
extern int pci_assign_resource(struct pci_dev *dev, int resno);

#endif
