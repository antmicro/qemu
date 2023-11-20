/*
 * QEMU warp-pipe PCI device
 *
 * Copyright 2023 Antmicro <www.antmicro.com>
 * Copyright 2023 Meta
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <pcie_comm/client.h>

#include "qemu/osdep.h"
#include "qemu/sockets.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "qapi/error.h"

#define TYPE_PCI_PIPE_DEVICE "pipe"
typedef struct PipeState PipeState;
DECLARE_INSTANCE_CHECKER(PipeState, PIPE,
                         TYPE_PCI_PIPE_DEVICE)

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096

struct PipeState {
    PCIDevice pdev;
    MemoryRegion mmio;
    struct client_t client;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define PIPE_STATUS_COMPUTING    0x01
#define PIPE_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define PIPE_DMA_RUN             0x1
#define PIPE_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define PIPE_DMA_FROM_PCI       0
# define PIPE_DMA_TO_PCI         1
#define PIPE_DMA_IRQ             0x4
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    char dma_buf[DMA_SIZE];
};

static bool pipe_msi_enabled(PipeState *pipe)
{
    return msi_enabled(&pipe->pdev);
}

static void pipe_raise_irq(PipeState *pipe, uint32_t val)
{
    pipe->irq_status |= val;
    if (pipe->irq_status) {
        if (pipe_msi_enabled(pipe)) {
            msi_notify(&pipe->pdev, 0);
        } else {
            pci_set_irq(&pipe->pdev, 1);
        }
    }
}

static int pcied_read_handler(uint64_t addr, void *data, int length, void *opaque)
{
    PipeState *pipe = opaque;

    pci_dma_read(&pipe->pdev, addr, data, length);

    return 0;
}

static void pcied_write_handler(uint64_t addr, const void *data, int length, void *opaque)
{
    PipeState *pipe = opaque;

    pci_dma_write(&pipe->pdev, addr, data, length);
}

static uint8_t completion_data[8];
static bool received_cpl;
static void pcied_completion_handler(const struct completion_status_t completion_status,
                const void *data, int length)
{
   received_cpl = true;
   memcpy(completion_data, data, length);
}

static uint64_t pipe_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PipeState *pipe = opaque;

    union {
        uint8_t raw[8];
        uint8_t u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
    } data;

    assert(size == 1 || size == 2 || size == 4 || size == 8);

    received_cpl = false;
    pcie_read(&pipe->client, addr, size, pcied_completion_handler);

    while (pipe->client.active && !received_cpl) {
        client_read(&pipe->client);
    }

    memcpy(data.raw, completion_data, size);
    switch (size) {
    case 1:
        return data.u8;
    case 2:
        return data.u16;
    case 4:
        return data.u32;
    case 8:
        return data.u64;
    }
    return -1;
}

static void pipe_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    PipeState *pipe = opaque;

    union {
        uint8_t raw[8];
        uint8_t u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
    } data;

    assert(size == 1 || size == 2 || size == 4 || size == 8);
    switch (size) {
    case 1:
        data.u8 = val;
        break;
    case 2:
        data.u16 = val;
        break;
    case 4:
        data.u32 = val;
        break;
    case 8:
        data.u64 = val;
        break;
    }

    pcie_write(&pipe->client, addr, data.raw, size);
}

static const MemoryRegionOps pipe_mmio_ops = {
    .read = pipe_mmio_read,
    .write = pipe_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },

};

/*
 * We purposely use a thread, so that users are forced to wait for the status
 * register.
 */
static void *pipe_fact_thread(void *opaque)
{
    PipeState *pipe = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&pipe->thr_mutex);
        while ((qatomic_read(&pipe->status) & PIPE_STATUS_COMPUTING) == 0 &&
                        !pipe->stopping) {
            qemu_cond_wait(&pipe->thr_cond, &pipe->thr_mutex);
        }

        if (pipe->stopping) {
            qemu_mutex_unlock(&pipe->thr_mutex);
            break;
        }

        val = pipe->fact;
        qemu_mutex_unlock(&pipe->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&pipe->thr_mutex);
        pipe->fact = ret;
        qemu_mutex_unlock(&pipe->thr_mutex);
        qatomic_and(&pipe->status, ~PIPE_STATUS_COMPUTING);

        /* Clear COMPUTING flag before checking IRQFACT.  */
        smp_mb__after_rmw();

        if (qatomic_read(&pipe->status) & PIPE_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            pipe_raise_irq(pipe, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void pci_pipe_realize(PCIDevice *pdev, Error **errp)
{
    PipeState *pipe = PIPE(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    if (pipe->client.opaque == NULL) {
        error_setg(errp, "No transport specified.");
        return;
    }

    pcie_register_read_cb(&pipe->client, pcied_read_handler);
    pcie_register_write_cb(&pipe->client, pcied_write_handler);

    qemu_mutex_init(&pipe->thr_mutex);
    qemu_cond_init(&pipe->thr_cond);
    qemu_thread_create(&pipe->thread, "pipe", pipe_fact_thread,
                       pipe, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&pipe->mmio, OBJECT(pipe), &pipe_mmio_ops, pipe,
                    "pipe-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &pipe->mmio);
}

static void pci_pipe_uninit(PCIDevice *pdev)
{
    PipeState *pipe = PIPE(pdev);

    qemu_mutex_lock(&pipe->thr_mutex);
    pipe->stopping = true;
    qemu_mutex_unlock(&pipe->thr_mutex);
    qemu_cond_signal(&pipe->thr_cond);
    qemu_thread_join(&pipe->thread);

    qemu_cond_destroy(&pipe->thr_cond);
    qemu_mutex_destroy(&pipe->thr_mutex);

    msi_uninit(pdev);
}

static void pipe_connect(Object *obj, const char *addr, Error **errp)
{
    PipeState *pipe = PIPE(obj);
    int fd = inet_connect(addr, errp);

    if (fd == -1)
        return;

    client_create(&pipe->client, fd);
    pipe->client.opaque = pipe;
}

static void pipe_instance_init(Object *obj)
{
    object_property_add_str(obj, "connect", NULL, pipe_connect);
}

static void pipe_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_pipe_realize;
    k->exit = pci_pipe_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = 0x11e8;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void pci_pipe_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo pipe_info = {
        .name          = TYPE_PCI_PIPE_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(PipeState),
        .instance_init = pipe_instance_init,
        .class_init    = pipe_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&pipe_info);
}
type_init(pci_pipe_register_types)
