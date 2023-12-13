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

#include <warppipe/client.h>

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

#define TYPE_PCI_PIPE_DEVICE "warp-pipe"
typedef struct WarpPipeState WarpPipeState;
DECLARE_INSTANCE_CHECKER(WarpPipeState, WARP_PIPE,
                         TYPE_PCI_PIPE_DEVICE)

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096

struct WarpPipeState {
    PCIDevice pdev;
    MemoryRegion mmio;
    struct warppipe_client_t client;

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

    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    char dma_buf[DMA_SIZE];
};

static bool warp_msi_enabled(WarpPipeState *warp)
{
    return msi_enabled(&warp->pdev);
}

static void warp_raise_irq(WarpPipeState *warp, uint32_t val)
{
    warp->irq_status |= val;
    if (warp->irq_status) {
        if (warp_msi_enabled(warp)) {
            msi_notify(&warp->pdev, 0);
        } else {
            pci_set_irq(&warp->pdev, 1);
        }
    }
}

static int pcied_read_handler(uint64_t addr, void *data, int length, void *opaque)
{
    WarpPipeState *warp = opaque;

    pci_dma_read(&warp->pdev, addr, data, length);

    return 0;
}

static void pcied_write_handler(uint64_t addr, const void *data, int length, void *opaque)
{
    WarpPipeState *warp = opaque;

    pci_dma_write(&warp->pdev, addr, data, length);
}

static uint8_t completion_data[8];
static bool received_cpl;
static void pcied_completion_handler(const struct warppipe_completion_status_t completion_status,
                const void *data, int length)
{
   received_cpl = true;
   memcpy(completion_data, data, length);
}

static uint64_t warp_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    WarpPipeState *warp = opaque;

    union {
        uint8_t raw[8];
        uint8_t u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
    } data;

    assert(size == 1 || size == 2 || size == 4 || size == 8);

    received_cpl = false;
    warppipe_read(&warp->client, addr, size, pcied_completion_handler);

    while (warp->client.active && !received_cpl) {
        warppipe_client_read(&warp->client);
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

static void warp_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    WarpPipeState *warp = opaque;

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

    warppipe_write(&warp->client, addr, data.raw, size);
}

static const MemoryRegionOps warp_mmio_ops = {
    .read = warp_mmio_read,
    .write = warp_mmio_write,
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
static void *warp_fact_thread(void *opaque)
{
    WarpPipeState *warp = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&warp->thr_mutex);
        while ((qatomic_read(&warp->status) & PIPE_STATUS_COMPUTING) == 0 &&
                        !warp->stopping) {
            qemu_cond_wait(&warp->thr_cond, &warp->thr_mutex);
        }

        if (warp->stopping) {
            qemu_mutex_unlock(&warp->thr_mutex);
            break;
        }

        val = warp->fact;
        qemu_mutex_unlock(&warp->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&warp->thr_mutex);
        warp->fact = ret;
        qemu_mutex_unlock(&warp->thr_mutex);
        qatomic_and(&warp->status, ~PIPE_STATUS_COMPUTING);

        /* Clear COMPUTING flag before checking IRQFACT.  */
        smp_mb__after_rmw();

        if (qatomic_read(&warp->status) & PIPE_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            warp_raise_irq(warp, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void pci_warp_realize(PCIDevice *pdev, Error **errp)
{
    WarpPipeState *warp = WARP_PIPE(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    if (warp->client.opaque == NULL) {
        error_setg(errp, "No transport specified.");
        return;
    }

    warppipe_register_read_cb(&warp->client, pcied_read_handler);
    warppipe_register_write_cb(&warp->client, pcied_write_handler);

    qemu_mutex_init(&warp->thr_mutex);
    qemu_cond_init(&warp->thr_cond);
    qemu_thread_create(&warp->thread, "warp-pipe", warp_fact_thread,
                       warp, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&warp->mmio, OBJECT(warp), &warp_mmio_ops, warp,
                    "warp-pipe-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &warp->mmio);
}

static void pci_warp_uninit(PCIDevice *pdev)
{
    WarpPipeState *warp = WARP_PIPE(pdev);

    qemu_mutex_lock(&warp->thr_mutex);
    warp->stopping = true;
    qemu_mutex_unlock(&warp->thr_mutex);
    qemu_cond_signal(&warp->thr_cond);
    qemu_thread_join(&warp->thread);

    qemu_cond_destroy(&warp->thr_cond);
    qemu_mutex_destroy(&warp->thr_mutex);

    msi_uninit(pdev);
}

static void warp_connect(Object *obj, const char *addr, Error **errp)
{
    WarpPipeState *warp = WARP_PIPE(obj);
    int fd = inet_connect(addr, errp);

    if (fd == -1)
        return;

    warppipe_client_create(&warp->client, fd);
    warp->client.opaque = warp;
}

static void warp_instance_init(Object *obj)
{
    object_property_add_str(obj, "connect", NULL, warp_connect);
}

static void warp_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_warp_realize;
    k->exit = pci_warp_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = 0x11e8;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void pci_warp_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo warp_info = {
        .name          = TYPE_PCI_PIPE_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(WarpPipeState),
        .instance_init = warp_instance_init,
        .class_init    = warp_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&warp_info);
}
type_init(pci_warp_register_types)
