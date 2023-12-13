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
#define DMA_SIZE 4096

struct WarpPipeState {
    PCIDevice pdev;
    MemoryRegion mmio;
    struct warppipe_client_t client;

    char dma_buf[DMA_SIZE];
};

static uint8_t completion_data[8];
static bool received_cpl;

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

static void pcied_completion_handler(const struct warppipe_completion_status_t completion_status,
                const void *data, int length)
{
   memcpy(completion_data, data, length);
   received_cpl = true;
}

static uint64_t warp_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    WarpPipeState *warp = opaque;

    uint64_t data = 0;

    received_cpl = false;
    warppipe_read(&warp->client, addr, size, pcied_completion_handler);

    while (warp->client.active && !received_cpl) {
        warppipe_client_read(&warp->client);
    }

    // take into account host endianness and pass correct bytes
    for (int i = 0; i < size; i++) {
        data |= completion_data[i] << (i * 8);
    }

    return data;
}

static void warp_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    WarpPipeState *warp = opaque;
    // take into account host endianness and pass correct bytes
    uint8_t val_raw[] = {val, val >> 8, val >> 16, val >> 24, val >> 32, val >> 48, val >> 56};

    warppipe_write(&warp->client, addr, val_raw, size);
}

static const MemoryRegionOps warp_mmio_ops = {
    .read = warp_mmio_read,
    .write = warp_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },

};

static void pci_warp_realize(PCIDevice *pdev, Error **errp)
{
    WarpPipeState *warp = WARP_PIPE(pdev);

    if (warp->client.opaque == NULL) {
        error_setg(errp, "No transport specified.");
        return;
    }

    warppipe_register_read_cb(&warp->client, pcied_read_handler);
    warppipe_register_write_cb(&warp->client, pcied_write_handler);

    memory_region_init_io(&warp->mmio, OBJECT(warp), &warp_mmio_ops, warp,
                    "warp-pipe-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &warp->mmio);
}

static void pci_warp_uninit(PCIDevice *pdev)
{
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
