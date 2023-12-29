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

#include <warppipe/server.h>

#include "qemu/osdep.h"
#include "qemu/sockets.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "qapi/error.h"
#include "sysemu/kvm.h"

#define TYPE_PCI_PIPE_DEVICE "warp-pipe"
typedef struct WarpPipeState WarpPipeState;
DECLARE_INSTANCE_CHECKER(WarpPipeState, WARP_PIPE,
                         TYPE_PCI_PIPE_DEVICE)
#define DMA_SIZE 4096

typedef struct MSIVector {
    PCIDevice *pdev;
    int virq;
    bool unmasked;
} MSIVector;

struct WarpPipeState {
    PCIDevice pdev;
    MemoryRegion mmio;
    struct warppipe_server_t pcied_server;
    bool pcied_server_configured;
    QemuThread thread;
    QemuMutex thr_mutex_loop;
    QemuMutex thr_mutex_cfg;
    QemuMutex thr_mutex_data;
    QemuCond thr_loop;
    QemuCond thr_cfg_rdy;
    QemuCond thr_data_rdy;

#define WARP_PIPE_STATUS_IDLE    (1 << 0)
#define WARP_PIPE_STATUS_CFG_CPL    (1 << 1)
#define WARP_PIPE_STATUS_DATA_CPL   (1 << 2)
    uint32_t status;
    uint8_t completion_cfg_data[64];
    uint8_t completion_data[8];

    char dma_buf[DMA_SIZE];
    uint32_t vectors;
    MSIVector *msi_vectors;
};

static int pcied_read_handler(uint64_t addr, void *data, int length, void *private_data)
{
    WarpPipeState *warp = private_data;

    memcpy(data, warp->dma_buf + addr, length);

    return 0;
}

static void pcied_write_handler(uint64_t addr, const void *data, int length, void *private_data)
{
    WarpPipeState *warp = private_data;

    memcpy(warp->dma_buf + addr, data, length);
}

static int pcied_read_config0_handler(uint64_t addr, void *data, int length, void *private_data)
{
    WarpPipeState *pipe = private_data;

    memcpy(data, pipe->pdev.config + addr, length);

    return 0;
}

static void pcied_write_config0_handler(uint64_t addr, const void *data, int length, void *private_data)
{
    WarpPipeState *pipe = private_data;
    uint32_t val = *(uint32_t *)data;
    pci_default_write_config(&pipe->pdev, addr, val, length);
}

static void pcied_msix_write_handler(uint64_t addr, const void *data, int length, void *private_data)
{
    WarpPipeState *pipe = private_data;
    msix_notify(&pipe->pdev, 0);
}


static void pcied_completion_handler(const struct warppipe_completion_status_t completion_status,
                const void *data, int length, void *private_data)
{
    WarpPipeState *warp = private_data;

    memcpy(warp->completion_data, data, length);
    qatomic_set(&warp->status, WARP_PIPE_STATUS_DATA_CPL);
    qemu_mutex_lock(&warp->thr_mutex_data);
    qemu_cond_signal(&warp->thr_data_rdy); // wakeup warp-pipe thread
    qemu_mutex_unlock(&warp->thr_mutex_data);
}

static void pcied_config0_read_completion_handler(const struct warppipe_completion_status_t completion_status, const void *data, int length, void *private_data)
{
    WarpPipeState *warp = private_data;

    memcpy(warp->completion_cfg_data, data, length);
    qatomic_set(&warp->status, WARP_PIPE_STATUS_CFG_CPL);
    qemu_mutex_lock(&warp->thr_mutex_cfg);
    qemu_cond_signal(&warp->thr_cfg_rdy); // wakeup warp-pipe thread
    qemu_mutex_unlock(&warp->thr_mutex_cfg);
}

static uint32_t pcied_read_config(PCIDevice *d,
                                 uint32_t address, int len)
{
    WarpPipeState *warp = WARP_PIPE(d);
    if (warp->pcied_server.listen) {
        return pci_default_read_config(d, address, len);
    }
    uint32_t data;

    assert(len == 1 || len == 2 || len == 4);


    warppipe_config0_read(TAILQ_FIRST(&warp->pcied_server.clients)->client, address, len, &pcied_config0_read_completion_handler);

    qemu_mutex_lock(&warp->thr_mutex_loop);
    qemu_cond_signal(&warp->thr_loop); // wakeup warp-pipe thread
    qemu_mutex_unlock(&warp->thr_mutex_loop);
    qemu_mutex_lock(&warp->thr_mutex_cfg);
    qemu_mutex_unlock_iothread();
    while ((qatomic_read(&warp->status) & WARP_PIPE_STATUS_CFG_CPL) == 0) {
        qemu_cond_wait(&warp->thr_cfg_rdy, &warp->thr_mutex_cfg);
    }
    qemu_mutex_lock_iothread();

    memcpy(&data, warp->completion_cfg_data, len);
    qatomic_set(&warp->status, WARP_PIPE_STATUS_IDLE);
    qemu_mutex_unlock(&warp->thr_mutex_cfg);

    return data;
}

static void pcied_write_config(PCIDevice *d,
                                 uint32_t address, uint32_t val_in, int len)
{
    WarpPipeState *pipe = WARP_PIPE(d);
    if (pipe->pcied_server.listen) {
        pci_default_write_config(d, address, val_in, len);
        return;
    }

    warppipe_config0_write(TAILQ_FIRST(&pipe->pcied_server.clients)->client, address, &val_in, len);
    // We need to read it back and save it to local copy,
    // as QEMU have side-effects on config write
    pci_default_write_config(d, address, val_in, len);
}

static uint64_t warp_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    WarpPipeState *warp = opaque;

    uint64_t data = 0;

    warppipe_read(TAILQ_FIRST(&warp->pcied_server.clients)->client, 0, addr, size, pcied_completion_handler);
    qemu_mutex_lock(&warp->thr_mutex_loop);
    qemu_cond_signal(&warp->thr_loop); // wakeup warp-pipe thread
    qemu_mutex_unlock(&warp->thr_mutex_loop);
    qemu_mutex_lock(&warp->thr_mutex_data);
    qemu_mutex_unlock_iothread();
    while ((qatomic_read(&warp->status) & WARP_PIPE_STATUS_DATA_CPL) == 0) {
        qemu_cond_wait(&warp->thr_data_rdy, &warp->thr_mutex_data);
    }
    qemu_mutex_lock_iothread();

    // take into account host endianness and pass correct bytes
    for (int i = 0; i < size; i++) {
        data |= warp->completion_data[i] << (i * 8);
    }
    qatomic_set(&warp->status, WARP_PIPE_STATUS_IDLE);
    qemu_mutex_unlock(&warp->thr_mutex_data);

    return data;
}

static void warp_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    WarpPipeState *warp = opaque;
    // take into account host endianness and pass correct bytes
    uint8_t val_raw[] = {val, val >> 8, val >> 16, val >> 24, val >> 32, val >> 48, val >> 56};

    warppipe_write(TAILQ_FIRST(&warp->pcied_server.clients)->client, 0, addr, val_raw, size);
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

static void *warp_thread(void *opaque)
{
    WarpPipeState *warp = opaque;

    if (warp->pcied_server.listen) {
        int counter = 0;
        while (1) {
            qemu_cond_timedwait(&warp->thr_loop, &warp->thr_mutex_loop, 50);
            qemu_mutex_lock_iothread();
            warppipe_server_loop(&warp->pcied_server);
            qemu_mutex_unlock_iothread();
            counter++;
            if (counter == 60) { // every 60 messages or 60s
                // Send notification for test purposes
                MSIMessage msg = msix_get_message((PCIDevice *)warp, 0);
                struct warppipe_client_node_t *i;
                TAILQ_FOREACH(i, &warp->pcied_server.clients, next)
                    warppipe_write(i->client, 1, 0x0, &msg.data, sizeof(msg.data));
                counter = 0;
            }
            if (warp->pcied_server.quit)
                break;
        }
    } else {
        while (1) {
            qemu_mutex_lock(&warp->thr_mutex_loop);
            qemu_cond_timedwait(&warp->thr_loop, &warp->thr_mutex_loop, 50);
            // when we have some data already available,
            // wait
            while ((qatomic_read(&warp->status) & WARP_PIPE_STATUS_IDLE) == 0) {
                // when signaled or every 100ms
                qemu_cond_timedwait(&warp->thr_loop, &warp->thr_mutex_loop, 100);
            }
            qemu_mutex_lock_iothread();
            warppipe_server_loop(&warp->pcied_server);
            qemu_mutex_unlock_iothread();
            qemu_mutex_unlock(&warp->thr_mutex_loop);
            if (warp->pcied_server.quit)
                break;
        }
    }

    return NULL;
}

static int pci_warp_vector_unmask(PCIDevice *dev, unsigned vector,
                                 MSIMessage msg)
{
    WarpPipeState *warp = WARP_PIPE(dev);
    MSIVector *v = &warp->msi_vectors[vector];

    kvm_irqchip_update_msi_route(kvm_state, v->virq, msg, dev);
    kvm_irqchip_commit_routes(kvm_state);

    v->unmasked = true;

    return 0;
}

static void pci_warp_vector_mask(PCIDevice *dev, unsigned vector)
{
    WarpPipeState *warp = WARP_PIPE(dev);
    MSIVector *v = &warp->msi_vectors[vector];

    v->unmasked = false;
}

static void pci_warp_vector_poll(PCIDevice *dev,
                                unsigned int vector_start,
                                unsigned int vector_end)
{
}

static void pci_warp_realize(PCIDevice *pdev, Error **errp)
{
    WarpPipeState *warp = WARP_PIPE(pdev);


    warp->vectors = 1;
    warp->msi_vectors = g_new0(MSIVector, warp->vectors);
    if (msix_init_exclusive_bar(PCI_DEVICE(warp), warp->vectors, 1, errp)) {
        return;
    }
    for (int i = 0; i < warp->vectors; i++) {
        msix_vector_use(PCI_DEVICE(warp), i);
    }
    for (int i = 0; i < warp->vectors; i++) {
        msix_set_mask(PCI_DEVICE(warp), i, false);
    }
    if (msix_set_vector_notifiers(pdev,
                                  pci_warp_vector_unmask,
                                  pci_warp_vector_mask,
                                  pci_warp_vector_poll)) {
        return;
    }
    if(TAILQ_FIRST(&warp->pcied_server.clients)) {
        if (TAILQ_FIRST(&warp->pcied_server.clients)->client->private_data == NULL) {
            error_setg(errp, "No transport specified.");
            return;
        }
    }

    qemu_mutex_init(&warp->thr_mutex_loop);
    qemu_mutex_init(&warp->thr_mutex_cfg);
    qemu_mutex_init(&warp->thr_mutex_data);
    qemu_cond_init(&warp->thr_loop);
    qemu_cond_init(&warp->thr_cfg_rdy);
    qemu_cond_init(&warp->thr_data_rdy);
    qatomic_set(&warp->status, WARP_PIPE_STATUS_IDLE);

    qemu_thread_create(&warp->thread, "warp", warp_thread,
                       warp, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&warp->mmio, OBJECT(warp), &warp_mmio_ops, warp,
                    "warp-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &warp->mmio);

    if (warp->pcied_server.listen) {
        for (int i = 0; i < DMA_SIZE; i++)
        {
            warp->dma_buf[i] = i;
        }
    }

}

static void pci_warp_uninit(PCIDevice *pdev)
{
    WarpPipeState *warp = WARP_PIPE(pdev);

    warp->pcied_server.quit = true;
    qemu_thread_join(&warp->thread);

    qemu_cond_destroy(&warp->thr_loop);
    qemu_cond_destroy(&warp->thr_cfg_rdy);
    qemu_cond_destroy(&warp->thr_data_rdy);
    qemu_mutex_destroy(&warp->thr_mutex_loop);
    qemu_mutex_destroy(&warp->thr_mutex_cfg);
    qemu_mutex_destroy(&warp->thr_mutex_data);

    msix_uninit_exclusive_bar(pdev);
}

static void warp_server_accept_cb(struct warppipe_client_t *client, void *private_data)
{
    WarpPipeState *pipe = private_data;
    client->private_data = pipe;

    warppipe_register_bar(client, 0x10000000, 1 * MiB, 0, pcied_read_handler, pcied_write_handler);
    warppipe_register_bar(client, 0x20000000, 1 * MiB, 1, NULL, pcied_msix_write_handler);
    warppipe_register_config0_read_cb(client, pcied_read_config0_handler);
    warppipe_register_config0_write_cb(client, pcied_write_config0_handler);
}

static void warp_create_server(WarpPipeState *warp, bool listen, const char *addr, Error **errp)
{
    InetSocketAddress *inet_addr = g_new(InetSocketAddress, 1);
    if (inet_parse(inet_addr, addr, errp)) {
        return;
    }

    warp->pcied_server.host = malloc(strlen(inet_addr->host) * sizeof(char));
    warp->pcied_server.port = malloc(strlen(inet_addr->port) * sizeof(char));
    strcpy((char *)warp->pcied_server.host, inet_addr->host);
    strcpy((char *)warp->pcied_server.port, inet_addr->port);
    warp->pcied_server.addr_family = AF_INET;
    warp->pcied_server.listen = listen;
    warp->pcied_server.quit = false;
    if (warp->pcied_server.listen)
        warp->pcied_server.host = NULL;

    warp->pcied_server.private_data = warp;
    warppipe_server_register_accept_cb(&warp->pcied_server, warp_server_accept_cb);

    if (warppipe_server_create(&warp->pcied_server) == -1) {
        printf("Failed to create server!");
        return;
    }
}

static void warp_listen(Object *obj, const char *addr, Error **errp)
{
    if (WARP_PIPE(obj)->pcied_server_configured) {
        printf("warp-pipe object already initialized, please use only single \"connect\" or \"listen\" property");
        return;
    }
    WARP_PIPE(obj)->pcied_server_configured = true;
    warp_create_server(WARP_PIPE(obj), true, addr, errp);
}

static void warp_connect(Object *obj, const char *addr, Error **errp)
{
    if (WARP_PIPE(obj)->pcied_server_configured) {
        printf("warp-pipe object already initialized, please use only single \"connect\" or \"listen\" property");
        return;
    }
    WARP_PIPE(obj)->pcied_server_configured = true;
    warp_create_server(WARP_PIPE(obj), false, addr, errp);
}

static void warp_instance_init(Object *obj)
{
    object_property_add_str(obj, "listen", NULL, warp_listen);
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
    k->config_read = pcied_read_config;
    k->config_write = pcied_write_config;
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
