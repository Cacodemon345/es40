#include "StdAfx.h"
#include "System.h"
#include "DMA.h"
#include "AliM1543C.h"
#include "PCIDevice.h"

CDMA* theDMA = 0;

#define LEGACY_IO(id,port,size) c->RegisterMemory(this, id, U64(0x00000801fc000000) + port, size)

int CDMA::dma_get_drq(int channel)
{
    return !!(dma_stat_rq_pc & (1 << channel));
}

void CDMA::dma_set_drq(int channel, int set)
{
    dma_stat_rq_pc &= ~(1 << channel);
    if (set)
        dma_stat_rq_pc |= (1 << channel);
}

int CDMA::dma_transfer_size(dma_t* dev)
{
    return dev->transfer_mode & 0xff;
}

void CDMA::dma_sg_next_addr(dma_t* dev)
{
    int ts = dma_transfer_size(dev);

	theAli->do_pci_read(dev->ptr_cur, (uint8_t*)&(dev->addr), 1, ts);
    theAli->do_pci_read(dev->ptr_cur + 4, (uint8_t*)&(dev->count), 1, ts);
    //dma_log("DMA S/G DWORDs: %08X %08X\n", dev->addr, dev->count);
    dev->eot = dev->count >> 31;
    dev->count &= 0xfffe;
    dev->cb = (uint16_t)dev->count;
    dev->cc = dev->count;
    if (!dev->count)
        dev->count = 65536;
    if (ts == 2)
        dev->addr &= 0xfffffffe;
    dev->ab = dev->addr & dma_mask;
    dev->ac = dev->addr & dma_mask;
    dev->page = dev->page_l = (dev->ac >> 16) & 0xff;
    dev->page_h = (dev->ac >> 24) & 0xff;
    dev->ptr_cur += 8;
}

void CDMA::dma_reset(void)
{
    int c;

    dma_wp[0] = dma_wp[1] = 0;
    dma_m = 0;

    dma_e = 0xff;

    for (c = 0; c < 16; c++)
        dmaregs[0][c] = dmaregs[1][c] = 0;

    for (c = 0; c < 8; c++) {
        memset(&(dma[c]), 0x00, sizeof(dma_t));
        dma[c].size = (c & 4) ? 1 : 0;
        dma[c].transfer_mode = (c & 4) ? 0x0202 : 0x0101;
    }

    dma_stat = 0x00;
    dma_stat_rq = 0x00;
    dma_stat_rq_pc = 0x00;
    dma_stat_adv_pend = 0x00;
    dma_req_is_soft = 0;
    dma_advanced = 0;

    memset(dma_buffer, 0x00, sizeof(dma_buffer));
    memset(dma16_buffer, 0x00, sizeof(dma16_buffer));

    //dma_remove_sg();
    dma_sg_base = 0x0400;

    dma_mask = 0x00ffffff;

    dma_at = 1;
}

CDMA::CDMA(CConfigurator* cfg, CSystem* c) : CSystemComponent(cfg, c)
{
	dma_reset();
    dma_set_params(1, 0xffffffff);

#define LEGACY_IO(id,port,size) c->RegisterMemory(this, id, U64(0x00000801fc000000) + port, size)
    LEGACY_IO(DMA0_IO_CHANNEL, 0x00, 8);
    LEGACY_IO(DMA0_IO_MAIN, 0x08, 8);
    LEGACY_IO(DMA_IO_LPAGE, 0x80, 16);
    LEGACY_IO(DMA1_IO_CHANNEL, 0xc0, 16);
    LEGACY_IO(DMA1_IO_MAIN, 0xd0, 16);
    LEGACY_IO(DMA_IO_HPAGE, 0x480, 8);
    LEGACY_IO(DMA0_IO_EXT, 0x040b, 1);
    LEGACY_IO(DMA1_IO_EXT, 0x04D6, 1);
    LEGACY_IO(DMA_ALIAS_1, 0x0090, 2);
    LEGACY_IO(DMA_ALIAS_2, 0x0093, 13);
	LEGACY_IO(DMA_SG_1, 0x040a, 1);
    LEGACY_IO(DMA_SG_2, 0x0410, 8);
    LEGACY_IO(DMA_SG_3, 0x0418, 8);
    LEGACY_IO(DMA_SG_4, 0x0420, 32);
}

void CDMA::dma_set_params(uint8_t advanced, uint32_t mask)
{
    dma_advanced = advanced;
    dma_mask = mask;
}

void CDMA::WriteMem(int index, u64 address, int dsize, u64 data)
{
    if (index >= DMA_SG_2) {
        switch (dsize) {
            case 32:
                dma_sg_writel(address & 0xFFFF, data, this);
                return;
            case 16:
                dma_sg_writew(address & 0xFFFF, data, this);
                return;
            case 8:
                dma_sg_write(address & 0xFFFF, data, this);
                return;
        }
    }
    switch (dsize)
    {
    case 32:
        WriteMem(index, address + 0, 8, (data >> 0) & 0xff);
        WriteMem(index, address + 1, 8, (data >> 8) & 0xff);
        WriteMem(index, address + 2, 8, (data >> 16) & 0xff);
        WriteMem(index, address + 3, 8, (data >> 24) & 0xff);
        return;

    case 16:
        WriteMem(index, address + 0, 8, (data >> 0) & 0xff);
        WriteMem(index, address + 1, 8, (data >> 8) & 0xff);
        return;
    
    case 8:
    {
        address &= 0xFFFF;
        if (address >= 0x00 && address <= 0xf) {
            // return dma_read(address);
			dma_write(address, data & 0xff);
        }
        if (address >= 0x80 && address <= 0x8f) {
            // return dma_page_read(address);
			dma_page_write(address, data & 0xff);
        }
        if (address >= 0xC0 && address <= 0xDF) {
            //return dma16_read(address);
			dma16_write(address, data & 0xff);
        }
        if (address >= 0x480 && address <= 0x487) {
            // return dma_high_page_read(address);
			dma_high_page_write(address, data & 0xff);
        }
        if (address != 0x92 && address >= 0x90 && address <= 0x9f) {
            // return dma_page_read(address);
			dma_page_write(address, data & 0xff);
        }
        return;
    }
    }
}

u64 CDMA::ReadMem(int index, u64 address, int dsize)
{
    if (index >= DMA_SG_2) {
        switch (dsize) {
            case 32:
				return dma_sg_readl(address & 0xFFFF, this);
            case 16:
                return dma_sg_readw(address & 0xFFFF, this);
            case 8:
                return dma_sg_read(address & 0xFFFF, this);
        }
    }
    if (index == DMA_SG_1) {
		return dma_sg_int_status_read(address & 0xFFFF);
    }
    u64 ret;
    u8  data = 0;
    int ctrlr;
    int num;
    //printf("dma: Readmem %s, %" PRIx64 ", %x\n",DMA_INDEX(index),address, dsize);
    switch (dsize)
    {
    case 32:
        ret = ReadMem(index, address, 8);
        ret |= ReadMem(index, address + 1, 8) << 8;
        ret |= ReadMem(index, address + 2, 8) << 16;
        ret |= ReadMem(index, address + 3, 8) << 24;
        return ret;

    case 16:
        ret = ReadMem(index, address, 8);
        ret |= ReadMem(index, address + 1, 8) << 8;
        return ret;
    case 8:
    {
        address &= 0xFFFF;
        if (address >= 0x00 && address <= 0xf) {
            return dma_read(address);
        }
		if (address >= 0x80 && address <= 0x8f) {
            return dma_page_read(address);
        }
		if (address >= 0xC0 && address <= 0xDF) {
            return dma16_read(address);
        }
		if (address >= 0x480 && address <= 0x487) {
            return dma_high_page_read(address);
        }
		if (address != 0x92 && address >= 0x90 && address <= 0x9f) {
            return dma_page_read(address);
        }
        return 0xff;
    }
    }

    return -1;
}

void CDMA::send_data(int channel, void* data)
{
    uint8_t* src = (uint8_t*)data;
    int res;

    while (true) {
        res = dma_channel_write(channel, *src);
        if (res == DMA_NODATA)
            break;
        src++;
    }
}

void CDMA::recv_data(int channel, void* data)
{
    uint8_t* src = (uint8_t*)data;
    int res;

    while (true) {
        res = dma_channel_read(channel);
        if (res == DMA_NODATA)
            break;
        *src = res & 0xff;
        src++;
    }
}

void CDMA::dma_block_transfer(int channel)
{
    int bit16 = (channel >= 4);

    if (dma_advanced)
        bit16 = !!(dma_transfer_size(&(dma[channel])) == 2);

    dma_req_is_soft = 1;
    for (uint16_t i = 0; i <= dma[channel].cb; i++) {
        if ((dma[channel].mode & 0x8c) == 0x84) {
            if (bit16)
                dma_channel_write(channel, dma16_buffer[i]);
            else
                dma_channel_write(channel, dma_buffer[i]);
        }
        else if ((dma[channel].mode & 0x8c) == 0x88) {
            if (bit16)
                dma16_buffer[i] = dma_channel_read(channel);
            else
                dma_buffer[i] = dma_channel_read(channel);
        }
    }
    dma_req_is_soft = 0;
}

void CDMA::dma_mem_to_mem_transfer(void)
{
    int i;

    if ((dma[0].mode & 0x0c) != 0x08)
        fatal("DMA memory to memory transfer: channel 0 mode not read\n");
    if ((dma[1].mode & 0x0c) != 0x04)
        fatal("DMA memory to memory transfer: channel 1 mode not write\n");

    dma_req_is_soft = 1;

    for (i = 0; i <= dma[0].cb; i++)
        dma_buffer[i] = dma_channel_read(0);

    for (i = 0; i <= dma[1].cb; i++)
        dma_channel_write(1, dma_buffer[i]);

    dma_req_is_soft = 0;
}

void CDMA::dma_sg_write(uint16_t port, uint8_t val, void* priv)
{
    dma_t* dev = (dma_t*)priv;

    port &= 0xff;

    if (port < 0x20)
        port &= 0xf8;
    else
        port &= 0xe3;

    switch (port) {
    case 0x00:
        if ((val & 1) && !(dev->sg_command & 1)) { /*Start*/
#ifdef ENABLE_DMA_LOG
            dma_log("DMA S/G start\n");
#endif
            dev->ptr_cur = dev->ptr;
            dma_sg_next_addr(dev);
            dev->sg_status = (dev->sg_status & 0xf7) | 0x01;
        }
        if (!(val & 1) && (dev->sg_command & 1)) { /*Stop*/
#ifdef ENABLE_DMA_LOG
            dma_log("DMA S/G stop\n");
#endif
            dev->sg_status &= ~0x81;
        }

        dev->sg_command = val;
        break;
    case 0x20:
        dev->ptr = (dev->ptr & 0xffffff00) | (val & 0xfc);
        dev->ptr %= (1 << theSystem->iNumMemoryBits);
        dev->ptr0 = val;
        break;
    case 0x21:
        dev->ptr = (dev->ptr & 0xffff00fc) | (val << 8);
        dev->ptr %= (1 << theSystem->iNumMemoryBits);
        break;
    case 0x22:
        dev->ptr = (dev->ptr & 0xff00fffc) | (val << 16);
        dev->ptr %= (1 << theSystem->iNumMemoryBits);
        break;
    case 0x23:
        dev->ptr = (dev->ptr & 0x00fffffc) | (val << 24);
        dev->ptr %= (1 << theSystem->iNumMemoryBits);
        break;

    default:
        break;
    }
}

void CDMA::dma_sg_writew(uint16_t port, uint16_t val, void* priv)
{
    dma_t* dev = (dma_t*)priv;

    port &= 0xff;

    if (port < 0x20)
        port &= 0xf8;
    else
        port &= 0xe3;

    switch (port) {
    case 0x00:
        dma_sg_write(port, val & 0xff, priv);
        break;
    case 0x20:
        dev->ptr = (dev->ptr & 0xffff0000) | (val & 0xfffc);
        dev->ptr %= (1 << theSystem->iNumMemoryBits);
        dev->ptr0 = val & 0xff;
        break;
    case 0x22:
        dev->ptr = (dev->ptr & 0x0000fffc) | (val << 16);
        dev->ptr %= (1 << theSystem->iNumMemoryBits);
        break;

    default:
        break;
    }
}

void CDMA::dma_sg_writel(uint16_t port, uint32_t val, void* priv)
{
    dma_t* dev = (dma_t*)priv;

    port &= 0xff;

    if (port < 0x20)
        port &= 0xf8;
    else
        port &= 0xe3;

    switch (port) {
    case 0x00:
        dma_sg_write(port, val & 0xff, priv);
        break;
    case 0x20:
        dev->ptr = (val & 0xfffffffc);
        dev->ptr %= (1 << theSystem->iNumMemoryBits);
        dev->ptr0 = val & 0xff;
        break;

    default:
        break;
    }
}

uint8_t CDMA::dma_sg_read(uint16_t port, void* priv)
{
    const dma_t* dev = (dma_t*)priv;

    uint8_t ret = 0xff;

    port &= 0xff;

    if (port < 0x20)
        port &= 0xf8;
    else
        port &= 0xe3;

    switch (port) {
    case 0x08:
        ret = (dev->sg_status & 0x01);
        if (dev->eot)
            ret |= 0x80;
        if ((dev->sg_command & 0xc0) == 0x40)
            ret |= 0x20;
        if (dev->ab != 0x00000000)
            ret |= 0x08;
        if (dev->ac != 0x00000000)
            ret |= 0x04;
        break;
    case 0x20:
        ret = dev->ptr0;
        break;
    case 0x21:
        ret = dev->ptr >> 8;
        break;
    case 0x22:
        ret = dev->ptr >> 16;
        break;
    case 0x23:
        ret = dev->ptr >> 24;
        break;

    default:
        break;
    }

    return ret;
}

uint16_t CDMA::dma_sg_readw(uint16_t port, void* priv)
{
    const dma_t* dev = (dma_t*)priv;

    uint16_t ret = 0xffff;

    port &= 0xff;

    if (port < 0x20)
        port &= 0xf8;
    else
        port &= 0xe3;

    switch (port) {
    case 0x08:
        ret = (uint16_t)dma_sg_read(port, priv);
        break;
    case 0x20:
        ret = dev->ptr0 | (dev->ptr & 0xff00);
        break;
    case 0x22:
        ret = dev->ptr >> 16;
        break;

    default:
        break;
    }

    return ret;
}

uint32_t CDMA::dma_sg_readl(uint16_t port, void* priv)
{
    const dma_t* dev = (dma_t*)priv;

    uint32_t ret = 0xffffffff;

    port &= 0xff;

    if (port < 0x20)
        port &= 0xf8;
    else
        port &= 0xe3;

    switch (port) {
    case 0x08:
        ret = (uint32_t)dma_sg_read(port, priv);
        break;
    case 0x20:
        ret = dev->ptr0 | (dev->ptr & 0xffffff00);
        break;

    default:
        break;
    }

    return ret;
}

void CDMA::dma_ext_mode_write(uint16_t addr, uint8_t val)
{
    int channel = (val & 0x03);

    if ((addr & 0xffff) == 0x4d6)
        channel |= 4;

    dma[channel].ext_mode = val & 0x7c;

    switch ((val > 2) & 0x03) {
    case 0x00:
        dma[channel].transfer_mode = 0x0101;
        break;
    case 0x01:
        dma[channel].transfer_mode = 0x0202;
        break;
    case 0x02: /* 0x02 is reserved. */
        /* Logic says this should be an undocumented mode that counts by words,
           but is 8-bit I/O, thus only transferring every second byte. */
        dma[channel].transfer_mode = 0x0201;
        break;
    case 0x03:
        dma[channel].transfer_mode = 0x0102;
        break;

    default:
        break;
    }
}

uint8_t CDMA::dma_sg_int_status_read(uint16_t addr)
{
    uint8_t ret = 0x00;

    for (uint8_t i = 0; i < 8; i++) {
        if (i != 4)
            ret = (!!(dma[i].sg_status & 8)) << i;
    }

    return ret;
}

uint8_t CDMA::dma_read(uint16_t addr)
{
    int     channel = (addr >> 1) & 3;
    int     count;
    uint8_t ret = (dmaregs[0][addr & 0xf]);

    switch (addr & 0xf) {
    case 0:
    case 2:
    case 4:
    case 6: /*Address registers*/
        dma_wp[0] ^= 1;
        if (dma_wp[0])
            ret = (dma[channel].ac & 0xff);
        else
            ret = ((dma[channel].ac >> 8) & 0xff);
        break;

    case 1:
    case 3:
    case 5:
    case 7: /*Count registers*/
        dma_wp[0] ^= 1;
        count = dma[channel].cc/* + 1*/;
        if (dma_wp[0])
            ret = count & 0xff;
        else
            ret = count >> 8;
        break;

    case 8: /*Status register*/
        ret = dma_stat_rq_pc & 0xf;
        ret <<= 4;
        ret |= dma_stat & 0xf;
        dma_stat &= ~0xf;
        break;

    case 0xd: /*Temporary register*/
        ret = 0x00;
        break;

    default:
        break;
    }

    return ret;
}

void CDMA::dma_write(uint16_t addr, uint8_t val)
{
    int channel = (addr >> 1) & 3;

    dmaregs[0][addr & 0xf] = val;
    switch (addr & 0xf) {
    case 0:
    case 2:
    case 4:
    case 6: /*Address registers*/
        dma_wp[0] ^= 1;
        if (dma_wp[0])
            dma[channel].ab = (dma[channel].ab & 0xffffff00 & dma_mask) | val;
        else
            dma[channel].ab = (dma[channel].ab & 0xffff00ff & dma_mask) | (val << 8);
        dma[channel].ac = dma[channel].ab;
        return;

    case 1:
    case 3:
    case 5:
    case 7: /*Count registers*/
        dma_wp[0] ^= 1;
        if (dma_wp[0])
            dma[channel].cb = (dma[channel].cb & 0xff00) | val;
        else
            dma[channel].cb = (dma[channel].cb & 0x00ff) | (val << 8);
        dma[channel].cc = dma[channel].cb;
        return;

    case 8: /*Control register*/
        dma_command[0] = val;
#ifdef ENABLE_DMA_LOG
        if (val & 0x01)
            dma_log("[%08X:%04X] Memory-to-memory enable\n", CS, cpu_state.pc);
#endif
        return;

    case 9: /*Request register */
        channel = (val & 3);
        if (val & 4) {
            dma_stat_rq_pc |= (1 << channel);
            if ((channel == 0) && (dma_command[0] & 0x01)) {
                dma_mem_to_mem_transfer();
            }
            else
                dma_block_transfer(channel);
        }
        else
            dma_stat_rq_pc &= ~(1 << channel);
        break;

    case 0xa: /*Mask*/
        channel = (val & 3);
        if (val & 4)
            dma_m |= (1 << channel);
        else
            dma_m &= ~(1 << channel);
        return;

    case 0xb: /*Mode*/
        channel = (val & 3);
        dma[channel].mode = val;
        if (dma_ps2.is_ps2) {
            dma[channel].ps2_mode &= ~0x1c;
            if (val & 0x20)
                dma[channel].ps2_mode |= 0x10;
            if ((val & 0xc) == 8)
                dma[channel].ps2_mode |= 4;
            else if ((val & 0xc) == 4)
                dma[channel].ps2_mode |= 0xc;
        }
        return;

    case 0xc: /*Clear FF*/
        dma_wp[0] = 0;
        return;

    case 0xd: /*Master clear*/
        dma_wp[0] = 0;
        dma_m |= 0xf;
        dma_stat_rq_pc &= ~0x0f;
        return;

    case 0xe: /*Clear mask*/
        dma_m &= 0xf0;
        return;

    case 0xf: /*Mask write*/
        dma_m = (dma_m & 0xf0) | (val & 0xf);
        return;

    default:
        break;
    }
}

uint8_t CDMA::dma_ps2_read(uint16_t addr)
{
    const dma_t* dma_c = &dma[dma_ps2.xfr_channel];
    uint8_t temp = 0xff;

    switch (addr) {
    case 0x1a:
        switch (dma_ps2.xfr_command) {
        case 2: /*Address*/
        case 3:
            switch (dma_ps2.byte_ptr) {
            case 0:
                temp = dma_c->ac & 0xff;
                dma_ps2.byte_ptr = 1;
                break;
            case 1:
                temp = (dma_c->ac >> 8) & 0xff;
                dma_ps2.byte_ptr = 2;
                break;
            case 2:
                temp = (dma_c->ac >> 16) & 0xff;
                dma_ps2.byte_ptr = 0;
                break;

            default:
                break;
            }
            break;

        case 4: /*Count*/
        case 5:
            if (dma_ps2.byte_ptr)
                temp = dma_c->cc >> 8;
            else
                temp = dma_c->cc & 0xff;
            dma_ps2.byte_ptr = (dma_ps2.byte_ptr + 1) & 1;
            break;

        case 6: /*Read DMA status*/
            if (dma_ps2.byte_ptr) {
                temp = ((dma_stat_rq & 0xf0) >> 4) | (dma_stat & 0xf0);
                dma_stat &= ~0xf0;
                dma_stat_rq &= ~0xf0;
            }
            else {
                temp = (dma_stat_rq & 0xf) | ((dma_stat & 0xf) << 4);
                dma_stat &= ~0xf;
                dma_stat_rq &= ~0xf;
            }
            dma_ps2.byte_ptr = (dma_ps2.byte_ptr + 1) & 1;
            break;

        case 7: /*Mode*/
            temp = dma_c->ps2_mode;
            break;

        case 8: /*Arbitration Level*/
            temp = dma_c->arb_level;
            break;

        case 9: /*Set DMA mask*/
            dma_m |= (1 << dma_ps2.xfr_channel);
            break;

        case 0xa: /*Reset DMA mask*/
            dma_m &= ~(1 << dma_ps2.xfr_channel);
            break;

        case 0xb:
            if (!(dma_m & (1 << dma_ps2.xfr_channel)))
                dma_ps2_run(dma_ps2.xfr_channel);
            break;

        default:
            fatal("Bad XFR Read command %i channel %i\n", dma_ps2.xfr_command, dma_ps2.xfr_channel);
        }
        break;

    default:
        break;
    }
    return temp;
}

void CDMA::dma_ps2_write(uint16_t addr, uint8_t val)
{
    dma_t* dma_c = &dma[dma_ps2.xfr_channel];
    uint8_t mode;

    switch (addr) {
    case 0x18:
        dma_ps2.xfr_channel = val & 0x7;
        dma_ps2.xfr_command = val >> 4;
        dma_ps2.byte_ptr = 0;
        switch (dma_ps2.xfr_command) {
        case 9: /*Set DMA mask*/
            dma_m |= (1 << dma_ps2.xfr_channel);
            break;

        case 0xa: /*Reset DMA mask*/
            dma_m &= ~(1 << dma_ps2.xfr_channel);
            break;

        case 0xb:
            if (!(dma_m & (1 << dma_ps2.xfr_channel)))
                dma_ps2_run(dma_ps2.xfr_channel);
            break;

        default:
            break;
        }
        break;

    case 0x1a:
        switch (dma_ps2.xfr_command) {
        case 0: /*I/O address*/
            if (dma_ps2.byte_ptr)
                dma_c->io_addr = (dma_c->io_addr & 0x00ff) | (val << 8);
            else
                dma_c->io_addr = (dma_c->io_addr & 0xff00) | val;
            dma_ps2.byte_ptr = (dma_ps2.byte_ptr + 1) & 1;
            break;

        case 2: /*Address*/
            switch (dma_ps2.byte_ptr) {
            case 0:
                dma_c->ac = (dma_c->ac & 0xffff00) | val;
                dma_ps2.byte_ptr = 1;
                break;

            case 1:
                dma_c->ac = (dma_c->ac & 0xff00ff) | (val << 8);
                dma_ps2.byte_ptr = 2;
                break;

            case 2:
                dma_c->ac = (dma_c->ac & 0x00ffff) | (val << 16);
                dma_ps2.byte_ptr = 0;
                break;

            default:
                break;
            }
            dma_c->ab = dma_c->ac;
            break;

        case 4: /*Count*/
            if (dma_ps2.byte_ptr)
                dma_c->cc = (dma_c->cc & 0xff) | (val << 8);
            else
                dma_c->cc = (dma_c->cc & 0xff00) | val;
            dma_ps2.byte_ptr = (dma_ps2.byte_ptr + 1) & 1;
            dma_c->cb = dma_c->cc;
            break;

        case 7: /*Mode register*/
            mode = 0;
            if (val & DMA_PS2_DEC2)
                mode |= 0x20;
            if ((val & DMA_PS2_XFER_MASK) == DMA_PS2_XFER_MEM_TO_IO)
                mode |= 8;
            else if ((val & DMA_PS2_XFER_MASK) == DMA_PS2_XFER_IO_TO_MEM)
                mode |= 4;
            dma_c->mode = (dma_c->mode & ~0x2c) | mode;
            if (val & DMA_PS2_AUTOINIT)
                dma_c->mode |= 0x10;
            dma_c->ps2_mode = val;
            dma_c->size = val & DMA_PS2_SIZE16;
            break;

        case 8: /*Arbitration Level*/
            dma_c->arb_level = val;
            break;

        case 9: /*Set DMA mask*/
            dma_m |= (1 << dma_ps2.xfr_channel);
            break;

        case 0xa: /*Reset DMA mask*/
            dma_m &= ~(1 << dma_ps2.xfr_channel);
            break;

        case 0xb:
            if (!(dma_m & (1 << dma_ps2.xfr_channel)))
                dma_ps2_run(dma_ps2.xfr_channel);
            break;

        default:
            fatal("Bad XFR command %i channel %i val %02x\n", dma_ps2.xfr_command, dma_ps2.xfr_channel, val);
        }
        break;

    default:
        break;
    }
}

uint8_t CDMA::dma16_read(uint16_t addr)
{
    int     channel = ((addr >> 2) & 3) + 4;
#ifdef ENABLE_DMA_LOG
    uint16_t port = addr;
#endif
    uint8_t ret;
    int count;

    addr >>= 1;

    ret = dmaregs[1][addr & 0xf];

    switch (addr & 0xf) {
    case 0:
    case 2:
    case 4:
    case 6: /*Address registers*/
        dma_wp[1] ^= 1;
        if (dma_ps2.is_ps2) {
            if (dma_wp[1])
                ret = (dma[channel].ac);
            else
                ret = ((dma[channel].ac >> 8) & 0xff);
        }
        else if (dma_wp[1])
            ret = ((dma[channel].ac >> 1) & 0xff);
        else
            ret = ((dma[channel].ac >> 9) & 0xff);
        break;

    case 1:
    case 3:
    case 5:
    case 7: /*Count registers*/
        dma_wp[1] ^= 1;
        count = dma[channel].cc/* + 1*/;
        if (dma_wp[1])
            ret = count & 0xff;
        else
            ret = count >> 8;
        break;

    case 8: /*Status register*/
        ret = (dma_stat_rq_pc & 0xf0);
        ret |= dma_stat >> 4;
        dma_stat &= ~0xf0;
        break;

    default:
        break;
    }

    return ret;
}

void CDMA::dma16_write(uint16_t addr, uint8_t val)
{
    int channel = ((addr >> 2) & 3) + 4;

    addr >>= 1;

    dmaregs[1][addr & 0xf] = val;
    switch (addr & 0xf) {
    case 0:
    case 2:
    case 4:
    case 6: /*Address registers*/
        dma_wp[1] ^= 1;
        if (dma_ps2.is_ps2) {
            if (dma_wp[1])
                dma[channel].ab = (dma[channel].ab & 0xffffff00 & dma_mask) | val;
            else
                dma[channel].ab = (dma[channel].ab & 0xffff00ff & dma_mask) | (val << 8);
        }
        else {
            if (dma_wp[1])
                dma[channel].ab = (dma[channel].ab & 0xfffffe00 & dma_mask) | (val << 1);
            else
                dma[channel].ab = (dma[channel].ab & 0xfffe01ff & dma_mask) | (val << 9);
        }
        dma[channel].ac = dma[channel].ab;
        return;

    case 1:
    case 3:
    case 5:
    case 7: /*Count registers*/
        dma_wp[1] ^= 1;
        if (dma_wp[1])
            dma[channel].cb = (dma[channel].cb & 0xff00) | val;
        else
            dma[channel].cb = (dma[channel].cb & 0x00ff) | (val << 8);
        dma[channel].cc = dma[channel].cb;
        return;

    case 8: /*Control register*/
        return;

    case 9: /*Request register */
        channel = (val & 3) + 4;
        if (val & 4) {
            dma_stat_rq_pc |= (1 << channel);
            dma_block_transfer(channel);
        }
        else
            dma_stat_rq_pc &= ~(1 << channel);
        break;

    case 0xa: /*Mask*/
        channel = (val & 3);
        if (val & 4)
            dma_m |= (0x10 << channel);
        else
            dma_m &= ~(0x10 << channel);
        return;

    case 0xb: /*Mode*/
        channel = (val & 3) + 4;
        dma[channel].mode = val;
        if (dma_ps2.is_ps2) {
            dma[channel].ps2_mode &= ~0x1c;
            if (val & 0x20)
                dma[channel].ps2_mode |= 0x10;
            if ((val & 0xc) == 8)
                dma[channel].ps2_mode |= 4;
            else if ((val & 0xc) == 4)
                dma[channel].ps2_mode |= 0xc;
        }
        return;

    case 0xc: /*Clear FF*/
        dma_wp[1] = 0;
        return;

    case 0xd: /*Master clear*/
        dma_wp[1] = 0;
        dma_m |= 0xf0;
        dma_stat_rq_pc &= ~0xf0;
        return;

    case 0xe: /*Clear mask*/
        dma_m &= 0x0f;
        return;

    case 0xf: /*Mask write*/
        dma_m = (dma_m & 0x0f) | ((val & 0xf) << 4);
        return;

    default:
        break;
    }
}

#define CHANNELS               \
    {                          \
        8, 2, 3, 1, 8, 8, 8, 0 \
    }

void CDMA::dma_page_write(uint16_t addr, uint8_t val)
{
    uint8_t convert[8] = CHANNELS;

#ifdef USE_DYNAREC
    if ((addr == 0x84) && cpu_use_dynarec)
        update_tsc();
#endif

    addr &= 0x0f;
    dmaregs[2][addr] = val;

    {
        if (addr >= 8)
            addr = convert[addr & 0x07] | 4;
        else
            addr = convert[addr & 0x07];
    }

    if (addr < 8) {
        dma[addr].page_l = val;

        if (addr > 4) {
            dma[addr].page = val & 0xfe;
            dma[addr].ab = (dma[addr].ab & 0xff01ffff & dma_mask) | (dma[addr].page << 16);
            dma[addr].ac = (dma[addr].ac & 0xff01ffff & dma_mask) | (dma[addr].page << 16);
        }
        else {
            dma[addr].page = dma_at ? val : val & 0xf;
            dma[addr].ab = (dma[addr].ab & 0xff00ffff & dma_mask) | (dma[addr].page << 16);
            dma[addr].ac = (dma[addr].ac & 0xff00ffff & dma_mask) | (dma[addr].page << 16);
        }
    }
}

uint8_t CDMA::dma_page_read(uint16_t addr)
{
    uint8_t convert[8] = CHANNELS;
    uint8_t ret = 0xff;

  {
        addr &= 0x0f;
        ret = dmaregs[2][addr];

        if (addr >= 8)
            addr = convert[addr & 0x07] | 4;
        else
            addr = convert[addr & 0x07];

        if (addr < 8)
            ret = dma[addr].page_l;
    }

    return ret;
}

void CDMA::dma_high_page_write(uint16_t addr, uint8_t val)
{
    uint8_t convert[8] = CHANNELS;

    addr &= 0x0f;

    if (addr >= 8)
        addr = convert[addr & 0x07] | 4;
    else
        addr = convert[addr & 0x07];

    if (addr < 8) {
        dma[addr].page_h = val;

        dma[addr].ab = ((dma[addr].ab & 0xffffff) | (dma[addr].page << 24)) & dma_mask;
        dma[addr].ac = ((dma[addr].ac & 0xffffff) | (dma[addr].page << 24)) & dma_mask;
    }
}

uint8_t CDMA::dma_high_page_read(uint16_t addr)
{
    uint8_t convert[8] = CHANNELS;
    uint8_t ret = 0xff;

    addr &= 0x0f;

    if (addr >= 8)
        addr = convert[addr & 0x07] | 4;
    else
        addr = convert[addr & 0x07];

    if (addr < 8)
        ret = dma[addr].page_h;

    return ret;
}

int CDMA::dma_sg(uint8_t* data, int transfer_length, int out, void* priv)
{
    dma_t* dev = (dma_t*)priv;
#ifdef ENABLE_DMA_LOG
    char* sop;
#endif

    int force_end = 0;
    int buffer_pos = 0;

#ifdef ENABLE_DMA_LOG
    sop = out ? "Read" : "Writ";
#endif

    if (!(dev->sg_status & 1))
        return 2; /*S/G disabled*/

    while (1) {
        if (dev->count <= transfer_length) {
            //dma_log("%sing %i bytes to %08X\n", sop, dev->count, dev->addr);
            if (out)
                theAli->do_pci_read(dev->addr, data + buffer_pos, 1, dev->count); // dma_bm_read(dev->addr, (uint8_t*)(data + buffer_pos), dev->count, 4);
            else
                theAli->do_pci_write(dev->addr, (uint8_t*)(data + buffer_pos), dev->count, 4);
            transfer_length -= dev->count;
            buffer_pos += dev->count;
        }
        else {
            //dma_log("%sing %i bytes to %08X\n", sop, transfer_length, dev->addr);
            if (out)
                theAli->do_pci_read(dev->addr, (uint8_t*)(data + buffer_pos), transfer_length, 4);
            else
                theAli->do_pci_write(dev->addr, (uint8_t*)(data + buffer_pos), transfer_length, 4);
            /* Increase addr and decrease count so that resumed transfers do not mess up. */
            dev->addr += transfer_length;
            dev->count -= transfer_length;
            transfer_length = 0;
            force_end = 1;
        }

        if (force_end) {
            //dma_log("Total transfer length smaller than sum of all blocks, partial block\n");
            return 1; /* This block has exhausted the data to transfer and it was smaller than the count, break. */
        }
        else {
            if (!transfer_length && !dev->eot) {
                //dma_log("Total transfer length smaller than sum of all blocks, full block\n");
                return 1; /* We have exhausted the data to transfer but there's more blocks left, break. */
            }
            else if (transfer_length && dev->eot) {
                //dma_log("Total transfer length greater than sum of all blocks\n");
                return 4; /* There is data left to transfer but we have reached EOT - return with error. */
            }
            else if (dev->eot) {
                //dma_log("Regular EOT\n");
                return 5; /* We have regularly reached EOT - clear status and break. */
            }
            else {
                /* We have more to transfer and there are blocks left, get next block. */
                dma_sg_next_addr(dev);
            }
        }
    }
}

uint8_t CDMA::_dma_read(uint32_t addr, dma_t* dma_c)
{
    uint8_t temp = 0;

    if (dma_advanced) {
        if (dma_c->sg_status & 1)
            dma_c->sg_status = (dma_c->sg_status & 0x0f) | (dma_sg(&temp, 1, 1, dma_c) << 4);
        else
            theAli->do_pci_read(addr, &temp, 1, 1);
    }
    else
        theAli->do_pci_read(addr, &temp, 1, 1);

    return temp;
}

uint16_t CDMA::_dma_readw(uint32_t addr, dma_t* dma_c)
{
    uint16_t temp = 0;

    if (dma_advanced) {
        if (dma_c->sg_status & 1)
            dma_c->sg_status = (dma_c->sg_status & 0x0f) | (dma_sg((uint8_t*)&temp, 2, 1, dma_c) << 4);
        else
            theAli->do_pci_read(addr, &temp, 2, 1); // dma_bm_read(addr, (uint8_t*)&temp, 2, dma_transfer_size(dma_c));
    }
    else
        temp = _dma_read(addr, dma_c) | (_dma_read(addr + 1, dma_c) << 8);

    return temp;
}

void CDMA::_dma_write(uint32_t addr, uint8_t val, dma_t* dma_c)
{
    if (dma_advanced) {
        if (dma_c->sg_status & 1)
            dma_c->sg_status = (dma_c->sg_status & 0x0f) | (dma_sg(&val, 1, 0, dma_c) << 4);
        else
            theAli->do_pci_write(addr, &val, 1, 1); // dma_bm_write(addr, &val, 1, dma_transfer_size(dma_c));
    }
    else {
        theAli->do_pci_write(addr, &val, 1, 1); // mem_writeb_phys(addr, val);
    }
}

void CDMA::_dma_writew(uint32_t addr, uint16_t val, dma_t* dma_c)
{
    if (dma_advanced) {
        if (dma_c->sg_status & 1)
            dma_c->sg_status = (dma_c->sg_status & 0x0f) | (dma_sg((uint8_t*)&val, 2, 0, dma_c) << 4);
        else
            theAli->do_pci_write(addr, &val, 2, 1);
    }
    else {
        _dma_write(addr, val & 0xff, dma_c);
        _dma_write(addr + 1, val >> 8, dma_c);
    }
}

void CDMA::dma_retreat(dma_t* dma_c)
{
    int as = dma_c->transfer_mode >> 8;

    if (dma->sg_status & 1) {
        dma_c->ac = (dma_c->ac - as) & dma_mask;

        dma_c->page = dma_c->page_l = (dma_c->ac >> 16) & 0xff;
        dma_c->page_h = (dma_c->ac >> 24) & 0xff;
    }
    else if (as == 2)
        dma_c->ac = ((dma_c->ac & 0xfffe0000) & dma_mask) | ((dma_c->ac - as) & 0x1ffff);
    else
        dma_c->ac = ((dma_c->ac & 0xffff0000) & dma_mask) | ((dma_c->ac - as) & 0xffff);
}

void CDMA::dma_advance(dma_t* dma_c)
{
    int as = dma_c->transfer_mode >> 8;

    if (dma->sg_status & 1) {
        dma_c->ac = (dma_c->ac + as) & dma_mask;

        dma_c->page = dma_c->page_l = (dma_c->ac >> 16) & 0xff;
        dma_c->page_h = (dma_c->ac >> 24) & 0xff;
    }
    else if (as == 2)
        dma_c->ac = ((dma_c->ac & 0xfffe0000) & dma_mask) | ((dma_c->ac + as) & 0x1ffff);
    else
        dma_c->ac = ((dma_c->ac & 0xffff0000) & dma_mask) | ((dma_c->ac + as) & 0xffff);
}

int CDMA::dma_channel_readable(int channel)
{
    dma_t* dma_c = &dma[channel];
    int      ret = 1;

    if (channel < 4) {
        if (dma_command[0] & 0x04)
            ret = 0;
    }
    else {
        if (dma_command[1] & 0x04)
            ret = 0;
    }

    if (!(dma_e & (1 << channel)))
        ret = 0;
    if ((dma_m & (1 << channel)) && !dma_req_is_soft)
        ret = 0;
    if ((dma_c->mode & 0xC) != 8)
        ret = 0;

    return ret;
}

int CDMA::dma_channel_read_only(int channel)
{
    dma_t* dma_c = &dma[channel];
    uint16_t temp;

    if (channel < 4) {
        if (dma_command[0] & 0x04)
            return (DMA_NODATA);
    }
    else {
        if (dma_command[1] & 0x04)
            return (DMA_NODATA);
    }

    if (!(dma_e & (1 << channel)))
        return (DMA_NODATA);
    if ((dma_m & (1 << channel)) && !dma_req_is_soft)
        return (DMA_NODATA);
    if ((dma_c->mode & 0xC) != 8)
        return (DMA_NODATA);

    dma_channel_advance(channel);

    if (!dma_c->size) {
        temp = _dma_read(dma_c->ac, dma_c);

        if (dma_c->mode & 0x20) {
            if (dma_ps2.is_ps2)
                dma_c->ac--;
            else if (dma_advanced)
                dma_retreat(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xffff0000 & dma_mask) | ((dma_c->ac - 1) & 0xffff);
        }
        else {
            if (dma_ps2.is_ps2)
                dma_c->ac++;
            else if (dma_advanced)
                dma_advance(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xffff0000 & dma_mask) | ((dma_c->ac + 1) & 0xffff);
        }
    }
    else {
        temp = _dma_readw(dma_c->ac, dma_c);

        if (dma_c->mode & 0x20) {
            if (dma_ps2.is_ps2)
                dma_c->ac -= 2;
            else if (dma_advanced)
                dma_retreat(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xfffe0000 & dma_mask) | ((dma_c->ac - 2) & 0x1ffff);
        }
        else {
            if (dma_ps2.is_ps2)
                dma_c->ac += 2;
            else if (dma_advanced)
                dma_advance(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xfffe0000 & dma_mask) | ((dma_c->ac + 2) & 0x1ffff);
        }
    }

    dma_stat_rq |= (1 << channel);

    dma_stat_adv_pend |= (1 << channel);

    return temp;
}

int CDMA::dma_channel_advance(int channel)
{
    dma_t* dma_c = &dma[channel];
    int      tc = 0;

    if (dma_stat_adv_pend & (1 << channel)) {
        dma_c->cc--;
        if (dma_c->cc < 0) {
            if (dma_advanced && (dma_c->sg_status & 1) && !(dma_c->sg_status & 6))
                dma_sg_next_addr(dma_c);
            else {
                tc = 1;
                if (dma_c->mode & 0x10) { /*Auto-init*/
                    dma_c->cc = dma_c->cb;
                    dma_c->ac = dma_c->ab;
                }
                else
                    dma_m |= (1 << channel);
                dma_stat |= (1 << channel);
            }
        }

        if (tc) {
            if (dma_advanced && (dma_c->sg_status & 1) && ((dma_c->sg_command & 0xc0) == 0x40)) {
                theAli->pic_interrupt(1, 13 - 8); // picint(1 << 13);
                dma_c->sg_status |= 8;
            }
        }

        dma_stat_adv_pend &= ~(1 << channel);
    }

    return tc;
}

int CDMA::dma_channel_read(int channel)
{
    dma_t* dma_c = &dma[channel];
    uint16_t temp;
    int      tc = 0;

    if (channel < 4) {
        if (dma_command[0] & 0x04)
            return (DMA_NODATA);
    }
    else {
        if (dma_command[1] & 0x04)
            return (DMA_NODATA);
    }

    if (!(dma_e & (1 << channel)))
        return (DMA_NODATA);
    if ((dma_m & (1 << channel)) && !dma_req_is_soft)
        return (DMA_NODATA);
    if ((dma_c->mode & 0xC) != 8)
        return (DMA_NODATA);

    if (dma_stat_adv_pend & (1 << channel))
        dma_channel_advance(channel);

    if (!dma_c->size) {
        temp = _dma_read(dma_c->ac, dma_c);

        if (dma_c->mode & 0x20) {
            if (dma_ps2.is_ps2)
                dma_c->ac--;
            else if (dma_advanced)
                dma_retreat(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xffff0000 & dma_mask) | ((dma_c->ac - 1) & 0xffff);
        }
        else {
            if (dma_ps2.is_ps2)
                dma_c->ac++;
            else if (dma_advanced)
                dma_advance(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xffff0000 & dma_mask) | ((dma_c->ac + 1) & 0xffff);
        }
    }
    else {
        temp = _dma_readw(dma_c->ac, dma_c);

        if (dma_c->mode & 0x20) {
            if (dma_ps2.is_ps2)
                dma_c->ac -= 2;
            else if (dma_advanced)
                dma_retreat(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xfffe0000 & dma_mask) | ((dma_c->ac - 2) & 0x1ffff);
        }
        else {
            if (dma_ps2.is_ps2)
                dma_c->ac += 2;
            else if (dma_advanced)
                dma_advance(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xfffe0000 & dma_mask) | ((dma_c->ac + 2) & 0x1ffff);
        }
    }

    dma_stat_rq |= (1 << channel);

    dma_c->cc--;
    if (dma_c->cc < 0) {
        if (dma_advanced && (dma_c->sg_status & 1) && !(dma_c->sg_status & 6))
            dma_sg_next_addr(dma_c);
        else {
            tc = 1;
            if (dma_c->mode & 0x10) { /*Auto-init*/
                dma_c->cc = dma_c->cb;
                dma_c->ac = dma_c->ab;
            }
            else
                dma_m |= (1 << channel);
            dma_stat |= (1 << channel);
        }
    }

    if (tc) {
        if (dma_advanced && (dma_c->sg_status & 1) && ((dma_c->sg_command & 0xc0) == 0x40)) {
            theAli->pic_interrupt(1, 13 - 8);
            dma_c->sg_status |= 8;
        }

        return (temp | DMA_OVER);
    }

    return temp;
}

int CDMA::dma_channel_write(int channel, uint16_t val)
{
    dma_t* dma_c = &dma[channel];

    if (channel < 4) {
        if (dma_command[0] & 0x04)
            return (DMA_NODATA);
    }
    else {
        if (dma_command[1] & 0x04)
            return (DMA_NODATA);
    }

    if (!(dma_e & (1 << channel)))
        return (DMA_NODATA);
    if ((dma_m & (1 << channel)) && !dma_req_is_soft)
        return (DMA_NODATA);
    if ((dma_c->mode & 0xC) != 4)
        return (DMA_NODATA);

    if (!dma_c->size) {
        _dma_write(dma_c->ac, val & 0xff, dma_c);

        if (dma_c->mode & 0x20) {
            if (dma_ps2.is_ps2)
                dma_c->ac--;
            else if (dma_advanced)
                dma_retreat(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xffff0000 & dma_mask) | ((dma_c->ac - 1) & 0xffff);
        }
        else {
            if (dma_ps2.is_ps2)
                dma_c->ac++;
            else if (dma_advanced)
                dma_advance(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xffff0000 & dma_mask) | ((dma_c->ac + 1) & 0xffff);
        }
    }
    else {
        _dma_writew(dma_c->ac, val, dma_c);

        if (dma_c->mode & 0x20) {
            if (dma_ps2.is_ps2)
                dma_c->ac -= 2;
            else if (dma_advanced)
                dma_retreat(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xfffe0000 & dma_mask) | ((dma_c->ac - 2) & 0x1ffff);
            dma_c->ac = (dma_c->ac & 0xfffe0000 & dma_mask) | ((dma_c->ac - 2) & 0x1ffff);
        }
        else {
            if (dma_ps2.is_ps2)
                dma_c->ac += 2;
            else if (dma_advanced)
                dma_advance(dma_c);
            else
                dma_c->ac = (dma_c->ac & 0xfffe0000 & dma_mask) | ((dma_c->ac + 2) & 0x1ffff);
        }
    }

    dma_stat_rq |= (1 << channel);

    dma_stat_adv_pend &= ~(1 << channel);

    dma_c->cc--;
    if (dma_c->cc < 0) {
        if (dma_advanced && (dma_c->sg_status & 1) && !(dma_c->sg_status & 6))
            dma_sg_next_addr(dma_c);
        else {
            if (dma_c->mode & 0x10) { /*Auto-init*/
                dma_c->cc = dma_c->cb;
                dma_c->ac = dma_c->ab;
            }
            else
                dma_m |= (1 << channel);
            dma_stat |= (1 << channel);
        }
    }

    if (dma_m & (1 << channel)) {
        if (dma_advanced && (dma_c->sg_status & 1) && ((dma_c->sg_command & 0xc0) == 0x40)) {
            theAli->pic_interrupt(1, 13 - 8);
            dma_c->sg_status |= 8;
        }

        return DMA_OVER;
    }

    return 0;
}

void CDMA::dma_ps2_run(int channel)
{
}
