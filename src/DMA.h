/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * WWW    : http://es40.org
 * E-mail : camiel@es40.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Although this is not required, the author would appreciate being notified of,
 * and receiving any modifications you may make to the source code that might serve
 * the general public.
 */

 /**
  * \file
  * Contains the definitions for the emulated DMA controller.
  *
  * $Id$
  *
  * X-1.5        Camiel Vanderhoeven                             29-APR-2008
  *      Removed unused reference to floppy disk image.
  *
  * X-1.4        Brian Wheeler                                   29-APR-2008
  *      Fixed floppy disk implementation.
  *
  * X-1.3        Brian Wheeler                                   18-APR-2008
  *      Rewrote DMA code to make it ready for floppy support.
  *
  * X-1.2        Camiel Vanderhoeven                             14-MAR-2008
  *      Formatting.
  *
  * X-1.1        Camiel Vanderhoeven                             26-FEB-2008
  *      Created. Contains code previously found in AliM1543C.h
  *
  * \author Camiel Vanderhoeven (camiel@camicom.com / http://www.camicom.com)
  **/
#if !defined(INCLUDED_DMA_H)
#define INCLUDED_DMA_H

#include "SystemComponent.h"

  /**
   * \brief Emulated DMA controller.
   **/

#define DMA_NODATA -1
#define DMA_OVER   0x10000
#define DMA_VERIFY 0x20000


typedef struct dma_t {
  uint8_t  m;
  uint8_t  mode;
  uint8_t  page;
  uint8_t  stat;
  uint8_t  stat_rq;
  uint8_t  command;
  uint8_t  ps2_mode;
  uint8_t  arb_level;
  uint8_t  sg_command;
  uint8_t  sg_status;
  uint8_t  ptr0;
  uint8_t  enabled;
  uint8_t  ext_mode;
  uint8_t  page_l;
  uint8_t  page_h;
  uint8_t  pad;
  uint16_t cb;
  uint16_t io_addr;
  uint16_t base;
  uint16_t transfer_mode;
  uint32_t ptr;
  uint32_t ptr_cur;
  uint32_t addr;
  uint32_t ab;
  uint32_t ac;
  int      cc;
  int      wp;
  int      size;
  int      count;
  int      eot;
} dma_t;

class CDMA : public CSystemComponent
{
public:
  dma_t dma[8];
  uint8_t dma_e;
  uint8_t dma_m;

  virtual int   RestoreState(FILE* f) override
  {
    return 0;
  }
  virtual int   SaveState(FILE* f) override
  {
    return 0;
  }

  CDMA(CConfigurator* cfg, CSystem* c);
  virtual       ~CDMA() {}

  virtual int   DoClock() { return 0;  }
  virtual void  WriteMem(int index, u64 address, int dsize, u64 data);
  virtual u64   ReadMem(int index, u64 address, int dsize);

  void          send_data(int channel, void* data);
  void          recv_data(int channel, void* data);
  int           get_count(int channel) { return dma[channel].cc; };
  size_t        get_transfer_size(int channel) { return (size_t)dma[channel].cc + 1; };


  int  dma_get_drq(int channel);
  void dma_set_drq(int channel, int set);
  void dma_set_params(uint8_t advanced, uint32_t mask);

  void dma_set_mask(uint32_t mask);
  void dma_set_at(uint8_t at);

  void dma_ext_mode_init(void);
  void dma_high_page_init(void);

  void dma_remove_sg(void);
  void dma_set_sg_base(uint8_t sg_base);
  int  dma_channel_readable(int channel);
  
  int dma_channel_read_only(int channel);
  int dma_channel_advance(int channel);
  int dma_channel_read(int channel);
  int dma_channel_write(int channel, uint16_t val);
  void dma_reset(void);


private:
  int      dma_sg(uint8_t* data, int transfer_length, int out, void* priv);
  void     dma_ps2_run(int channel);
  int      dma_transfer_size(dma_t* dev);
  void     dma_sg_next_addr(dma_t* dev);
  void     dma_block_transfer(int channel);
  void     dma_mem_to_mem_transfer(void);
  void     dma_sg_write(uint16_t port, uint8_t val, void* priv);
  void     dma_sg_writew(uint16_t port, uint16_t val, void* priv);
  void     dma_sg_writel(uint16_t port, uint32_t val, void* priv);
  uint8_t  dma_sg_read(uint16_t port, void* priv);
  uint16_t dma_sg_readw(uint16_t port, void* priv);
  uint32_t dma_sg_readl(uint16_t port, void* priv);
  void     dma_ext_mode_write(uint16_t addr, uint8_t val);
  uint8_t  dma_sg_int_status_read(uint16_t addr);
  uint8_t  dma_read(uint16_t addr);
  void     dma_write(uint16_t addr, uint8_t val);
  uint8_t  dma_ps2_read(uint16_t addr);
  void     dma_ps2_write(uint16_t addr, uint8_t val);
  uint8_t  dma16_read(uint16_t addr);
  void     dma16_write(uint16_t addr, uint8_t val);
  void     dma_page_write(uint16_t addr, uint8_t val);
  uint8_t  dma_page_read(uint16_t addr);
  void     dma_high_page_write(uint16_t addr, uint8_t val);
  uint8_t  dma_high_page_read(uint16_t addr);

  uint8_t  _dma_read(uint32_t addr, dma_t* dma_c);
  uint16_t _dma_readw(uint32_t addr, dma_t* dma_c);
  void     _dma_write(uint32_t addr, uint8_t val, dma_t* dma_c);
  void     _dma_writew(uint32_t addr, uint16_t val, dma_t* dma_c);
  void     dma_retreat(dma_t* dma_c);
  void     dma_advance(dma_t* dma_c);


  uint8_t  dmaregs[3][16];
  int      dma_wp[2];
  uint8_t  dma_stat;
  uint8_t  dma_stat_rq;
  uint8_t  dma_stat_rq_pc;
  uint8_t  dma_stat_adv_pend;
  uint8_t  dma_command[2];
  uint8_t  dma_req_is_soft;
  uint8_t  dma_advanced;
  uint8_t  dma_at;
  uint8_t  dma_buffer[65536];
  uint16_t dma_sg_base;
  uint16_t dma16_buffer[65536];
  uint32_t dma_mask;

  struct dma_ps2_t {
    int xfr_command;
    int xfr_channel;
    int byte_ptr;

    int is_ps2;
  } dma_ps2;

#define DMA_PS2_IOA            (1 << 0)
#define DMA_PS2_AUTOINIT       (1 << 1)
#define DMA_PS2_XFER_MEM_TO_IO (1 << 2)
#define DMA_PS2_XFER_IO_TO_MEM (3 << 2)
#define DMA_PS2_XFER_MASK      (3 << 2)
#define DMA_PS2_DEC2           (1 << 4)
#define DMA_PS2_SIZE16         (1 << 6)
};

#define DMA_IO_BASE 0x1000
#define DMA0_IO_MAIN DMA_IO_BASE + 0
#define DMA1_IO_MAIN DMA_IO_BASE + 1
#define DMA_IO_LPAGE DMA_IO_BASE + 2
#define DMA_IO_HPAGE DMA_IO_BASE + 3
#define DMA0_IO_CHANNEL DMA_IO_BASE + 4
#define DMA1_IO_CHANNEL DMA_IO_BASE + 5
#define DMA0_IO_EXT DMA_IO_BASE + 6
#define DMA1_IO_EXT DMA_IO_BASE + 7
#define DMA_ALIAS_1 DMA_IO_BASE + 8
#define DMA_ALIAS_2 DMA_IO_BASE + 9
#define DMA_SG_1 DMA_IO_BASE + 10
#define DMA_SG_2 DMA_IO_BASE + 11
#define DMA_SG_3 DMA_IO_BASE + 12
#define DMA_SG_4 DMA_IO_BASE + 13

extern CDMA* theDMA;

#endif // !defined(INCLUDED_DMA_H)
