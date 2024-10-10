#include "arch/x86/isr.h"
#include "arch/x86/ports.h"
#include "arch/x86/timer.h"
#include "drivers/screen.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PORT_DATA(CHANNEL) ((CHANNEL)->port_base + 0)
#define PORT_ERROR(CHANNEL) ((CHANNEL)->port_base + 1)
#define PORT_SECTORCOUNT(CHANNEL) ((CHANNEL)->port_base + 2)
#define PORT_LBA_LO(CHANNEL) ((CHANNEL)->port_base + 3)
#define PORT_LBA_MID(CHANNEL) ((CHANNEL)->port_base + 4)
#define PORT_LBA_HI(CHANNEL) ((CHANNEL)->port_base + 5)
#define PORT_DRIVE_SELECT(CHANNEL) ((CHANNEL)->port_base + 6)
#define PORT_STATUS(CHANNEL) ((CHANNEL)->port_base + 7)
#define PORT_COMMAND(CHANNEL) ((CHANNEL)->port_base + 7)

#define PORT_CONTROL(CHANNEL) ((CHANNEL)->port_base + 0x206)
#define PORT_ALTERNATIVE_STATUS(CHANNEL) ((CHANNEL)->port_base + 0x206)

// NOT: The value of Alternate Status is always the same as the
// Regular Status port (0x1F7 on the Primary bus), but reading the
// Alternate Status port does not affect interrupts.

#define SELECT_MASTER 0xa0
#define SELECT_SLAVE 0xb0

#define CMD_IDENTIFY 0xec

#define STATUS_BSY 0x80
#define STATUS_DRQ 0x08
#define STATUS_ERR 0x01

#define BLOCK_SIZE_SECTOR 512

#define MAX_BUSY_WAIT_TIME 3000

// Support the two "legacy" ATA channels (bus) found in a standard PC.
// The first two buses are called the Primary and Secondary ATA bus.
// They are almost always controlled by the IO ports PORT_BASE_PRIMARY and
// PORT_BASE_SECONDARY unless you change it. If the next two buses exist,
// they are norammly controlled by IO ports 0x1e8-0x1ef and 0x168-0x16f,
// respectively.
#define N_CHANNELS 2
// A channel has two devices, the master and the slave.
#define N_DEVICES_PER_CHANNEL 2

#define PORT_BASE_PRIMARY 0x1f0
#define PORT_BASE_SECONDARY 0x170
// The IRQ for the Primary bus is IRQ14.
#define IRQ_PRIMARY 14
// The IRQ for the Secondary bus is IRQ15.
#define IRQ_SECONDARY 15

// According to specs, the PCI disk controller is supposed to be in
// "Legacy/Compatibility" mode when the system boots. The system "should"
// use these standardized IO port settings defined above.

typedef struct ata_device {
  char name[8];
  int id;
  struct ata_channel *channel;
  bool is_ata_disk;
} ata_device;

typedef struct ata_channel {
  char name[8];
  uint16_t port_base;
  uint8_t irq;
  ata_device devices[N_DEVICES_PER_CHANNEL];
} ata_channel;

static ata_channel channels[N_CHANNELS];

void ata_select_device(const ata_device *device) {
  // kprintf("selecting device %d\n", device->id);
  uint16_t id = device->id == 1 ? SELECT_MASTER : SELECT_SLAVE;
  port_byte_out(PORT_DRIVE_SELECT(device->channel), id);
  port_byte_in(PORT_ALTERNATIVE_STATUS(device->channel));
  // TOOD: We should wait 400 ns here. But, we currently only have msleep.
  timer_msleep(1);
}

// The controller is idle when the BSY and DRQ bits are cleared.
void wait_until_idle(const ata_device *device) {
  for (size_t i = 0; i < 1000; i++) {
    uint16_t status = port_byte_in(PORT_ALTERNATIVE_STATUS(device->channel));
    if ((status & (STATUS_BSY | STATUS_DRQ)) == 0) {
      return;
    }
    timer_msleep(10);
  }
}

void sector_in(const ata_device *device, void *sector) {
  insw(PORT_DATA(device->channel), sector, BLOCK_SIZE_SECTOR / 2);
}

bool wait_while_busy(const ata_device *device) {
  // For any other value, poll the status port until bit 7 clears.
  for (size_t i = 0; i < MAX_BUSY_WAIT_TIME; i++) {
    uint8_t status = port_byte_in(PORT_ALTERNATIVE_STATUS(device->channel));
    if (!(status & STATUS_BSY)) {
      // Some ATAPI drives do not follow spec... So we need to check the
      // LBA_MID and LBA_HI ports to see if they are non-zero. If they are
      // the drive is not ATA.
      uint8_t lba_mid = port_byte_in(PORT_LBA_MID(device->channel));
      uint8_t lba_hi = port_byte_in(PORT_LBA_HI(device->channel));
      if (lba_mid > 0 || lba_hi > 0) {
        kprint("not an ata device\n");
        return false;
      }

      status = port_byte_in(PORT_ALTERNATIVE_STATUS(device->channel));
      if (status & STATUS_ERR) {
        // Failed to read disk.
        kprint("failed to read disk\n");
        return false;
      }

      return (status & STATUS_DRQ) != 0;
    }
    timer_msleep(10);
  }
  kprintf("done busy waiting\n");
  return false;
}

static void reset_channel(ata_channel *channel) {
  bool present[2];

  for (size_t i = 0; i < 2; i++) {
    ata_device *device = &channel->devices[i];

    ata_select_device(device);

    port_byte_out(PORT_SECTORCOUNT(channel), 0x55);
    port_byte_out(PORT_LBA_LO(channel), 0xaa);

    port_byte_out(PORT_SECTORCOUNT(channel), 0xaa);
    port_byte_out(PORT_LBA_LO(channel), 0x55);

    port_byte_out(PORT_SECTORCOUNT(channel), 0x55);
    port_byte_out(PORT_LBA_LO(channel), 0xaa);

    uint16_t sectorcount = port_byte_in(PORT_SECTORCOUNT(channel));
    uint16_t lba_lo = port_byte_in(PORT_LBA_LO(channel));

    present[i] = (sectorcount == 0x55) && (lba_lo == 0xaa);
  }

  port_byte_out(PORT_CONTROL(channel), 0);
  timer_msleep(10);
  port_byte_out(PORT_CONTROL(channel), 0x04);
  timer_msleep(10);
  port_byte_out(PORT_CONTROL(channel), 0);

  timer_msleep(150);

  if (present[0]) {
    ata_select_device(&channel->devices[0]);
    wait_while_busy(&channel->devices[0]);
  }

  if (present[1]) {
    ata_select_device(&channel->devices[1]);
    for (size_t i = 0; i < 3000; i++) {
      uint16_t sectorcount = port_byte_in(PORT_SECTORCOUNT(channel));
      uint16_t lba_lo = port_byte_in(PORT_LBA_LO(channel));
      if (sectorcount == 1 && lba_lo == 1) {
        break;
      }
      timer_msleep(10);
    }

    bool status = wait_while_busy(&channel->devices[1]);
  }
}

// All current BIOSes have standardized the use of the IDENTIFY command
// to detect the existence of ATA bus devices, e.g., PATA, PATAPI, SATAPI, SATA.
void ata_identify_device(ata_device *device) {
  ata_channel *channel = device->channel;
  uint8_t sector[BLOCK_SIZE_SECTOR];

  // To use the IDENTIFY command,
  // 1. Select the correct device.
  wait_until_idle(device);
  ata_select_device(device);
  wait_until_idle(device);

  // 2. Set the SECTORCOUNT, LBA_LO, LBA_MID and LBA_HI ports to 0.
  port_byte_out(PORT_SECTORCOUNT(channel), 0);
  port_byte_out(PORT_LBA_LO(channel), 0);
  port_byte_out(PORT_LBA_MID(channel), 0);
  port_byte_out(PORT_LBA_HI(channel), 0);

  // 3. Send the IDENTIFY command (CMD_IDENTIFY) to the command port.
  port_byte_out(PORT_COMMAND(channel), CMD_IDENTIFY);

  // 4. Read the status port (this is the same as the command port).
  uint8_t status = port_byte_in(PORT_ALTERNATIVE_STATUS(channel));
  // If the status is 0, the drive does not exist.
  if (status == 0) {
    // Does not exist
    kprint("drive does not exist\n");
    return;
  }

  if (!wait_while_busy(device)) {
    device->is_ata_disk = false;
    return;
  }

  // Data is ready to be sent. Read 256 16-bit values from the data port
  // and store that information.
  sector_in(device, sector);
  kprint(sector);
  kprint("\n");
  uint32_t capacity = *(uint32_t *)&sector[60 * 2];
  kprintf("capacity %d\n", capacity);
}

static void interrupt_handler(registers_t *regs) {
  for (size_t i = 0; i < 2; i++) {
    ata_channel *channel = &channels[i];
    if (regs->int_no != channel->irq) {
      continue;
    }
    port_byte_in(PORT_STATUS(channel));
    kprint("ata interrupt\n");
  }
}

void ata_init() {
  // Initialize channels
  for (size_t i = 0; i < N_CHANNELS; i++) {
    kprintf("setting up channel %d\n", i);
    ata_channel *channel = &channels[i];

    // Set base port address and IRQ
    if (i == 0) {
      channel->port_base = PORT_BASE_PRIMARY;
      channel->irq = IRQ_PRIMARY + 0x20;
    } else {
      channel->port_base = PORT_BASE_SECONDARY;
      channel->irq = IRQ_SECONDARY + 0x20;
    }

    // Initialize devices.
    for (size_t i = 0; i < N_DEVICES_PER_CHANNEL; i++) {
      ata_device *device = &channel->devices[i];

      device->channel = channel;
      device->id = i;
    }

    // Register interrupt handler.
    register_interrupt_handler(channel->irq, interrupt_handler);

    // Reset hardware.
    // reset_channel(channel);

    // Read hard disk identity information.
    for (size_t i = 0; i < N_DEVICES_PER_CHANNEL; i++) {
      ata_device *device = &channel->devices[i];
      ata_identify_device(device);
    }
  }
}