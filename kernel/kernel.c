#include "arch/x86/isr.h"
#include "arch/x86/timer.h"
#include "devices/ata.h"
#include "drivers/keyboard.h"
#include "drivers/screen.h"

void kernel_main() {
  isr_install();
  asm volatile("sti");
  clear_screen();
  init_keyboard();
  timer_init();
  ata_init();

  while (1) {
    // timer_msleep(100);
    // kprint("Hello\n");
  }
}