#include "drivers.h"
extern volatile unsigned char tx_done_flag;

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN) {
        if (rf_irq_src & FLD_RF_IRQ_TX) {
            tx_done_flag = 1;
        }
        rf_irq_clr_src(FLD_RF_IRQ_ALL);
    }
    irq_clr_src2(FLD_IRQ_ALL);
}
