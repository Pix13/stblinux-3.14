#ifndef __NEXUS_INTERNAL_GPIO_H
#define __NEXUS_INTERNAL_GPIO_H

#define GPIO_DT_COMPAT	"brcm,brcmstb-gpio"
#define GIO_BANK_SIZE	0x20

int brcmstb_gpio_request(unsigned int gpio);
int brcmstb_gpio_find_by_addr(uint32_t addr, unsigned int shift);

#endif /* __NEXUS_INTERNAL_GPIO_H */
