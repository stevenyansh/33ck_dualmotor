1.硬件上两路spi，加锁两个可以正常读，不加锁会崩
2.spi引脚配置:
MOSI改成Z（pin 21 PR76）
CLK不变（pin 12 RD 13）
MISO（pin48 RP40）