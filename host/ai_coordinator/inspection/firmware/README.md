# ACG720 FX2 运行固件

来源：本机开发板厂商配套资料 `ACG720` 第65章“USB2.0的FPGA控制SlaveFIFO回送测试”工程，原文件 `slave_for_adc.iic`。教程65.3.3要求先向USB芯片RAM加载该固件，再进行FPGA Slave FIFO回送。原始二进制按原样保存，不改EEPROM。

原文件SHA256：`0a11e7a3de3cbcc2feafafbb7077bfadf191e4a2f23fa70b5edf87c15ee60e40`。

`fx2_firmware.reviewed()`仅接受上述哈希及固定IIC记录合同，在运行RAM记录中把8051地址`0x08de`的一字节立即数从`0xe3`改为`0xa3`，对应唯一的`MOV DPTR,#0xe601; MOV A,#IFCONFIG; MOVX @DPTR,A`。只清除IFCONFIG的48MHz选择位，保留内部时钟源、时钟输出、同步Slave FIFO等设置；8051 CPU时钟不变。运行记录SHA256：`e003178b3368aa5c916ea8c40bb584a6894d5e821832f0189a7c6f5a907e31f4`。

选择30MHz是为了满足FX2内部时钟模式同步读18.7ns的建立时间，并保留FPGA时钟树、I/O与板级传播余量。USB仍为480Mbps；当前每4接口时钟传一个16位字的上限为15MB/s，816×420 RGB565的10fps有效载荷约6.85MB/s。实际性能需完整帧联调确认。

默认CLI只离线检查。`--load-volatile --config ...`才执行RAM下载：A0写CPUCS暂停，分块写入0x0000～0x3fff内部程序RAM，再释放CPUCS。记录不允许重叠、越界或其他寄存器写入；不存在EEPROM命令。掉电后恢复板上原有启动固件。描述符重现不是成功凭证，后续必须通过带FPGA特征变换和独立IFCLK测频的诊断响应。

时序依据：[Infineon 38-08032 Rev.AD](https://www.infineon.com/assets/row/public/documents/24/49/infineon-cy7c68013a-cy7c68014a-cy7c68015a-cy7c68016a-ez-usb-fx2lp-usb-microcontroller-high-speed-usb-peripheral-controller-datasheet-en.pdf)，表20、23、26、31。厂商固件保留其原有权利；本项目只维护加载器、严格校验和上述单字节运行配置。
