# UART回显上板示例

使用Gowin EDA打开`uart_echo.gprj`，依次执行综合、布局布线和下载。

串口助手设置为`115200 8N1`并关闭本地回显。电脑发送的文本或十六进制数据应由FPGA原样返回。错误LED正常时熄灭，检测到接收错误后保持点亮，按复位键清除。

`acx720_uart_echo.sdc`把`i_clock`约束为50 MHz（20 ns周期），与RTL中的`CLK_FREQ_HZ=50_000_000`一致。

本示例的管脚和时钟约束适用于当前ACX720/GW5AT-138B板卡，移植到其他开发板时需要替换`.cst`并按实际输入频率修改`.sdc`和`CLK_FREQ_HZ`。
