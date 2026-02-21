# stm32_i2c 使用教程
  call I2C_init -> call I2C_Start_send_bit -> call I2C_send_long_byte -> call I2C_Stop_send_bit
## stm32_i2c I2C_init
  - 用于配置相关引脚 I2C_Pin_Content I2C_init(GPIO_TypeDef *GPIOX, uint16_t SCL, uint16_t SDA, uint16_t RST
  GPIOX = \{GPIOA|GPIOB|GPIOC\}
  SCL = iic的时钟\(SCL\)引脚
  SDA = iic的数据引脚
  RST = iic的重置引脚没写实现需要读取外部实现重置
  - 返回值为数据结构 I2C_Pin_Content
  手动设置一个变量然后使用该函数给这个变量初始化即可
## I2C_LOOP_nop
  实现us的延时一般来说用户没有用
## I2C_Start_send_bit
  I2C的开始数据包在发送bit数据的时候需要先调用此函数，用来开启iic传输
## I2C_send_one_byte
  用于向从机发送字节data为一个8位的数据，
  ### 第一个数据要用从机地址左移然后加上读写位
  ps：
    '''c
    uint8_t address = 0b01110000;
    address = \(address << 1\) + 0/1(w\(写\)=0、r\(读\)=1);
    call I2C_send_one_byte
    '''
## I2C_Stop_send_bit
  结束iic通信在I2C_send_one_byte发送完毕使用结束iic的通信
## I2C_send_long_byte
  发送长数据用数组表示，数组第一位要是从机地址
