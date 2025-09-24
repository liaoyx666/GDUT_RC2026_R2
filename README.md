# 1.开发指南

## 1.1.CAN

### 1.1.1.开启一个can外设

1. 首先在`cubeMX`里初始化波特率为1M

2. 实例化`Can`类，传入相应的设备句柄

    ```
    can::Can can1(hfdcan1);
    ```

    

3. 初始化过滤器并启动can外设

    ```
     can1.Can_Filter_Init(FDCAN_STANDARD_ID, 1, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
     can1.Can_Filter_Init(FDCAN_STANDARD_ID, 2, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
     can1.Can_Start();
    ```

    

### 1.1.1.添加一个使用can的类（如电机）

1. 创建类时公有继承`CanHandler`

    ```
    class DjiMotor : public Motor, public can::CanHandler, public tim::TimHandler
    ```

2. 



# 2.使用指南









