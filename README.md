# A Self balancing bot based on ESP32 &MPU6050

## Codes :
The self_balance_bot_code folder contains the actual code. Just install the libraries and the code is good to run. MPU6050_kalman folder has a primitive code based on the kalman filter library , but that was replaced in the actual code with a dynamic complementary filter.
## Node-red flows :
*flows.JSON* contains the node-red flow configurations to interact with the *mosquitto* MQTT server setup on  my local machine.
## Fritzing Sketch : 
The .fzz fritzing sketch of the bot is available. Only the schematic diagram is properly routed.

## About the Dynamic Complementary Filter :
Normally the complementary filter  combines the gyro and accelerometer values through a constant k & its complementary (1-k). 
```python
gyro_angle_x = (1 - k )*gyro_angle_x + k * acc_angle_x
```
But this filter can be unstable when arbitrary  acceleration is introduced in the system and most of the times acceleration is hard to predict.
Thus in the filter implementation I've done in my code dynamically reduces the constant k based on the acceleration and the change in acceleration. The function which varies the k constant is a bell curve i.e.  $y=e^{b(x-c)^2}$  where b determines the slope of the decrement , c determines the calibrated value of acceleration/jerk where k should be maximum i.e the function returns 1.0 .