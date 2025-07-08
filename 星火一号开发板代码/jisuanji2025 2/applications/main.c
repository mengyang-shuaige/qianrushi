#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <msh.h>
#include <sys/socket.h>
#include "netdb.h"
#include <wlan_mgnt.h>
#include <wlan_prot.h>
#include <wlan_cfg.h>
#include <stdio.h>
#include <stdlib.h>
#include <rttlogo.h>
#include <rtthread.h>
#include "board.h"
#include "sensor.h"
#include "sensor_hc_sr04.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

char *recv_data;


#define SR04_TRIG_PIN GET_PIN(B, 10)
#define SR04_ECHO_PIN GET_PIN(B, 11)
//#define PIN_BEEP2                GET_PIN(G, 2)//测距


#define PWM_DEV_NAME "pwm2"    // PWM设备名称（TIM2）
#define PWM_CHANNEL 4          // 通道号
#define PWM_PERIOD 20000000    // 周期20ms（单位：纳秒）
#define PWM_PULSE_90 1500000   // 90度对应脉冲宽度1.5ms
#define PWM_PULSE_0 500000     // 0度对应脉冲宽度0.5ms
#define PWM_PULSE_180 2500000  // 180度对应脉冲宽度2.5ms
static struct rt_device_pwm *pwm_dev;



/* 配置 LED 灯引脚 */
#define PIN_BEEP1                GET_PIN(G, 1)//电动车
#define PIN_BEEP2                GET_PIN(G, 2)//测距
#define PIN_BEEP3                GET_PIN(G, 3)//火焰
#define PIN_BEEP4                GET_PIN(G, 4)//水位
#define PIN_BEEP5                GET_PIN(G, 5)//温湿度
#define PIN_BEEP6                GET_PIN(G, 6)//烟雾
#define PIN_BEEP7                GET_PIN(G, 7)//充电桩温度

#define PIN_BEEP8                GET_PIN(D, 8)//水泵

#define PIN_LED_B                GET_PIN(B, 6)//小灯泡1




static rt_mutex_t sensor_mutex = RT_NULL;
static rt_uint32_t flame_voltage_global = 0;
static rt_uint32_t water_voltage_global = 0;
static rt_uint32_t smoke_voltage_global = 0;


static int on = 0;
static rt_mutex_t on_mutex = RT_NULL;


static volatile int sr04_distance = 0;
static rt_mutex_t sr04_mutex = RT_NULL;


int sr04_read_distance_sample(void);
int rt_hw_sr04_port(void);



#define IR_SENSOR_PIN    GET_PIN(D, 10)  // 假设传感器OUT接PF12

int ir_sensor_init(void) {
    rt_pin_mode(IR_SENSOR_PIN, PIN_MODE_INPUT_PULLUP); // 上拉输入模式
    return RT_EOK;
}



#define WLAN_SSID "redian"
#define WLAN_PASSWORD "88888888"

#define NET_READY_TIME_OUT (rt_tick_from_millisecond(15 * 1000))
#define BUFSZ   1024
static const char send_data[] = "This is TCP Client from RT-Thread.";

static void print_wlan_information(struct rt_wlan_info *info,int index);
static int wifi_autoconnect(void);

static struct rt_semaphore net_ready;
static struct rt_semaphore scan_done;

void wlan_scan_report_hander(int event,struct rt_wlan_buff *buff,void *parameter)
{
    struct rt_wlan_info *info = RT_NULL;
    int index = 0;
    RT_ASSERT(event == RT_WLAN_EVT_SCAN_REPORT);
    RT_ASSERT(buff != RT_NULL);
    RT_ASSERT(parameter != RT_NULL);

    info = (struct rt_wlan_info *)buff->data;
    index = *((int *)(parameter));
    print_wlan_information(info,index);
    ++ *((int *)(parameter));
}

void wlan_scan_done_hander(int event,struct rt_wlan_buff *buff,void *parameter)
{
    RT_ASSERT(event == RT_WLAN_EVT_SCAN_DONE);
    rt_sem_release(&scan_done);
}

void wlan_ready_handler(int event, struct rt_wlan_buff *buff, void *parameter)
{
    rt_sem_release(&net_ready);
}

/* 断开连接回调函数 */
void wlan_station_disconnect_handler(int event, struct rt_wlan_buff *buff, void *parameter)
{
    LOG_I("disconnect from the network!");
}

static void wlan_connect_handler(int event, struct rt_wlan_buff *buff, void *parameter)
{
    rt_kprintf("%s\n", __FUNCTION__);
    if ((buff != RT_NULL) && (buff->len == sizeof(struct rt_wlan_info)))
    {
        rt_kprintf("ssid : %s \n", ((struct rt_wlan_info *)buff->data)->ssid.val);
    }
}

static void wlan_connect_fail_handler(int event, struct rt_wlan_buff *buff, void *parameter)
{
    rt_kprintf("%s\n", __FUNCTION__);
    if ((buff != RT_NULL) && (buff->len == sizeof(struct rt_wlan_info)))
    {
        rt_kprintf("ssid : %s \n", ((struct rt_wlan_info *)buff->data)->ssid.val);
    }
}

void blink_leds(int times) {
    int rs = 0;
    while (rs < times) {
        rt_pin_write(PIN_LED_B, PIN_LOW);
        rt_thread_mdelay(200);
        rt_pin_write(PIN_LED_B, PIN_HIGH);
        rs++;
    }
}
// 调用函数
//blink_leds(6); // 闪烁6次
//blink_leds(3); // 闪烁3次

// 全局PWM设备指针
static struct rt_device_pwm *pwm_dev = RT_NULL;

/* 系统初始化时调用 */
int pwm_init(void)
{
    /* 查找PWM设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (!pwm_dev) {
        rt_kprintf("PWM device %s not found!\n", PWM_DEV_NAME);
        return -RT_ERROR;
    }

    /* 使能PWM通道 */
    rt_pwm_enable(pwm_dev, PWM_CHANNEL);
    return RT_EOK;
}
/* 导出到  自动初始化（可选） */
INIT_APP_EXPORT(pwm_init);

/* 封装舵机角度控制函数 */
static void set_servo_angle(int angle)
{
    rt_uint32_t pulse;
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    pulse = PWM_PULSE_0 + (angle * (PWM_PULSE_180 - PWM_PULSE_0)) / 180;

    /* 设置PWM参数 */
    if (pwm_dev != RT_NULL)
    {
        rt_pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD, pulse);
      //  rt_kprintf("Set servo to %d° (pulse: %d ns)\n", angle, pulse);
    }
}


static void beep_ctrl_thread_entry(void *parameter)
{
    rt_pin_mode(PIN_BEEP1, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_BEEP1, PIN_HIGH);
    int current_on = 0;
    int last_angle = -1;

    while (1) {
        rt_mutex_take(on_mutex, RT_WAITING_FOREVER);
        current_on = on;
        rt_mutex_release(on_mutex);
        int target_angle = current_on ? 0 : 90; // 确定目标角度
        if (target_angle != last_angle)
        {
            if (current_on)
            {
                blink_leds(3);
                rt_pin_write(PIN_BEEP1, PIN_LOW);
                set_servo_angle(target_angle);
            }
            else
            {
                rt_pin_write(PIN_BEEP1, PIN_HIGH);
                //set_servo_angle(target_angle);
            }
            rt_thread_mdelay(4000);
            set_servo_angle(target_angle);
            last_angle = target_angle;
        }

        rt_thread_mdelay(500);
    }
}



/* ADC设备及通道定义 */
#define ADC_DEV_NAME       "adc1"       // ADC设备名称
#define FLAME_ADC_CHANNEL  6            // 火焰传感器通道（PA6）
#define WATER_ADC_CHANNEL  5            // 水位传感器通道（PA5）
#define SMOKE_ADC_CHANNEL  0            // 烟雾传感器通道（PA0）
#define REF_VOLTAGE        3300         // 参考电压3.3V（单位mV）
#define ADC_RESOLUTION     (1 << 12)    // 12位ADC分辨率

static rt_adc_device_t adc_dev = RT_NULL;

/* 初始化ADC设备 */
static int adc_init(void)
{
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL) {
        rt_kprintf("ADC设备未找到！\n");
        return -RT_ERROR;
    }
    /* 使能所有通道 */
    rt_adc_enable(adc_dev, FLAME_ADC_CHANNEL);
    rt_adc_enable(adc_dev, WATER_ADC_CHANNEL);
    rt_adc_enable(adc_dev, SMOKE_ADC_CHANNEL);
    return RT_EOK;
}

/* 传感器读取函数 */
static rt_uint32_t read_flame_sensor(void)
{
    rt_uint32_t raw_value = rt_adc_read(adc_dev, FLAME_ADC_CHANNEL);
    return (raw_value * REF_VOLTAGE) / ADC_RESOLUTION;
}

static rt_uint32_t read_water_sensor(void)
{
    rt_uint32_t raw_value = rt_adc_read(adc_dev, WATER_ADC_CHANNEL);
    return (raw_value * REF_VOLTAGE) / ADC_RESOLUTION;
}

static rt_uint32_t read_smoke_sensor(void)
{
    rt_uint32_t raw_value = rt_adc_read(adc_dev, SMOKE_ADC_CHANNEL);
    return (raw_value * REF_VOLTAGE) / ADC_RESOLUTION;
}


static void sensor_detect_thread_entry(void *parameter)
{
    rt_pin_mode(PIN_BEEP1, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_BEEP2, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_BEEP3, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_BEEP4, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_BEEP5, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_BEEP6, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_BEEP8, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_BEEP1, PIN_HIGH);
    rt_pin_write(PIN_BEEP2, PIN_HIGH);
    rt_pin_write(PIN_BEEP3, PIN_HIGH);
    rt_pin_write(PIN_BEEP4, PIN_HIGH);
    rt_pin_write(PIN_BEEP5, PIN_HIGH);
    rt_pin_write(PIN_BEEP6, PIN_HIGH);
    rt_pin_write(PIN_BEEP8, PIN_LOW);

    adc_init();
        while (1) {
    rt_uint32_t flame_voltage = read_flame_sensor();
    rt_kprintf("火焰: %d mV\t", flame_voltage);
        if (flame_voltage > 3240) {

        rt_pin_write(PIN_BEEP8, PIN_HIGH);
       // blink_leds(3);
        rt_pin_write(PIN_BEEP3, PIN_LOW);
        }
         else{
            // rt_pin_write(PIN_BEEP8, PIN_LOW);
          rt_pin_write(PIN_BEEP3, PIN_HIGH);
         }
    rt_thread_mdelay(3000);
    rt_pin_write(PIN_BEEP8, PIN_LOW);
            rt_uint32_t water_voltage = read_water_sensor();
            rt_kprintf("水位电压: %d mV\n", water_voltage);

           if (water_voltage > 1000) {
               //rt_pin_write(PIN_BEEP3, PIN_LOW);
               rt_pin_write(PIN_BEEP4, PIN_LOW);
               rt_pin_write(PIN_BEEP8, PIN_HIGH);
                rt_kprintf("[水位过高警告] ");
              }
           else{
               rt_pin_write(PIN_BEEP8, PIN_LOW);
                rt_pin_write(PIN_BEEP4, PIN_HIGH);
           }
            rt_uint32_t smoke_voltag = read_smoke_sensor();
            rt_kprintf("烟雾: %d mV\n", smoke_voltag);

            if (smoke_voltag > 10500) {
                //blink_leds(3);
                rt_pin_write(PIN_BEEP6, PIN_LOW);
            }
            else{
                rt_pin_write(PIN_BEEP6, PIN_HIGH);
            }
            if (rt_mutex_take(sensor_mutex, RT_WAITING_FOREVER) == RT_EOK) {
                        flame_voltage_global = flame_voltage;
                        water_voltage_global = water_voltage;
                        smoke_voltage_global = smoke_voltag;
                        rt_mutex_release(sensor_mutex);
                    }
            rt_thread_mdelay(1000); // 统一检测周期500ms
        }
}



void read_sensor_data(rt_uint32_t *flame, rt_uint32_t *water, rt_uint32_t *smoke) {
    if (rt_mutex_take(sensor_mutex, RT_WAITING_FOREVER) == RT_EOK) {
        *flame = flame_voltage_global;
        *water = water_voltage_global;
        *smoke = smoke_voltage_global;
        rt_mutex_release(sensor_mutex);
    }
}




/* 命令行测试函数 */
static int flame_test(int argc, char *argv[]) {
    rt_kprintf("火焰电压: %d mV\n", read_flame_sensor());
    return RT_EOK;
}
MSH_CMD_EXPORT(flame_test, 火焰传感器测试);

static int water_test(int argc, char *argv[]) {
    rt_kprintf("水位电压: %d mV\n", read_water_sensor());
    return RT_EOK;
}
MSH_CMD_EXPORT(water_test, 水位传感器测试);

static int smoke_test(int argc, char *argv[]) {
    rt_kprintf("烟雾电压: %d mV\n", read_smoke_sensor());
    return RT_EOK;
}
MSH_CMD_EXPORT(smoke_test, 烟雾传感器测试);



int lianjie(void)
{
    static int i = 0;
    int result = RT_EOK;
    struct rt_wlan_info info;


    rt_thread_mdelay(500);
    /* 扫描热点 */
    LOG_D("start to scan ap ...");
    /* 执行扫描 */
    rt_sem_init(&scan_done,"scan_done",0,RT_IPC_FLAG_FIFO);
    rt_wlan_register_event_handler(RT_WLAN_EVT_SCAN_REPORT, wlan_scan_report_hander,&i);
    rt_wlan_register_event_handler(RT_WLAN_EVT_SCAN_DONE, wlan_scan_done_hander,RT_NULL);

    if(rt_wlan_scan() == RT_EOK)
    {
        LOG_D("the scan is started... ");
    }else
    {
        LOG_E("scan failed");
    }
    rt_sem_take(&scan_done,RT_WAITING_FOREVER);
    LOG_D("start to connect ap ...");
    rt_sem_init(&net_ready, "net_ready", 0, RT_IPC_FLAG_FIFO);
    rt_wlan_register_event_handler(RT_WLAN_EVT_READY, wlan_ready_handler, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_DISCONNECTED, wlan_station_disconnect_handler, RT_NULL);
    result = rt_wlan_connect(WLAN_SSID, WLAN_PASSWORD);
    if (result == RT_EOK)
    {
        rt_memset(&info, 0, sizeof(struct rt_wlan_info));
        /* 获取当前连接热点信息 */
        rt_wlan_get_info(&info);
        LOG_D("station information:");
        print_wlan_information(&info,0);
        /* 等待成功获取 IP */
        result = rt_sem_take(&net_ready, NET_READY_TIME_OUT);
        if (result == RT_EOK)
        {
            LOG_D("networking ready!");
            msh_exec("ifconfig", rt_strlen("ifconfig"));
        }
        else
        {
            LOG_D("wait ip got timeout!");
        }
        /* 回收资源 */
        rt_wlan_unregister_event_handler(RT_WLAN_EVT_READY);
        rt_sem_detach(&net_ready);
    }
    else
    {
        LOG_E("The AP(%s) is connect failed!", WLAN_SSID);
    }

    rt_thread_mdelay(5000);
    LOG_D("ready to disconect from ap ...");
    rt_wlan_disconnect();

    /* 自动连接 */
    LOG_D("start to autoconnect ...");
    wifi_autoconnect();

    rt_thread_mdelay(3000);



    int ret;
    int sock, bytes_received;
    recv_data = rt_malloc(BUFSZ);
    if (recv_data == RT_NULL)
    {
        rt_kprintf("No memory\n");
    }

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        rt_kprintf("Socket error\n");
        rt_free(recv_data);
    }
    rt_kprintf("Socket rignt\n");

    struct hostent *host;
    host = (struct hostent *) gethostbyname("192.168.21.193");
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(7788);
    server_addr.sin_addr = *((struct in_addr *)host->h_addr);
    rt_memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
        rt_kprintf("Connect fail!\n");
        closesocket(sock);
        rt_free(recv_data);
    }
    else
    {
        rt_kprintf("Connect successful\n");
    }



#define TEMP_THRESHOLD 60.0
    rt_pin_mode(PIN_BEEP7, PIN_MODE_OUTPUT);
        rt_pin_write(PIN_BEEP7, PIN_HIGH);

    while (1)
    {
        bytes_received = recv(sock, recv_data, BUFSZ - 1, 0);
        if (bytes_received < 0)
        {
            closesocket(sock);
            rt_kprintf("\nreceived error,close the socket.\r\n");
            rt_free(recv_data);
            break;
        }
        else if (bytes_received == 0)
        {
            closesocket(sock);
            rt_kprintf("\nreceived error,close the socket.\r\n");
            rt_free(recv_data);
            break;
        }

        recv_data[bytes_received] = '\0';
        if (strncmp(recv_data, "q", 1) == 0 || strncmp(recv_data, "Q", 1) == 0)
        {
            closesocket(sock);
            rt_kprintf("\n got a 'q' or 'Q',close the socket.\r\n");
            rt_free(recv_data);
           break;
        }


        if (strncmp(recv_data, "TEMP:", 5) == 0)
            {
                float temperature = 0.0;
                if (sscanf(recv_data + 5, "%f", &temperature) == 1)
                {
                    //rt_kprintf("Current temperature: %.2f°C\n", temperature);

                    // 温度过高报警
                    if (temperature > TEMP_THRESHOLD)
                    {
                        rt_pin_write(PIN_BEEP7, PIN_HIGH);
                        rt_kprintf("Temperature exceeds threshold! Alarm activated!\n");
                    }
                    else
                    {
                        rt_pin_write(PIN_BEEP7, PIN_LOW);
                        rt_kprintf("Alarm activated!\n");
                    }
                }
                else
                {
                    rt_kprintf("Invalid temperature format\n");
                }
            }
            // 处理控制指令
            else if (strncmp(recv_data, "1", 1) == 0)
            {
                rt_kprintf("Control command: 1 (ON)\n");
                rt_mutex_take(on_mutex, RT_WAITING_FOREVER);
                on = 1;
                rt_mutex_release(on_mutex);
            }
            else if (strncmp(recv_data, "0", 1) == 0)
            {
                rt_kprintf("Control command: 0 (OFF)\n");
                rt_mutex_take(on_mutex, RT_WAITING_FOREVER);
                on = 0;
                rt_mutex_release(on_mutex);
            }
            else
            {

                int value = atoi(recv_data);
                if (recv_data >= 60)
                {
                    rt_pin_write(PIN_BEEP7, PIN_LOW);
                    rt_kprintf("High value command: %s (Beep ON)\n", recv_data);
                }
                else
                {
                    rt_pin_write(PIN_BEEP7, PIN_HIGH);
                    rt_kprintf("Unknown command: %s\n", recv_data);
                    rt_kprintf("Unknown command: %d\n", value);
                }
            }




           // else{
               // rt_pin_write(PIN_BEEP7, PIN_HIGH);
               // rt_kprintf("Unknown command: %s\n", recv_data);
                //rt_pin_write(PIN_BEEP7, PIN_LOW);
          //  }







                /*if(recv_data>40)
            {
                rt_kprintf("Unknown command: %s\n", recv_data);
                blink_leds(3);
                rt_pin_write(PIN_BEEP7, PIN_LOW);
            }
            else{
                rt_pin_write(PIN_BEEP7, PIN_HIGH);
            }*/



                 /*rt_uint32_t flame, water, smoke;
                char send_buffer[BUFSZ];
                read_sensor_data(&flame, &water, &smoke);

                int len = rt_snprintf(send_buffer, BUFSZ,
                    "Flame: %d mV, Water: %d mV, Smoke: %d mV",
                    flame, water, smoke);

                if (len > 0 && len < BUFSZ) {
                    send(sock, send_buffer, len, 0);
                }*/

        rt_uint32_t flame, water, smoke;
        //int distance, distance_cm, distance_mm;
        rt_uint32_t distance, distance_cm, distance_mm;
        char send_buffer[BUFSZ];
        read_sensor_data(&flame, &water, &smoke);

        // 获取SR04距离数据
        rt_mutex_take(sr04_mutex, RT_WAITING_FOREVER);
        distance = sr04_distance;
        rt_mutex_release(sr04_mutex);

        distance_cm = distance / 10;  // 厘米部分
        distance_mm = distance % 10;  // 毫米部分

        // 格式化数据
        int len = rt_snprintf(send_buffer, BUFSZ,
            "Flame: %d mV, Water: %d mV, Smoke: %d mV, Distance: %d.%d cm",
            flame, water, smoke, distance_cm, distance_mm);

        // 发送数据
        if (len > 0 && len < BUFSZ) {
            send(sock, send_buffer, len, 0);
            rt_kprintf("Control\n");
        }
                rt_thread_mdelay(1000);
    }
    return 0;
}

static void print_wlan_information(struct rt_wlan_info *info,int index)
{
        char *security;

        if(index == 0)
        {
            rt_kprintf("             SSID                      MAC            security    rssi chn Mbps\n");
            rt_kprintf("------------------------------- -----------------  -------------- ---- --- ----\n");
        }

        {
            rt_kprintf("%-32.32s", &(info->ssid.val[0]));
            rt_kprintf("%02x:%02x:%02x:%02x:%02x:%02x  ",
                    info->bssid[0],
                    info->bssid[1],
                    info->bssid[2],
                    info->bssid[3],
                    info->bssid[4],
                    info->bssid[5]
                    );
            switch (info->security)
            {
            case SECURITY_OPEN:
                security = "OPEN";
                break;
            case SECURITY_WEP_PSK:
                security = "WEP_PSK";
                break;
            case SECURITY_WEP_SHARED:
                security = "WEP_SHARED";
                break;
            case SECURITY_WPA_TKIP_PSK:
                security = "WPA_TKIP_PSK";
                break;
            case SECURITY_WPA_AES_PSK:
                security = "WPA_AES_PSK";
                break;
            case SECURITY_WPA2_AES_PSK:
                security = "WPA2_AES_PSK";
                break;
            case SECURITY_WPA2_TKIP_PSK:
                security = "WPA2_TKIP_PSK";
                break;
            case SECURITY_WPA2_MIXED_PSK:
                security = "WPA2_MIXED_PSK";
                break;
            case SECURITY_WPS_OPEN:
                security = "WPS_OPEN";
                break;
            case SECURITY_WPS_SECURE:
                security = "WPS_SECURE";
                break;
            default:
                security = "UNKNOWN";
                break;
            }
            rt_kprintf("%-14.14s ", security);
            rt_kprintf("%-4d ", info->rssi);
            rt_kprintf("%3d ", info->channel);
            rt_kprintf("%4d\n", info->datarate / 1000000);
        }
}


static int wifi_autoconnect(void)
{
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    rt_wlan_config_autoreconnect(RT_TRUE);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED, wlan_connect_handler, RT_NULL);
    rt_wlan_register_event_handler(RT_WLAN_EVT_STA_CONNECTED_FAIL, wlan_connect_fail_handler, RT_NULL);
    return 0;
}

/*
void ir_sensor_thread_entry(void *parameter) {
    rt_pin_mode(PIN_BEEP2, PIN_MODE_OUTPUT);
     rt_pin_write(PIN_BEEP2, PIN_HIGH);
    while (1) {
        if (rt_pin_read(IR_SENSOR_PIN) == PIN_LOW) {
            rt_pin_write(PIN_BEEP2, PIN_LOW);
            set_servo_angle(0);
            rt_kprintf("Obstacle detected!\n");
        } else {
            //set_servo_angle(90);
            rt_pin_write(PIN_BEEP2, PIN_HIGH);
            //rt_kprintf("detected!\n");
        }
rt_thread_mdelay(250); // 50ms延时消抖
    }rt_thread_mdelay(2500);
        set_servo_angle(90);
}*/
void ir_sensor_thread_entry(void *parameter) {
    rt_pin_mode(PIN_BEEP2, PIN_MODE_OUTPUT);
     rt_pin_write(PIN_BEEP2, PIN_HIGH);
     int need_reset_servo = 0;
     rt_tick_t reset_time;

     while (1) {
         // 传感器检测逻辑
         if (rt_pin_read(IR_SENSOR_PIN) == PIN_LOW) {

             rt_pin_write(PIN_BEEP2, PIN_LOW);
             set_servo_angle(0);
             rt_kprintf("Obstacle detected!\n");

             // 设置重置标志和时间戳
             need_reset_servo = 1;
             reset_time = rt_tick_get() + rt_tick_from_millisecond(2500);
         } else {
             rt_pin_write(PIN_BEEP2, PIN_HIGH);
         }

         // 检查是否需要重置舵机
         if (need_reset_servo && (rt_tick_get() >= reset_time)) {
             set_servo_angle(90);
             need_reset_servo = 0;
         }

         rt_thread_mdelay(150); // 保持消抖延时
     }
}



static void sr04_read_distance_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    rt_size_t res;
    dev = rt_device_find(parameter);
    if (dev == RT_NULL) {
        rt_kprintf("Can't find device:%s\n", parameter);
        return;
    }
    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK) {
        rt_kprintf("open device failed!\n");
        return;
    }
    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)100);
    while (1) {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1) {
            rt_kprintf("read data failed!size is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else {
            rt_kprintf("distance:%3d.%dcm, timestamp:%5d\n", sensor_data.data.proximity / 10, sensor_data.data.proximity % 10, sensor_data.timestamp);
            if(sensor_data.data.proximity < 250)
            {
                rt_kprintf("有车\n");
            }
            else {
                rt_kprintf("无车\n");
            }
            rt_mutex_take(sr04_mutex, RT_WAITING_FOREVER);
            sr04_distance = sensor_data.data.proximity;
            rt_mutex_release(sr04_mutex);
        }
        rt_thread_mdelay(1000);
    }
}




int rt_hw_sr04_port(void)
{
    struct rt_sensor_config cfg;
    rt_base_t pins[2] = {SR04_TRIG_PIN, SR04_ECHO_PIN};
    cfg.intf.dev_name = "timer14";
    cfg.intf.user_data = (void *)pins;
    rt_hw_sr04_init("sr04", &cfg);
    return RT_EOK;
}

INIT_COMPONENT_EXPORT(rt_hw_sr04_port);


static rt_thread_t core_thread1 = RT_NULL;      /* 线程句柄 */
static rt_thread_t core_thread2 = RT_NULL;      /* 线程句柄 */


int main(void) {
    sensor_mutex = rt_mutex_create("sensor_mtx", RT_IPC_FLAG_FIFO);
        if (sensor_mutex == RT_NULL) {
            rt_kprintf("Failed to create sensor mutex!\n");
            return -1;
        }
        on_mutex = rt_mutex_create("on_mutex", RT_IPC_FLAG_FIFO);
        if (on_mutex == RT_NULL) {
            rt_kprintf("Failed to create mutex\n");
        }
        sr04_mutex = rt_mutex_create("sr04_mutex", RT_IPC_FLAG_FIFO);
        if (sr04_mutex == RT_NULL) {
            rt_kprintf("Failed to create sr04_mutex\n");
        }
    rt_thread_t core_thread1, core_thread2, sensor_thread,sr04_thread;
    //core_thread1 = rt_thread_create("lianjie", lianjie, RT_NULL, 4096, 10, 5);
    core_thread1 = rt_thread_create("lianjie", lianjie, RT_NULL, 9192, 10, 5);
    if (core_thread1 != RT_NULL) {
        rt_thread_startup(core_thread1);
    }
    core_thread2 = rt_thread_create("beep_ctrl_thread_entry",beep_ctrl_thread_entry,RT_NULL,2048,6, 4);
    if (core_thread2 != RT_NULL) {
        rt_thread_startup(core_thread2);
    }
    sensor_thread = rt_thread_create("sensor_detect",sensor_detect_thread_entry,RT_NULL,4096, 20, 10);
    if (sensor_thread != RT_NULL) {
        rt_thread_startup(sensor_thread);
    }
    ir_sensor_init();
        rt_thread_t tid = rt_thread_create("ir_sensor", ir_sensor_thread_entry, RT_NULL, 512, 10, 10);
        if (tid != RT_NULL) rt_thread_startup(tid);
            sr04_thread = rt_thread_create("sr04",sr04_read_distance_entry,"pr_sr04", 2048,RT_THREAD_PRIORITY_MAX / 2, 20);
            if (sr04_thread != RT_NULL) {
                rt_thread_startup(sr04_thread);
            }
    // 主线程保持活跃
    while (1) {
        rt_thread_mdelay(100);
    }
    return RT_EOK;
}



