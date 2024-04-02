#include <mosquitto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // 为了使用 sleep()

int main(int argc, char *argv[]) {
    struct mosquitto *mosq;
    int rc;
    char msg[5]; // 数据帧大小为5字节
    int sensorValue = 0; // 假设的传感器数值

    mosquitto_lib_init();

    mosq = mosquitto_new("publisher-test", true, NULL);
    if(mosq == NULL){
        fprintf(stderr, "Error: Out of memory.\n");
        return 1;
    }

    rc = mosquitto_connect(mosq, "111.230.18.220", 1883, 60); // 更改为您的服务器IP地址
    if(rc != 0){
        fprintf(stderr, "Unable to connect (%s).\n", mosquitto_strerror(rc));
        return 1;
    }

    while (1) {
        // 温度数据帧
        msg[0] = 0x5A; msg[1] = 0x51; msg[2] = (char)sensorValue; msg[3] = 0xA5;
        mosquitto_publish(mosq, NULL, "test/topic", sizeof(msg), msg, 0, false);
        sleep(1); // 等待1秒

        // 湿度数据帧
        msg[1] = 0x52;
        mosquitto_publish(mosq, NULL, "test/topic", sizeof(msg), msg, 0, false);
        sleep(1); // 等待1秒

        // 光照强度数据帧
        msg[1] = 0x53;
        mosquitto_publish(mosq, NULL, "test/topic", sizeof(msg), msg, 0, false);
        sleep(1); // 等待1秒
    }

    // 以下代码在实际应用中可能永远不会执行，因为上面的循环是无限循环
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}
