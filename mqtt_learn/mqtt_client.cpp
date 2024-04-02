#include <mosquitto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // 为了使用 sleep()

// 将浮点数转换为字节数组
void floatToBytes(float value, unsigned char* bytes) {
    union {
        float a;
        unsigned char bytes[4];
    } thing;

    thing.a = value;
    memcpy(bytes, thing.bytes, sizeof(thing.bytes));
}

int main(int argc, char *argv[]) {
    struct mosquitto *mosq;
    int rc;
    unsigned char msg[7]; // 数据帧大小为7字节
    float sensorValue = 0.0; // 使用浮点数作为传感器数值

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
        msg[0] = 0x5A; // 起始标识符
        // 温度数据帧
        msg[1] = 0x01; // 功能码为0x01
        floatToBytes(sensorValue, &msg[2]);
        msg[6] = 0xA5; // 结束标识符
        mosquitto_publish(mosq, NULL, "test/topic", sizeof(msg), msg, 0, false);
        sleep(1); // 等待1秒

        // 湿度数据帧
        msg[1] = 0x02; // 功能码变为0x02
        floatToBytes(sensorValue + 1.0, &msg[2]); // 假设的湿度值，这里只是示例
        mosquitto_publish(mosq, NULL, "test/topic", sizeof(msg), msg, 0, false);
        sleep(1); // 等待1秒

        // 光照强度数据帧
        msg[1] = 0x03; // 功能码变为0x03
        floatToBytes(sensorValue + 2.0, &msg[2]); // 假设的光照强度值，这里只是示例
        mosquitto_publish(mosq, NULL, "test/topic", sizeof(msg), msg, 0, false);
        sleep(1); // 等待1秒
    }

    // 以下代码在实际应用中可能永远不会执行，因为上面的循环是无限循环
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}
