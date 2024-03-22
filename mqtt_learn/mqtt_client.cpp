#include <mosquitto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 

int main(int argc, char *argv[]) {
    struct mosquitto *mosq;
    int rc;

    mosquitto_lib_init();

    mosq = mosquitto_new("publisher-test", true, NULL);
    if(mosq == NULL){
        fprintf(stderr, "Error: Out of memory.\n");
        return 1;
    }

    rc = mosquitto_connect(mosq, "localhost", 1883, 60);
    if(rc != 0){
        fprintf(stderr, "Unable to connect (%s).\n", mosquitto_strerror(rc));
        return 1;
    }

    mosquitto_publish(mosq, NULL, "test/topic", strlen("Hello, World!"), "Hello, World!", 0, false);

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}