#include <mosquitto.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    printf("Received message: %s\n", (char *) message->payload);
}

int main(int argc, char *argv[]) {
    struct mosquitto *mosq;
    int rc;

    mosquitto_lib_init();

    mosq = mosquitto_new("subscriber-test", true, NULL);
    if(mosq == NULL){
        fprintf(stderr, "Error: Out of memory.\n");
        return 1;
    }

    mosquitto_message_callback_set(mosq, on_message);

    rc = mosquitto_connect(mosq, "localhost", 1883, 60);
    if(rc != 0){
        fprintf(stderr, "Unable to connect (%s).\n", mosquitto_strerror(rc));
        return 1;
    }

    mosquitto_subscribe(mosq, NULL, "test/topic", 0);

    mosquitto_loop_forever(mosq, -1, 1);

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}