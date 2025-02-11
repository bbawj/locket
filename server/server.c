#include "mongoose.h"
#include <stdio.h>

static const char *s_listen_on = "mqtt://0.0.0.0:1883";

// A list of subscription, held in memory
struct sub {
  struct sub *next;
  struct mg_connection *c;
  struct mg_str topic;
  uint8_t qos;
};
static struct sub *s_subs = NULL;

static void ev_handler(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_MQTT_CMD) {
    struct mg_mqtt_message *mm = (struct mg_mqtt_message *)ev_data;
    MG_DEBUG(("cmd %d qos %d", mm->cmd, mm->qos));
    switch (mm->cmd) {
    case MQTT_CMD_CONNECT: {
      if (mm->dgram.len < 9) {
        mg_error(c, "Malformed MQTT frame");
      } else if (mm->dgram.buf[8] != 4) {
        mg_error(c, "Unsupported MQTT version %d", mm->dgram.buf[8]);
      } else {
        uint8_t response[] = {0, 0};
        mg_mqtt_send_header(c, MQTT_CMD_CONNACK, 0, sizeof(response));
        mg_send(c, response, sizeof(response));
      }
      break;
    }
    case MQTT_CMD_SUBSCRIBE: {
      uint8_t qos, resp[256];
      struct mg_str topic;
      int num_topics = 0;
      struct sub *sub = calloc(1, sizeof(*sub));
      sub->c = c;
      sub->topic = mg_strdup(topic);
      sub->qos = qos;
      LIST_ADD_HEAD(struct sub, &s_subs, sub);
      MG_INFO(("SUB %p [%.*s]", c->fd, (int)sub->topic.len, sub->topic.buf));
      resp[num_topics++] = qos;
      mg_mqtt_send_header(c, MQTT_CMD_SUBACK, 0, num_topics + 2);
      uint16_t id = mg_htons(mm->id);
      mg_send(c, &id, 2);
      mg_send(c, resp, num_topics);
      break;
    }
    case MQTT_CMD_PUBLISH: {
      // Client published message. Push to all subscribed channels
      printf("PUB in [%.*s]: %zu bytes\n", (int)mm->topic.len, mm->topic.buf,
               (int)mm->data.len);

      FILE *f = fopen("image.jpg", "w");
      size_t written = fwrite(mm->data.buf, 1, mm->data.len, f);
      if (written != mm->data.len) {
        MG_ERROR(("failed to save jpeg image"));
        return;
      }
      fclose(f);

      for (struct sub *sub = s_subs; sub != NULL; sub = sub->next) {
        if (mg_match(mm->topic, sub->topic, NULL)) {
          struct mg_mqtt_opts pub_opts;
          memset(&pub_opts, 0, sizeof(pub_opts));
          pub_opts.topic = mm->topic;
          pub_opts.message = mm->data;
          pub_opts.qos = 1, pub_opts.retain = false;
          mg_mqtt_pub(sub->c, &pub_opts);
        }
      }
      break;
    }
    }
  }
}

int main(void) {
  struct mg_mgr mgr;                        // Declare event manager
  mg_mgr_init(&mgr);                        // Initialise event manager
  MG_INFO(("Starting on %s", s_listen_on)); // Inform that we're starting
  mg_mqtt_listen(&mgr, s_listen_on, ev_handler, NULL); // Create MQTT listener
  while (true)
    mg_mgr_poll(&mgr, 1000); // Event loop, 1s timeout
  mg_mgr_free(&mgr);         // Cleanup
  return 0;
}
