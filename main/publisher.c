#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_camera.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/image.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rmw_microros/rmw_microros.h>

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

#define C_PUBLISHER_NAME "esp32_img_pub"
#define C_PUBLISHER_NAMESPACE ""
#define C_PUBLISHER_TOPIC_NAME "img"

// Number of retries for discovery before giving up
static const uint32_t C_RETRY_DISCOVERY_NUMBER = 3u;
// Time for discovery [ms]
static const uint32_t C_RETRY_DISCOVERY_TIME = 10000u;
// Time when discovery failed during the last attempts [min]
static const uint32_t C_RETRY_DISCOVERY_FAILED_TIME = 10u;
// Time between 2 camera image aquisition [ms]
static const uint32_t C_TIMER_PICTURE_AQUISITION = 10u;
// Tag before a log
static const char *LOG_TAG = "picture:publish";
// Parameter name
static const char *parameter_name = "parameter_int";

typedef struct
{
	rcl_publisher_t publisher;
	sensor_msgs__msg__Image img_msg;
	bool publish;
	rclc_support_t support;
	rcl_node_t node;
	rcl_timer_t timer_picture;
	rclc_parameter_server_t params;
} TPublisher;

static TPublisher publisher;

static void publisher_init_img_msg(sensor_msgs__msg__Image *p_img_msg)
{
	memset(p_img_msg, sizeof(*p_img_msg), 0);

	p_img_msg->is_bigendian = false;
	rosidl_runtime_c__String__init(&p_img_msg->header.frame_id);
	rosidl_runtime_c__String__assign(&p_img_msg->header.frame_id, "image_frame");
	rosidl_runtime_c__String__init(&p_img_msg->encoding);
	rosidl_runtime_c__String__assign(&p_img_msg->encoding, "jpeg");
}

static void timer_picture_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	rcl_ret_t retval = 0;
	if (timer != NULL)
	{
		camera_fb_t *fb = esp_camera_fb_get();

		if (fb == NULL)
		{
			ESP_LOGE(LOG_TAG, "on camera get");
		}
		else
		{
			publisher.img_msg.height = fb->height;
			publisher.img_msg.width = fb->width;
			publisher.img_msg.step = publisher.img_msg.width;
			publisher.img_msg.data.data = fb->buf;
			publisher.img_msg.data.size = fb->len;
			publisher.img_msg.data.capacity = publisher.img_msg.data.size;
			publisher.img_msg.header.stamp.sec = fb->timestamp.tv_sec;
			publisher.img_msg.header.stamp.nanosec = fb->timestamp.tv_usec * 1000;

			retval = rcl_publish(&publisher.publisher, &publisher.img_msg, NULL);
			if (retval == RCL_RET_OK)
			{
				ESP_LOGI(LOG_TAG, "Published: %ux%u %u", fb->height, fb->width, fb->len);
			}
			else
			{
				ESP_LOGE(LOG_TAG, "on publish %d", retval);
				// Count number of errors, if too many, we restart.
			}

			esp_camera_fb_return(fb);
		}
	}
}

static rcl_ret_t publisher_discover_agent(rcl_init_options_t *init_options)
{
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(init_options);
	rcl_ret_t retval;

	// Static Agent IP and port can be used instead of autodisvery.
	// RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
	//		  CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

	bool discovery_retry = true;
	while (discovery_retry)
	{
		for (uint32_t i = 1; i <= C_RETRY_DISCOVERY_NUMBER; i++)
		{
			// Need to open port on 8888 and discovery on 7400 (default):
			//    MicroXRCEAgent udp4 -p 8888 -d
			retval = rmw_uros_discover_agent(rmw_options);
			if (retval == RCL_RET_OK)
			{
				discovery_retry = false;
				break;
			}

			ESP_LOGW(LOG_TAG, "on discovery: %d - retry: %u/%u wait %u [ms]",
					 retval, i, C_RETRY_DISCOVERY_NUMBER, C_RETRY_DISCOVERY_TIME);
			// Sleep a bit.. and retry it takes multiple tries sometimes..
			rclc_sleep_ms(C_RETRY_DISCOVERY_TIME);
		}

		// Now, we cannot do much, we need to wait for the publisher again. But this could happened
		// after a while.. So let's sleep and retry.

		if (discovery_retry)
		{
			ESP_LOGW(LOG_TAG, "Discovery failed: retry after %u [min]", C_RETRY_DISCOVERY_FAILED_TIME);
			rclc_sleep_ms(C_RETRY_DISCOVERY_FAILED_TIME * (1000 * 60));
		}
	}

	return retval;
}

static rcl_ret_t publisher_setup_parameters(rclc_parameter_server_t *param_server)
{
	int64_t param_value = 100;
	rcl_ret_t retval = rclc_add_parameter(param_server, parameter_name, RCLC_PARAMETER_INT);
	if (RCL_RET_OK != retval)
	{
		ESP_LOGE(LOG_TAG, "on add param");
	}
	else
	{
		// Set parameter value
		retval = rclc_parameter_set_int(param_server, parameter_name, param_value);
		if (RCL_RET_OK != retval)
		{
			ESP_LOGE(LOG_TAG, "on set param");
		}
	}

	return retval;
}

static bool on_parameter_changed(const Parameter *old_param, const Parameter *new_param, void *context)
{
	RCLC_UNUSED(context);

	if (old_param == NULL && new_param == NULL)
	{
		ESP_LOGE(LOG_TAG, "on param cb args");
		return false;
	}

	bool retval = true;
	if (new_param == NULL)
	{
		ESP_LOGE(LOG_TAG, "on remove param %s", old_param->name.data);
		retval = false;
	}
	else if (strcmp(new_param->name.data, "publish_toogle") == 0 &&
			 new_param->value.type == RCLC_PARAMETER_BOOL)
	{
		publisher.publish = new_param->value.bool_value;
		ESP_LOGI(LOG_TAG, "update publish: %s", (publisher.publish) ? "ON" : "OFF");
	}
	else if (strcmp(new_param->name.data, "publish_rate_ms") == 0 &&
			 new_param->value.type == RCLC_PARAMETER_INT)
	{
		int64_t old;
		RCSOFTCHECK(rcl_timer_exchange_period(&publisher.timer_picture, RCL_MS_TO_NS(new_param->value.integer_value), &old));
		ESP_LOGI(LOG_TAG, "update timer %lld", new_param->value.integer_value);
	}

	return retval;
}

void micro_ros_task(void *arg)
{
	rcl_ret_t retval;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	// This will block until the agent is found!
	RCCHECK(publisher_discover_agent(&init_options));

	RCCHECK(rclc_support_init_with_options(&publisher.support, 0, NULL, &init_options, &allocator));
	RCCHECK(rclc_node_init_default(&publisher.node, C_PUBLISHER_NAME, C_PUBLISHER_NAMESPACE, &publisher.support));

	RCCHECK(rclc_publisher_init_default(&publisher.publisher, &publisher.node,
										ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
										C_PUBLISHER_TOPIC_NAME));

	RCCHECK(rclc_parameter_server_init_default(&publisher.params, &publisher.node));

	RCCHECK(rclc_timer_init_default2(&publisher.timer_picture, &publisher.support, 
									 RCL_MS_TO_NS(C_TIMER_PICTURE_AQUISITION),
									 timer_picture_callback, true));

	RCCHECK(rclc_executor_init(&executor, &publisher.support.context,
							   RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1,
							   &allocator));

	RCCHECK(rclc_executor_add_parameter_server(&executor, &publisher.params, on_parameter_changed));

	RCCHECK(rclc_executor_add_timer(&executor, &publisher.timer_picture));

	RCCHECK(publisher_setup_parameters(&publisher.params));

	publisher_init_img_msg(&publisher.img_msg);

	while (1)
	{
		retval = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		if (retval != RCL_RET_OK)
		{
			ESP_LOGW(LOG_TAG, "on spin %d", retval);
		}

		rclc_sleep_ms(1);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher.publisher, &publisher.node));
	RCCHECK(rclc_parameter_server_fini(&publisher.params, &publisher.node));
	RCCHECK(rcl_node_fini(&publisher.node));
	vTaskDelete(NULL);
}

void start_publish(void)
{
	// pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	xTaskCreate(micro_ros_task, "uros_task", 24000, NULL, 5, NULL);
}
