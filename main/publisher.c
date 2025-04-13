/**
 * @file esp32_image_publisher.c
 * @brief micro-ROS image publisher for ESP32-CAM using FreeRTOS.
 *
 * This file implements a ROS 2 image publisher that captures JPEG images from
 * an ESP32-CAM module and publishes them over a micro-ROS agent. It includes
 * support for parameter management to control publishing rate and enable/disable publishing.
 */

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

/// Macro to check return value and abort on failure
#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}

/// Macro to check return value and log error, but continue
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

/// ROS 2 node, namespace, topic names
#define C_PUBLISHER_NAME "esp32_img_pub"
#define C_PUBLISHER_NAMESPACE ""
#define C_PUBLISHER_TOPIC_NAME "img"

//! @brief Maximum number of retries during agent discovery.
static const uint32_t C_RETRY_DISCOVERY_NUMBER = 3u;
//! @brief Time delay between discovery retries (in ms).
static const uint32_t C_RETRY_DISCOVERY_TIME = 10000u;
//! @brief Wait time (in minutes) after all retries fail before trying again.
static const uint32_t C_RETRY_DISCOVERY_FAILED_TIME = 10u;
//! @brief Timer interval for camera picture acquisition (in ms).
static const uint32_t C_TIMER_PICTURE_AQUISITION = 10u;

/// Tag used for ESP_LOG output
static const char *LOG_TAG = "picture:publish";

/// Name of the configurable integer parameter
static const char *parameter_name = "parameter_int";

/**
 * @brief Structure containing all state and ROS 2 objects for the image publisher.
 */
typedef struct
{
	rcl_publisher_t publisher;		 ///< ROS 2 image publisher
	sensor_msgs__msg__Image img_msg; ///< Image message to publish
	bool publish;					 ///< Flag to control publishing
	rclc_support_t support;			 ///< ROS 2 support object
	rcl_node_t node;				 ///< ROS 2 node
	rcl_timer_t timer_picture;		 ///< Timer to trigger image publishing
	rclc_parameter_server_t params;	 ///< ROS 2 parameter server
} TPublisher;

static TPublisher publisher;

/**
 * @brief Initializes the image message fields with default values.
 *
 * @param p_img_msg Pointer to the image message to initialize.
 */
static void publisher_init_img_msg(sensor_msgs__msg__Image *p_img_msg)
{
	memset(p_img_msg, sizeof(*p_img_msg), 0);
	p_img_msg->is_bigendian = false;
	rosidl_runtime_c__String__init(&p_img_msg->header.frame_id);
	rosidl_runtime_c__String__assign(&p_img_msg->header.frame_id, "image_frame");
	rosidl_runtime_c__String__init(&p_img_msg->encoding);
	rosidl_runtime_c__String__assign(&p_img_msg->encoding, "jpeg");
}

/**
 * @brief Timer callback that captures a frame and publishes it.
 *
 * @param timer ROS 2 timer handle.
 * @param last_call_time Timestamp of last callback execution (unused).
 */
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
			}

			esp_camera_fb_return(fb);
		}
	}
}

/**
 * @brief Attempt to discover a micro-ROS agent over UDP.
 *
 * @param init_options Initialized rcl_init_options_t object.
 * @return rcl_ret_t Return code from discovery.
 */
static rcl_ret_t publisher_discover_agent(rcl_init_options_t *init_options)
{
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(init_options);
	rcl_ret_t retval;

	bool discovery_retry = true;
	while (discovery_retry)
	{
		for (uint32_t i = 1; i <= C_RETRY_DISCOVERY_NUMBER; i++)
		{
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

/**
 * @brief Adds and sets the default parameter on the parameter server.
 *
 * @param param_server Pointer to parameter server object.
 * @return rcl_ret_t Return code.
 */
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
		retval = rclc_parameter_set_int(param_server, parameter_name, param_value);
		if (RCL_RET_OK != retval)
		{
			ESP_LOGE(LOG_TAG, "on set param");
		}
	}
	return retval;
}

/**
 * @brief Callback triggered when a parameter is added, changed, or removed.
 *
 * @param old_param Pointer to previous parameter state.
 * @param new_param Pointer to new parameter value.
 * @param context Unused user data.
 * @return true if the change is accepted, false otherwise.
 */
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

/**
 * @brief Entry point for the micro-ROS task.
 *
 * Initializes all micro-ROS entities, discovers the agent, and enters
 * the executor loop to publish camera frames and handle parameters.
 *
 * @param arg Unused task argument.
 */
void micro_ros_task(void *arg)
{
	rcl_ret_t retval;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

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

	RCCHECK(rcl_publisher_fini(&publisher.publisher, &publisher.node));
	RCCHECK(rclc_parameter_server_fini(&publisher.params, &publisher.node));
	RCCHECK(rcl_node_fini(&publisher.node));
	vTaskDelete(NULL);
}

/**
 * @brief Starts the image publisher task.
 *
 * Pins the micro-ROS task to APP_CPU to let PRO_CPU handle WiFi operations.
 */
void start_publish(void)
{
	xTaskCreate(micro_ros_task, "uros_task", 24000, NULL, 5, NULL);
}
