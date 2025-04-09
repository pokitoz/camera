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

static const char *LOG_TAG = "picture:publish";
rcl_publisher_t publisher;
sensor_msgs__msg__Image img_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		camera_fb_t *fb = esp_camera_fb_get();

		if (fb == NULL)
		{
			ESP_LOGE(LOG_TAG, "on camera get");
		}
		else
		{
			img_msg.height = 1;
			img_msg.width = fb->len;
			img_msg.step = img_msg.width;
			img_msg.data.data = fb->buf;
			img_msg.data.size = fb->len;
			img_msg.data.capacity = img_msg.data.size;
			img_msg.header.stamp.sec = fb->timestamp.tv_sec;
			img_msg.header.stamp.nanosec = fb->timestamp.tv_usec * 1000;

			ESP_LOGI(LOG_TAG, "Publishing: %ux%u %u", fb->height, fb->width, fb->len);
			RCSOFTCHECK(rcl_publish(&publisher, &img_msg, NULL));

			esp_camera_fb_return(fb);
		}
	}
}

void micro_ros_task(void *arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_ret_t ret;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	// RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
	//		  CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

	for (int i = 0; i < 3; i++)
	{
		// Need to open port on 8888 and discovery on 7400 (default):
		//    MicroXRCEAgent udp4 -p 8888 -d
		ret = rmw_uros_discover_agent(rmw_options);
		if (ret == RCL_RET_OK)
		{
			break;
		}
		// Sleep a bit.. and retry it takes multiple tries sometimes..
		usleep(10000);
	}

	RCCHECK(ret);

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_pub", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(&publisher, &node,
										ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), "img"));

	// create timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(3000),
									 timer_callback, true));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	memset(&img_msg, sizeof(img_msg), 0);

	img_msg.is_bigendian = false;
	rosidl_runtime_c__String__init(&img_msg.header.frame_id);
	rosidl_runtime_c__String__assign(&img_msg.header.frame_id, "image_frame");
	rosidl_runtime_c__String__init(&img_msg.encoding);
	rosidl_runtime_c__String__assign(&img_msg.encoding, "jpeg");

	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}

void start_publish(void)
{
	// pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	xTaskCreate(micro_ros_task, "uros_task", 16000, NULL, 5, NULL);
}
