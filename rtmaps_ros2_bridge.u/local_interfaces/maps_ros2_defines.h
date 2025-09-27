/*
 * Copyright (c) 2020 Intempora
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Intempora nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEMPORA
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "ros2_msgs.h"

static const char* s_topic_types[] = {
		"std_msgs",
		"sensor_msgs",
		"geometry_msgs",
		"nav_msgs",
		"stereo_msgs",
		"trajectory_msgs",
		"visualization_msgs",
		"actionlib_msgs",
		"diagnostic_msgs",
		"can_msgs"
};

#define TOPIC_TYPE_STD 			0
#define TOPIC_TYPE_SENSOR 		1
#define TOPIC_TYPE_GEOM 		2
#define TOPIC_TYPE_NAV			3
#define TOPIC_TYPE_STEREO		4
#define TOPIC_TYPE_TRAJ			5
#define TOPIC_TYPE_VISU			6
#define TOPIC_TYPE_ACTION		7
#define TOPIC_TYPE_DIAG			8
#define TOPIC_TYPE_CAN			9

#define TOPIC_TYPE_STD_OFFSET 			0
#define TOPIC_TYPE_SENSOR_OFFSET 		40
#define TOPIC_TYPE_GEOM_OFFSET 		    80
#define TOPIC_TYPE_NAV_OFFSET			120
#define TOPIC_TYPE_STEREO_OFFSET		160
#define TOPIC_TYPE_TRAJ_OFFSET			200
#define TOPIC_TYPE_VISU_OFFSET			240
#define TOPIC_TYPE_ACTION_OFFSET		280
#define TOPIC_TYPE_DIAG_OFFSET			320
#define TOPIC_TYPE_CAN_OFFSET			360

static const char* s_std_msgs[] = {
		"Int 32",
		"Int 32 array",
		"Int 64",
		"Int 64 array",
		"Float 32",
		"Float 32 array",
		"Float 64",
		"Float 64 array",
		"Text",
		"Uint 8 array"
};

#define STD_MSG_INT32 			0 + TOPIC_TYPE_STD_OFFSET 
#define STD_MSG_INT32_ARRAY		1 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_INT64			2 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_INT64_ARRAY		3 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_FLOAT32			4 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_FLOAT32_ARRAY	5 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_FLOAT64			6 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_FLOAT64_ARRAY	7 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_TEXT			8 + TOPIC_TYPE_STD_OFFSET
#define STD_MSG_UINT8_ARRAY		9 + TOPIC_TYPE_STD_OFFSET


static const char* s_sensor_msgs[] = {
//		"CameraInfo",
//		"ChannelFloat32",
		"CompressedImage",
		"Image",
		"Imu",
		"JointState",
		"Joy",
		"JoyFeedback",
		"JoyFeedbackArray",
		"LaserScan",
		"NavSatFix",
		"NavSatStatus",
		"PointCloud",
		"PointCloud2",
		"PointField",
		"Range",
		"RegionOfInterest"
};

//#define SENSOR_MSG_CAMERA_INFO				0
//#define SENSOR_MSG_CHANNEL_FLOAT32			1
#define SENSOR_MSG_COMPRESSED_IMAGE			0  + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_IMAGE					1 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_IMU						2 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_JOINT_STATE				3 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_JOY						4 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_JOY_FEEDBACK				5 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_JOY_FEEDBACK_ARRAY		6 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_LASER_SCAN				7 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_NAV_SAT_FIX				8 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_NAV_SAT_STATUS			9 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_POINT_CLOUD				10 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_POINT_CLOUD2				11 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_POINT_FIELD				12 + TOPIC_TYPE_SENSOR_OFFSET
#define SENSOR_MSG_RANGE					13 + TOPIC_TYPE_SENSOR_OFFSET
//#define SENSOR_MSG_ROI						14

static const char* s_geometry_msgs[] = {
		"Point",
		"Point32",
		"PointStamped",
		"Polygon",
		"PolygonStamped",
		"Pose",
		"Pose2D",
		"PoseArray",
		"PoseStamped",
		"PoseWithCovariance",
		"Quaternion",
		"QuaternionStamped",
		"Transform",
		"TransformStamped",
		"Twist",
		"TwistStamped",
		"TwistWithCovariance",
		"TwistWidthCovarianceStamped",
		"Vector3",
		"Vector3Stamped",
		"Wrench",
		"WrenchStamped"
};

#define GEOM_MSG_POINT					0  + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POINT32				1 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POINT_STAMPED			2 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POLYGON				3 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POLYGON_STAMPED		4 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POSE					5 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POSE_2D				6 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POSE_ARRAY				7 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POSE_STAMPED			8 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_POSE_WITH_COV			9 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_QUATERNION				10 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_QUATERNION_STAMPED		11 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_TRANSFORM				12 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_TRANSFORM_STAMPED		13 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_TWIST					14 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_TWIST_STAMPED			15 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_TWIST_WITH_COV			16 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_TWIST_WITH_COV_STAMPED	17 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_VECTOR3				18 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_VECTOR3_STAMPED		19 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_WRENCH					20 + TOPIC_TYPE_GEOM_OFFSET
#define GEOM_MSG_WRENCH_STAMPED			21 + TOPIC_TYPE_GEOM_OFFSET

static const char* s_nav_msgs[] = {
        "GridCells",
        "MapMetaData",
        "OccupancyGrid",
        "Odometry",
        "Path"
};

#define NAV_MSG_GRIDCELLS               0 + TOPIC_TYPE_NAV_OFFSET
#define NAV_MSG_MAP_METADATA            1 + TOPIC_TYPE_NAV_OFFSET
#define NAV_MSG_OCCUPANCY_GRID          2 + TOPIC_TYPE_NAV_OFFSET
#define NAV_MSG_ODOMETRY                3 + TOPIC_TYPE_NAV_OFFSET
#define NAV_MSG_PATH                    4 + TOPIC_TYPE_NAV_OFFSET

static const char* s_visu_msgs[] = {
        "Marker",
        "MarkerArray"
};

#define VISU_MSG_MARKER        0 + TOPIC_TYPE_VISU_OFFSET
#define VISU_MSG_MARKER_ARRAY  1 + TOPIC_TYPE_VISU_OFFSET

static const char* s_can_msgs[] = {
        "Frame"
};

#define CAN_MSG_FRAME        0 + TOPIC_TYPE_CAN_OFFSET

#define MAX_ARRAY_LAYOUT_LABEL_SIZE	256
#define MAX_ARRAY_LAYOUT_DIMENSIONS	3

#define DATA_TYPE_TEXT 			0
#define DATA_TYPE_IMG			1
#define DATA_TYPE_INT32			2
#define DATA_TYPE_INT32_ARRAY	3
#define DATA_TYPE_INT64			4
#define DATA_TYPE_INT64_ARRAY	5
#define DATA_TYPE_FLOAT32		6
#define DATA_TYPE_FLOAT32_ARRAY	7
#define DATA_TYPE_FLOAT64		8
#define DATA_TYPE_FLOAT64_ARRAY	9
#define DATA_TYPE_LASER_SCAN	10
#define DATA_TYPE_TWIST			11

typedef struct ROSArrayLayoutDim
{
	char label[MAX_ARRAY_LAYOUT_LABEL_SIZE];
	int	 size;
	int  stride;
} ROSArrayLayoutDim;

typedef struct ROSArrayLayout
{
	int data_offset;
	int nb_dims;
	ROSArrayLayoutDim dim[MAX_ARRAY_LAYOUT_DIMENSIONS];
}ROSArrayLayout;
