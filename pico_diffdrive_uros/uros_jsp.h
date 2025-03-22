#ifndef UROS_JSP_H
    #define UROS_JSP_H

    #include <string.h>

    #include "uros_wrapper.h"

    #include <sensor_msgs/msg/joint_state.h>
    #include <rosidl_runtime_c/string_functions.h>
    #include <rosidl_runtime_c/primitives_sequence_functions.h>

    #define MAX_STRINGS 5
    #define MAX_LENGTH 20

    struct UrosJsp {
        
        uint8_t n_joints;
        char joint_names[MAX_STRINGS][MAX_LENGTH];

        UrosWrapperPublisher jsp_publisher;
        sensor_msgs__msg__JointState joint_state_msg;

    } joint_state_publisher;

    void jsp_callback(rcl_timer_t* timer, int64_t last_call_time) {

        // Stamp and publish joint_states message
        set_timestamp_now(
            &joint_state_publisher.joint_state_msg.header.stamp.sec,
            &joint_state_publisher.joint_state_msg.header.stamp.nanosec
        );

        rcl_ret_t ret = rcl_publish(
            &joint_state_publisher.jsp_publisher.publisher,
            &joint_state_publisher.joint_state_msg,
            NULL
        );

    }

    void update_joint_position(int joint_index, float value) {
        joint_state_publisher.joint_state_msg.position.data[joint_index] = value;
    }

    int jsp_add_joint(char* joint_name) {
        strcpy(joint_state_publisher.joint_names[(joint_state_publisher.n_joints)++], joint_name);
        return joint_state_publisher.n_joints-1; // joint index for the joint
    }

    void configure_jsp(UrosWrapperCore* uros_core, float publish_rate) {

        // tf publisher
        uros_wrapper_add_publisher(
            uros_core,
            &(joint_state_publisher.jsp_publisher),
            "joint_states",
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            jsp_callback,
            publish_rate
        );

        uint8_t n_joints = joint_state_publisher.n_joints;

        joint_state_publisher.joint_state_msg.name.data = (rosidl_runtime_c__String*) malloc(n_joints * sizeof(rosidl_runtime_c__String));
        joint_state_publisher.joint_state_msg.name.size = n_joints;
        joint_state_publisher.joint_state_msg.name.capacity = n_joints;

        // Initialize the `name` array properly
        rosidl_runtime_c__String__Sequence__init(&joint_state_publisher.joint_state_msg.name, n_joints);

        for (int i=0;i<n_joints;i++) {
            rosidl_runtime_c__String__assign(
                &joint_state_publisher.joint_state_msg.name.data[i],
                joint_state_publisher.joint_names[i]
            );
        }

        rosidl_runtime_c__double__Sequence__init(&joint_state_publisher.joint_state_msg.position, n_joints);
        joint_state_publisher.joint_state_msg.position.size = n_joints;
        joint_state_publisher.joint_state_msg.position.capacity = n_joints;

    }

#endif