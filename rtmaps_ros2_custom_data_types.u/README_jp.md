## RTMAPS ROS2カスタムデータタイプ

ROS2 カスタムメッセージタイプを作成し、RTMAPSコンポーネントの中で使用する方法の例。
ビルド方法:

    rtmaps_ros2_custom_data_types:

    # my_data_type.msgからコードを生成する
    cd rtmaps_ros2_custom_data_types/custom_msg/
    colcon build --packages-select my_data_type

    . install/setup.bash

    cd .. # back in rtmaps_ros2_custom_data_types
    cmake . && make


さらなるROS2の情報: https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/


