本"rtmaps-ros2-bridge" レポジトリには3つのプロジェクトがあります:

- rtmaps_ros2_bridge_core_function.u: RTMapsプロセスの中でROS2ノードに対してpublisherおよびsubscriberコンポーネントとして動作する2singletonです。 関連する .pck ファイルは以下の .pck ファイルのうちひとつ以上を読み込むときに自動的に読み込まれます。

- rtmaps_ros2_bridge.u: RTMAPS環境からトピックのpublishおよびsubscribeを実現するRTMaps ROS2 Bridge 自身

- rtmaps_ros2_custom_data_types.u : ROS2のカスタムメッセージタイプを生成しRTMAPSコンポーネントの中で使用する方法の例 (rtmaps_ros2_custom_data_types.u/README.md 参照)


要件:
- 正しい名前でdebファイルをビルドするための python3-rospkg deb パッケージ ("rosversion -d"コマンドによって)
