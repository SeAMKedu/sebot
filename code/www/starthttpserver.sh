cd /opt/nomga/www
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
python3 -m http.server 8080
