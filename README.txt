手动运行websock_server的步骤：
1. 关闭websock_server的自启动
sudo systemctl disable websock_server
2. 关闭websock_server服务
sudo systemctl stop websock_server
3. 在当前目录下将build文件夹下的可执行文件复制过来：
cp -p build/websocket_server websocket_server
4. 在当前目录下执行，注意不要在build下执行：
./websock_server



如果需要在当前目录下进行编译，可以在当前目录下执行：
make -C build