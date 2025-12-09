## start_arms.sh的权限
 chmod +x /home/你的用户名/start_arms.sh

 ## 创建 systemd 服务
 sudo nano /etc/systemd/system/mit-arms.service

 ## 服务的使用
### 重载 systemd 配置
sudo systemctl daemon-reload

### 启用开机自启
sudo systemctl enable mit-arms.service

### 立即启动测试
sudo systemctl start mit-arms.service

### 查看状态
sudo systemctl status mit-arms.service

### 禁用开机自启动
sudo systemctl disable mit-arms.service
