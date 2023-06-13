## Setting up BOOM Raspberry Pi

1. Move the `startup.sh` file to `/home/pi/startup.sh`
2. Run `sudo chmod +x /home/pi/startup.sh`
3. Move the `startup.service` file to `/etc/systemd/system/startup.service`
4. Run `sudo systemctl enable startup.service`