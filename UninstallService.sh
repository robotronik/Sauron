sudo systemctl stop sauron
sudo systemctl disable sauron
sudo rm /etc/systemd/system/sauron.service
sudo systemctl daemon-reload