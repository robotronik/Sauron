saurondir=`pwd`
echo $saurondir
cat $saurondir/sauron.service | sed "s|saurondir|${saurondir}|g" | sudo tee /etc/systemd/system/sauron.service
sudo chmod 640 /etc/systemd/system/sauron.service
sudo systemctl daemon-reload
sudo systemctl enable sauron
sudo systemctl start sauron
sudo systemctl status sauron.service