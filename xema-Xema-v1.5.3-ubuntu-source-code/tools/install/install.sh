#!/bin/sh
echo '[Desktop Entry]' >xema.desktop
echo 'Encoding=UTF-8' >>xema.desktop
echo 'Name=XEMA' >>xema.desktop
echo 'Comment=XEMA' >>xema.desktop
echo 'Exec=sh '$PWD'/open_cam3d_gui.sh' >>xema.desktop
echo 'Icon= '$PWD'/xema_logo.png' >>xema.desktop
echo 'Terminal=false' >>xema.desktop
echo 'StartupNotify=true' >>xema.desktop
echo 'Type=Application' >>xema.desktop
echo 'Categories=Application;Development;' >>xema.desktop
sudo chmod +x xema.desktop
rm ~/.local/share/applications/xema.desktop
cp xema.desktop ~/.local/share/applications

echo '[Desktop Entry]' >xema_config.desktop
echo 'Encoding=UTF-8' >>xema_config.desktop
echo 'Name=XEMA_CONFIG' >>xema_config.desktop
echo 'Comment=XEMA_CONFIG' >>xema_config.desktop
echo 'Exec=sh '$PWD'/configuring_ip_gui.sh' >>xema_config.desktop
echo 'Icon= '$PWD'/xema_config.png' >>xema_config.desktop
echo 'Terminal=false' >>xema_config.desktop
echo 'StartupNotify=true' >>xema_config.desktop
echo 'Type=Application' >>xema_config.desktop
echo 'Categories=Application;Development;' >>xema_config.desktop
sudo chmod +x xema_config.desktop
rm ~/.local/share/applications/xema_config.desktop
cp xema_config.desktop ~/.local/share/applications
