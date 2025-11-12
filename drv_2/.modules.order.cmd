cmd_/home/yu/Project/linuxDrv/drv_2/modules.order := {   echo /home/yu/Project/linuxDrv/drv_2/hello_drv.ko; :; } | awk '!x[$$0]++' - > /home/yu/Project/linuxDrv/drv_2/modules.order
