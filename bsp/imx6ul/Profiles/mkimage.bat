@cd /d %~dp0

del u-boot.bin
del u-boot.imx
copy ..\rtthread.bin u-boot.bin
mkimage.exe u-boot.cfg u-boot.bin u-boot.imx
..\MfgTool2.exe