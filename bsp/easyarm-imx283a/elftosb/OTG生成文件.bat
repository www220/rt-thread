@cd /d %~dp0

del .\imx28_ivt_uboot.sb
elftosb -f imx28 -c ./uboot_ivt.bd -o ./imx28_ivt_uboot.sb

OTG����uboot��DDR.bat