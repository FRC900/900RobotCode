	# use tar command below to create an archive of the patched dts and Image file
	# This will make it easy / easier to restore them without redownloading all of the 
	# L4T jetson linux image
	sudo tar -cjf jetpack_4.2.3_backup.tbz2 -C `pwd` `sudo find ~/nvidia/nvidia_sdk -name Image` `sudo find ~/nvidia/nvidia_sdk -name tegra186-quill-p3310-1000-a00-00-base.dtb` `sudo find ~/nvidia/nvidia_sdk -name tegra186-quill-p3310-1000-c03-00-base.dtb` `sudo find ~/nvidia/nvidia_sdk -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb` `sudo find ~/nvidia/nvidia_sdk -name tegra186-quill-p3489-1000-a00-00-ucm[12].dtb`


	# Overwride the downloaded jetpack files with 
	# the updates ones built to support the J120
	# cd somewhere
	# extract ~/2019Offseason/j120_hardware_dtb_l4t32-2-3-1.tbz2
	cp `find . -name Image | head -n1 ` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/kernel
	sudo cp `find . -name Image | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot

	# no hits for zImage in the nvidia 4.2 installer

	cp `find . -name tegra186-quill-p3310-1000-a00-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3310-1000-a00-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot

	cp `find . -name tegra186-quill-p3310-1000-c03-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3310-1000-c03-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/bootloader
	sudo cp `find . -name tegra186-quill-p3310-1000-c03-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot
 
	cp `find . -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot

	cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm1.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm1.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot

	cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm2.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm2.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.3_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot
