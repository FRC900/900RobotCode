	cd
	wget https://developer.nvidia.com/embedded/dlc/l4t-sources-32-1-JAX-TX2 -O l4t-sources-32-1-JAX-TX2.tbz2
	tar -xf l4t-sources-32-1-JAX-TX2.tbz2 public_sources/kernel_src.tbz2
	mkdir jetson_kernel
	cd jetson_kernel
	tar -xf ../public_sources/kernel_src.tbz2
	cp -r hardware hardware.orig
	
	# here a miracle happens 
	# - patch dts files in hardware tree
	# - copy whole hardware tree to jetson
	# - make -j6 dtbs
	# use tar command below to create an archive of the patched dts and Image file
	# copy it over to host machine
	# extract it
	# run cp commands below to overlay files in the JetPack installer

	sudo tar -cjf jetpack_4.2_backup.tbz2 -C `pwd` `sudo find ~/nvidia -name Image` `sudo find ~/nvidia -name tegra186-quill-p3310-1000-a00-00-base.dtb` `sudo find ~/nvidia -name tegra186-quill-p3310-1000-c03-00-base.dtb` `sudo find ~/nvidia -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb` `sudo find ~/nvidia -name tegra186-quill-p3489-1000-a00-00-ucm[12].dtb`

	cp `find . -name Image` ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3310/Linux_for_Tegra/kernel/Image
	cp `find . -name Image` ~/nvidia/nvidia_sdk/JetPack_4.2_Linux_P3310/Linux_for_Tegra/kernel/Image

	# no hits for zImage in the nvidia 4.2 installer

	cp `find . -name tegra186-quill-p3310-1000-a00-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3310-1000-a00-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot

	cp `find . -name tegra186-quill-p3310-1000-c03-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	cp `find . -name tegra186-quill-p3310-1000-c03-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/bootloader
	sudo cp `find . -name tegra186-quill-p3310-1000-c03-00-base.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot
 
	cp `find . -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3310-1000-c03-00-dsi-hdmi-dp.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot

	cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm1.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm1.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot

	cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm2.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/kernel/dtb
	sudo cp `find . -name tegra186-quill-p3489-1000-a00-00-ucm2.dtb | head -n1` ~/nvidia/nvidia_sdk/JetPack_4.2.1_Linux_GA_P3310/Linux_for_Tegra/rootfs/boot
