shell 需要进行一定的修改，shell部分在实机飞行时还需调用相机，该版本未加
模型检查中onFrame，还存在一定逻辑未补充，无人机投放函数暂未添加，如需添加请加至flight_control


该版本新增了 <onnxruntime_cxx_api.h> 头文件，还需在虚拟机进行配置


如果要在rviz里看摄像头，运行这个，找image里的uncompressed

# 核心命令：把 /camera/image_raw/compressed 解压为 /camera/image_raw/uncompressed
rosrun image_transport republish compressed in:=/camera/image_raw raw out:=/camera/image_raw/uncompressed