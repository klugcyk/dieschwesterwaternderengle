# 献给我的心上人等待天使的妹妹

# opencv required

# eigen3 required

# pcl required

# vortex required

git clone https://github.com/klugcyk/vortex.git

# source 中的设置

* 读取图片的路径
read_img_path "/home/klug/img/construct/"

* 读取标定图片的路径
read_img_path_cal "/home/klug/img/construct/cal/"

* 写入图片的路径
write_img_path "/home/klug/img/construct/"

* 写入畸变矫正图像路径
write_img_path_undistort "/home/klug/img/construct/undistort/"

* 双目结构光读取图片路径
zwei_read_img_path "/home/klug/img/zwei_construct/"

* 双目结构光写入图片路径
zwei_write_img_path "/home/klug/img/zwei_construct/"

* 保存文件路径
save_file_path "/home/klug/"

* 无畸变图像路径
undistort_img_path "/home/klug/img/construct/undistort/"

* 读取json参数路径
read_json_path "/home/klug/"

* 写入json参数文件路径
write_json_path "/home/klug/"

* 双目加载标定图片的数量
zwei_cal_img_num 30

* 单目加载标定图片的数量
cal_img_num 30 

* 结构光硬件上光线的条数
laserLineCnt 2 

* 相机数量
cameraCnt 1 

* 使用红色激光，与useBlueLaser只能选择一个
useRedLaser

* 使用蓝色激光，与useRedLaser只能选择一个
useBlueLaser

