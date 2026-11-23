DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_fr.jpg --intrinsics_calibration $DIR/camera_fr.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_fl.jpg --intrinsics_calibration $DIR/camera_fl.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_4.jpg --intrinsics_calibration $DIR/camera_rear_4.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_1.jpg --intrinsics_calibration $DIR/camera_rear_1.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_5.jpg --intrinsics_calibration $DIR/camera_rear_5.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_2.jpg --intrinsics_calibration $DIR/camera_rear_2.intrinsics.yaml
python3 $DIR/draw_rectified_image.py --jpeg_path $DIR/camera_images/camera_rear_3.jpg --intrinsics_calibration $DIR/camera_rear_3.intrinsics.yaml
