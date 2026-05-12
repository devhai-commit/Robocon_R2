import torch
from ultralytics import YOLO

# 1. Lưu lại hàm export gốc của PyTorch
original_export = torch.onnx.export

# 2. Tạo hàm "vá lỗi" (monkey patch) để ép loại bỏ tham số dynamo
def patched_export(*args, **kwargs):
    if 'dynamo' in kwargs:
        del kwargs['dynamo']
    return original_export(*args, **kwargs)

# 3. Ghi đè hàm gốc bằng hàm đã vá
torch.onnx.export = patched_export

# 4. Tiến hành load và export bình thường
if __name__ == "__main__":
    print("Bắt đầu export sang ONNX với bản vá lỗi dynamo...")

    model = YOLO("/home/robocon/yolo26s_best.pt")
    
    # Export với kích thước ảnh cố định để tối ưu hóa tốt nhất cho phần cứng
    model.export(format="onnx", opset=12, imgsz=640)
    print("Export thành công!")


# /usr/src/tensorrt/bin/trtexec 
# --onnx=/home/robocon/yolo26s_best.onnx 
# --saveEngine=/home/robocon/ros_ws/yolo26s_best.engine 
# --fp16