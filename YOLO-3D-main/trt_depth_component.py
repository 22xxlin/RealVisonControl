# 文件名: trt_depth_component.py
import cv2
import numpy as np
import torch
import tensorrt as trt

class TrtDepthEstimator:
    def __init__(self, engine_path, model_width=672, model_height=448):
        self.model_width = model_width
        self.model_height = model_height
        
        # 1. 初始化 TensorRT Logger
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        
        print(f"🚀 Loading TRT Engine (No-PyCUDA version): {engine_path}")
        
        # 2. 加载引擎
        try:
            with open(engine_path, 'rb') as f:
                self.engine = self.runtime.deserialize_cuda_engine(f.read())
            self.context = self.engine.create_execution_context()
        except Exception as e:
            print(f"❌ Failed to load engine: {e}")
            raise e

        # 3. 使用 PyTorch 分配 GPU 显存 (避免 PyCUDA 上下文冲突)
        # 输入: 1 x 3 x H x W
        self.input_shape = (1, 3, self.model_height, self.model_width)
        self.output_shape = (1, 1, self.model_height, self.model_width)
        
        # 在 GPU 上直接开辟空间
        self.d_input = torch.zeros(self.input_shape, dtype=torch.float32, device='cuda')
        self.d_output = torch.zeros(self.output_shape, dtype=torch.float32, device='cuda')
        
        # 绑定地址列表 (TensorRT 需要显存指针)
        self.bindings = [int(self.d_input.data_ptr()), int(self.d_output.data_ptr())]
        
        # 创建 CUDA Stream (使用 PyTorch 的 stream)
        self.stream = torch.cuda.Stream()

    def infer_raw(self, image):
        """
        核心功能：输入图片，返回 float32 类型的原始深度矩阵
        """
        # 1. 预处理 (CPU -> GPU)
        # Resize
        image_resized = cv2.resize(image, (self.model_width, self.model_height), interpolation=cv2.INTER_CUBIC)
        
        # 归一化 & 转换 (使用 numpy 做这一步比较简单，也可以用 torch 做)
        image_norm = image_resized.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        image_norm = (image_norm - mean) / std
        image_chw = image_norm.transpose(2, 0, 1) # HWC -> CHW
        image_batch = np.ascontiguousarray(image_chw[None, :, :, :])

        # Copy 到之前分配好的 PyTorch GPU Tensor 中
        # copy_ from numpy array
        self.d_input.copy_(torch.from_numpy(image_batch))

        # 2. 推理
        # 确保指针是最新的 (有时 PyTorch 可能会移动内存，虽然不太常见，但重新获取指针更安全)
        self.bindings = [int(self.d_input.data_ptr()), int(self.d_output.data_ptr())]
        
        # 执行推理
        self.context.execute_async_v2(
            bindings=self.bindings, 
            stream_handle=self.stream.cuda_stream
        )
        
        # 同步流
        self.stream.synchronize()
        
        # 3. 后处理 (GPU -> CPU)
        # 直接从 PyTorch Tensor 转回 Numpy
        raw_depth = self.d_output.squeeze().cpu().numpy()
        
        return raw_depth