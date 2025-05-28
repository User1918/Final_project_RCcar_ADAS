import cv2
import numpy as np
import time
import torch 
import serial
import struct
from picamera2 import Picamera2
from ultralytics import YOLO

speed_motor_requested = 0
ser = serial.Serial('/dev/ttyUSB0', 115200,timeout=1)
ULTRALYTICS_AVAILABLE = True
PYTORCH_MODEL_PATH = "/home/labthayphuc/Desktop/yolov11ncnn/Segmentation/yolov11n_seg.pt" 
MODEL_INPUT_WIDTH = 640
MODEL_INPUT_HEIGHT = 640
CONFIDENCE_THRESHOLD = 0.35 
MASK_THRESHOLD = 0.5      
MODEL_CLASS_NAMES = {0: 'lane'}
LETTERBOX_COLOR = (114, 114, 114)
DEVICE = 'cpu' 

#-------------------Steering Parameters-------------------------
STEERING_SENSITIVITY = 60 
MAX_STEERING_ANGLE = 35.0  
DEAD_ZONE_THRESHOLD = 0.05

def letter_box(img, new_shape=(640, 640), color=(114, 114, 114)):
    top_padding = 80
    bottom_padding = 80
    left_padding = 0
    right_padding = 0
    padded_img = cv2.copyMakeBorder(img, top_padding, bottom_padding, left_padding, right_padding, cv2.BORDER_CONSTANT, value=color)
    ratio_w_applied,ratio_h_applied = (1.0, 1.0)       
    return padded_img, (ratio_w_applied, ratio_h_applied), (left_padding, top_padding)

def process_pytorch_masks(results, letterbox_input_h, letterbox_input_w, original_frame_h, original_frame_w, ratio_wh, pad_wh, current_mask_threshold):
    final_combined_mask_orig_size = np.zeros((original_frame_h, original_frame_w), dtype=np.uint8)
    if not results or results[0].masks is None or len(results[0].masks.data) == 0:
        return final_combined_mask_orig_size
    masks_data = results[0].masks.data
    #r_w_ratio, r_h_ratio = ratio_wh
    pad_dw, pad_dh = pad_wh 
    for i in range(masks_data.shape[0]): 
        instance_mask_tensor = masks_data[i]  
        instance_mask_np = instance_mask_tensor.cpu().float().numpy() 
        mask_upscaled_letterbox_res = cv2.resize(instance_mask_np,(letterbox_input_w, letterbox_input_h), interpolation=cv2.INTER_LINEAR)
        binary_mask_letterbox_res = (mask_upscaled_letterbox_res > current_mask_threshold).astype(np.uint8)
        src_y1 = int(round(pad_dh))
        src_y2 = int(round(letterbox_input_h - pad_dh)) 
        src_x1 = int(round(pad_dw))
        src_x2 = int(round(letterbox_input_w - pad_dw))
        if src_y1 >= src_y2 or src_x1 >= src_x2:continue
        unpadded_binary_mask_segment = binary_mask_letterbox_res[src_y1:src_y2, src_x1:src_x2]      
        if unpadded_binary_mask_segment.shape[0] > 0 and unpadded_binary_mask_segment.shape[1] > 0:
            final_instance_mask_at_orig_res = cv2.resize(unpadded_binary_mask_segment, (original_frame_w, original_frame_h), interpolation=cv2.INTER_NEAREST)
            final_combined_mask_orig_size[final_instance_mask_at_orig_res == 1] = 255            
    return final_combined_mask_orig_size
def calculate_lane_background_ratios(inverted_binary_mask):
    global ratio_left, ratio_right
    if not isinstance(inverted_binary_mask, np.ndarray) or inverted_binary_mask.ndim != 2:
        raise ValueError("Must be Arry2D")
    _, mask_w = inverted_binary_mask.shape
    if mask_w < 2:
        return 0.0, 0.0
    mid_w = mask_w // 2
    left_half_mask = inverted_binary_mask[:, :mid_w]
    right_half_mask = inverted_binary_mask[:, mid_w:]

    black_pixels_left = np.sum(left_half_mask == 0)
    white_pixels_left = np.sum(left_half_mask == 255)
    if white_pixels_left > 0:
        ratio_left = black_pixels_left / white_pixels_left
    else:
        ratio_left = float('inf') if black_pixels_left > 0 else 0.0

    black_pixels_right = np.sum(right_half_mask == 0)
    white_pixels_right = np.sum(right_half_mask == 255)
    if white_pixels_right > 0:
        ratio_right = black_pixels_right / white_pixels_right
    else:
        ratio_right = float('inf') if black_pixels_right > 0 else 0.0
    return ratio_left, ratio_right

def signal_motor(key):
    global speed_motor_requested
    if key == ord('w'): speed_motor_requested = min(speed_motor_requested+5, 50)
    if key == ord('s'): speed_motor_requested = max(speed_motor_requested-5, 0)
    if key == ord('x'): speed_motor_requested = 0

def transform_speed(vel):
    f=vel/10
    rpm=(f*30*2.85)/(3.14*0.0475*3.6)
    return int(round(rpm))

def calculate_steering_from_ratios(ratio_l, ratio_r, sensitivity=STEERING_SENSITIVITY,max_angle=MAX_STEERING_ANGLE, dead_zone=DEAD_ZONE_THRESHOLD):
    global difference
    steering_angle_deg = 0.0  
    if ratio_l == float('inf') and ratio_r == float('inf'):
        steering_angle_deg = 0.0
    elif ratio_l == float('inf'):
        steering_angle_deg = -max_angle 
    elif ratio_r == float('inf'):
        steering_angle_deg = max_angle   
    else:
        difference = ratio_l - ratio_r
        if abs(difference) < dead_zone:
            steering_angle_deg = 0.0
        else:
            steering_angle_deg = sensitivity * difference
            steering_angle_deg = max(-max_angle, min(max_angle, steering_angle_deg))
    servo_value = int(round(steering_angle_deg + 90))
    servo_value = max(60, min(130, servo_value))
    return servo_value

# --- 1. Load the PyTorch Model ---
model = None
if ULTRALYTICS_AVAILABLE:
    model = YOLO(PYTORCH_MODEL_PATH) 
    model.to(DEVICE)
else:
    model = torch.jit.load(PYTORCH_MODEL_PATH, map_location=DEVICE)
model.eval()

# --- 2. Initialize Picamera2 ---
piCam = Picamera2()
piCam.preview_configuration.main.size=(640,480) #Äá»™ phÃ¢n giáº£i 640x480
piCam.preview_configuration.main.format='RGB888'
piCam.preview_configuration.controls.FrameRate=90 #Tá»‘c Ä‘á»™ khung hÃ¬nh
piCam.preview_configuration.align()
piCam.configure('preview')
piCam.start()
time.sleep(0.2) 

# --- 3. Real-time Segmentation Loop ---
prev_time = 0
try:
    while True:
        frame_rgb = piCam.capture_array() 
        if frame_rgb.shape[2] == 4:frame_rgb = cv2.cvtColor(frame_rgb, cv2.COLOR_RGBA2RGB)
        original_height, original_width = frame_rgb.shape[:2]
        current_time = time.time()
        fps = 1 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
        prev_time = current_time

        # --- Preprocess for PyTorch model ---
        letterboxed_img_rgb, ratio_wh, pad_wh = letter_box(frame_rgb, color=LETTERBOX_COLOR)
        img_tensor = torch.from_numpy(letterboxed_img_rgb).to(DEVICE)
        img_tensor = img_tensor.permute(2, 0, 1).contiguous()
        img_tensor = img_tensor.float() / 255.0 
        if img_tensor.ndimension() == 3: img_tensor = img_tensor.unsqueeze(0) 

        # --- PyTorch Inference ---
        lane_mask_binary_final = np.zeros((original_height, original_width), dtype=np.uint8) 
        with torch.no_grad(): 
            if ULTRALYTICS_AVAILABLE and isinstance(model, YOLO):
                 results = model.predict(source=frame_rgb, imgsz=MODEL_INPUT_WIDTH, conf=CONFIDENCE_THRESHOLD, device=DEVICE, verbose=False)
            else:
                 raw_output = model(img_tensor) 
                 if isinstance(raw_output, torch.Tensor):
                    class MockMasks:
                        def __init__(self, data_tensor): self.data = data_tensor 
                    class MockResult:
                        def __init__(self, masks_obj): self.masks = masks_obj
                    if raw_output.ndim == 4 and raw_output.shape[1] == 1: 
                         results = [MockResult(MockMasks(raw_output.squeeze(1)))]
                    elif raw_output.ndim == 4 and raw_output.shape[1] > 0: 
                         lane_masks_tensor = raw_output[:, 0:1, :, :] 
                         results = [MockResult(MockMasks(lane_masks_tensor.squeeze(1)))]
                    else: results = None 
                 else: results = None

        # --- Post-process Masks (common for both Ultralytics and potentially other models) ---
        if results and results[0].masks is not None and len(results[0].masks.data) > 0:
            lane_mask_binary_final = process_pytorch_masks(results, MODEL_INPUT_HEIGHT, MODEL_INPUT_WIDTH, original_height, original_width, ratio_wh, pad_wh, MASK_THRESHOLD)
        display_frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        display_frame_bgr[lane_mask_binary_final == 255] = [255, 0, 0] 
        inverted_binary_mask = cv2.bitwise_not(lane_mask_binary_final)
        main_ratio_left, main_ratio_right = calculate_lane_background_ratios(inverted_binary_mask)
        steering_angle = calculate_steering_from_ratios(main_ratio_left, main_ratio_right)
        key=cv2.waitKey(1)&0xFF
        signal_motor(key)
        rpm_cmd = transform_speed(speed_motor_requested)
        cv2.imshow("Binary Mask (PyTorch - Drivable=Black)", inverted_binary_mask)
        print(f"ratio left: {ratio_left}, ratio right: {ratio_right}, diff: {difference}, Steering: {steering_angle}, Speed: {speed_motor_requested}")
        if ser:
            data= struct.pack('<ii', steering_angle, rpm_cmd)
            ser.write(b'<'+data+b'>'); ser.flush()
        if key == ord('q'):
            data= struct.pack('<ii', 90 , 0)
            ser.write(b'<'+data+b'>'); ser.flush()
            print("Exiting loop by user request ('q' pressed).")
            break

except KeyboardInterrupt:
    print("\nInterrupted by user (KeyboardInterrupt). Exiting...")
except Exception as e_outer:
    print(f"--- FATAL PYTHON ERROR in main processing loop ---")
    import traceback
    traceback.print_exc()
finally:
    print("Cleaning up resources...")
    if 'picam2' in locals() and hasattr(piCam, 'started') and piCam.started:
        piCam.stop()
        print("Picamera2 stopped.")
    cv2.destroyAllWindows()
    print("OpenCV windows destroyed. Script finished.")
