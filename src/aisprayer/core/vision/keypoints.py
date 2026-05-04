import torch
import torchvision
from torchvision.models.detection import keypointrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.keypoint_rcnn import KeypointRCNNPredictor
from safetensors.torch import load_model
import cv2
import numpy as np
from PIL import Image
import argparse
import os

class TrousersKeypoints:
    """
    A class for detecting keypoints on trousers using a fine-tuned Keypoint R-CNN model.
    Based on the kengboon/keypointrcnn-trousers model from Hugging Face.
    """
    def __init__(self, model_path, device=None):
        """
        Initialize the detector.
        
        Args:
            model_path (str): Path to the model.safetensors file.
            device (str): Device to run the model on ('cuda' or 'cpu').
        """
        self.device = device if device else ("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Loading model on {self.device}...")
        self.model = self._load_model(model_path)
        self.model.to(self.device)
        self.model.eval()

    def _load_model(self, model_path):
        # Load a pre-trained Keypoint RCNN model foundation (weights=None to avoid downloading defaults)
        model = keypointrcnn_resnet50_fpn(weights=None)

        # Replace model's head as per model card specifications for 14 keypoints
        # num_classes = 2 (Background + Trousers)
        num_classes = 2
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

        # num_keypoints = 14
        num_keypoints = 14
        in_features_kpt = model.roi_heads.keypoint_predictor.kps_score_lowres.in_channels
        model.roi_heads.keypoint_predictor = KeypointRCNNPredictor(in_features_kpt, num_keypoints)

        # Load the safetensors weights
        load_model(model, model_path, device=str(self.device))
        
        return model

    def predict(self, image_path, threshold=0.5):
        """
        Predict keypoints for a given image.
        
        Args:
            image_path (str): Path to the input image.
            threshold (float): Confidence threshold for detections.
            
        Returns:
            dict: Detection result containing boxes, keypoints, and scores.
        """
        img = Image.open(image_path).convert("RGB")
        img_tensor = torchvision.transforms.functional.to_tensor(img).to(self.device)
        
        with torch.no_grad():
            prediction = self.model([img_tensor])
        
        # Filter by score
        scores = prediction[0]['scores'].cpu().numpy()
        high_scores_idxs = np.where(scores > threshold)[0]
        
        if len(high_scores_idxs) == 0:
            return None
        
        # Return the highest scoring detection
        best_idx = high_scores_idxs[0]
        
        keypoints = prediction[0]['keypoints'][best_idx].cpu().numpy()
        score = scores[best_idx]
        
        print(f"\n[DEBUG] Image Path: {image_path}")
        print(f"[DEBUG] Detection Score: {score:.4f}")
        print(f"[DEBUG] Detected {len(keypoints)} keypoints:")
        for i, (x, y, v) in enumerate(keypoints):
            print(f"  Keypoint {i}: x={x:.2f}, y={y:.2f}, visibility={v:.2f}")

        result = {
            'boxes': prediction[0]['boxes'][best_idx].cpu().numpy(),
            'keypoints': keypoints,
            'scores': score
        }
        return result

    def visualize(self, image_path, result, output_path, polygon=None):
        """
        Visualize detection results and save to an image.
        
        Args:
            image_path (str): Path to the original input image.
            result (dict): Prediction result from the predict method.
            output_path (str): Path to save the visualized image.
            polygon (numpy.ndarray, optional): Physical silhouette polygon to draw.
        """
        if result is None:
            print("No trousers detected above the threshold.")
            return

        img = cv2.imread(image_path)
        if img is None:
            print(f"Error: Could not read image at {image_path}")
            return

        # 0. 如果提供了物理轮廓，先画出来 (黄色，加粗)
        if polygon is not None:
            poly_pts = np.array(polygon, np.int32).reshape((-1, 1, 2))
            cv2.polylines(img, [poly_pts], True, (0, 255, 255), 2)
            cv2.putText(img, "PHYSICAL CONTOUR", (int(polygon[0,0]), int(polygon[0,1]-25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        keypoints = result['keypoints']
        
        # Specific connection order (1-based: 1-4-5-6-7-8-9-10-11-12-13-14-3-2)
        # 0-based mapping: [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 2, 1]
        connection_indices = [0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 2, 1]
        
        pts = []
        for idx in connection_indices:
            if idx < len(keypoints):
                x, y, v = keypoints[idx]
                if v > 0.5:
                    pts.append([int(x), int(y)])
        
        # Draw lines between keypoints in the specified sequence
        if len(pts) > 1:
            pts_array = np.array(pts, np.int32).reshape((-1, 1, 2))
            cv2.polylines(img, [pts_array], isClosed=True, color=(0, 255, 0), thickness=2) # Green lines

        # Draw keypoints and labels
        for i, (x, y, v) in enumerate(keypoints):
            if v > 0.5: # visibility threshold
                # Draw point (Red)
                cv2.circle(img, (int(x), int(y)), 6, (0, 0, 255), -1)
                # Draw label (Orange, starting from 1)
                cv2.putText(img, str(i + 1), (int(x), int(y) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        # Draw bounding box (Blue)
        box = result['boxes']
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 2)
        
        # Add score label
        score = result['scores']
        cv2.putText(img, f"Trousers: {score:.2f}", (int(box[0]), int(box[1]) - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imwrite(output_path, img)
        print(f"Success! Result saved to {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Trousers Keypoint Detection CLI")
    parser.add_argument("image", help="Path to the input trouser image")
    parser.add_argument("--model", default="models/model.safetensors", help="Path to model.safetensors weights")
    parser.add_argument("--output", default="data/images/output_keypoints.png", help="Path to save output visualization")
    parser.add_argument("--threshold", type=float, default=0.8, help="Detection confidence threshold")
    
    args = parser.parse_args()
    
    # Ensure model path is correct
    model_path = args.model
    if not os.path.isabs(model_path):
        # Try to find it relative to workspace root if it's not absolute
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.dirname(script_dir)
        model_path = os.path.join(workspace_root, args.model)

    if not os.path.exists(model_path):
        print(f"Error: Model file not found at {model_path}")
        return

    if not os.path.exists(args.image):
        print(f"Error: Image file not found at {args.image}")
        return

    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize and run detection
    try:
        detector = TrousersKeypoints(model_path)
        result = detector.predict(args.image, threshold=args.threshold)
        detector.visualize(args.image, result, args.output)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
