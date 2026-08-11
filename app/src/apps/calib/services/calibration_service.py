import sys
import os
import logging
from typing import List, Tuple, Dict, Any
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from core.vision.reconstruction import PoissonReconstructor # Dummy import for now or use actual solver from aisprayer
# The solver in the original code is at src/aisprayer/tools/calib/calib_ui/core/calib_solver.py
# Assuming we will move or use that solver eventually. For now, mocking the algorithm interface.

logger = logging.getLogger(__name__)

class CalibrationService:
    def __init__(self):
        self._samples: List[Dict[str, Any]] = []

    def add_sample(self, image: np.ndarray, robot_pose: List[float]) -> int:
        sample_id = len(self._samples)
        self._samples.append({
            "id": sample_id,
            "image": image,
            "pose": robot_pose
        })
        return len(self._samples)

    def delete_sample(self, index: int) -> bool:
        for i, s in enumerate(self._samples):
            if s["id"] == index:
                self._samples.pop(i)
                return True
        return False

    def get_samples_info(self) -> List[Dict[str, Any]]:
        # Returns metadata without the heavy image data
        return [{"id": s["id"], "pose": s["pose"]} for s in self._samples]

    def run_calibration(self) -> Dict[str, Any]:
        """
        Runs the hand-eye calibration solver on collected samples.
        """
        if len(self._samples) < 3:
            return {"success": False, "error": "Not enough samples. Need at least 3."}

        # TODO: Call the actual calib_solver.py functions here.
        # This is a placeholder for the algorithm execution.
        logger.info(f"Running calibration with {len(self._samples)} samples.")
        
        # Mock result
        dummy_matrix = np.eye(4).tolist()
        dummy_error = 0.5
        
        return {
            "success": True,
            "matrix": dummy_matrix,
            "reproj_error": dummy_error
        }

    def save_calibration(self, db_session) -> bool:
        # Here we would save to the calib_records table
        pass

calibration_service = CalibrationService()
