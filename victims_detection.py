import torch
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
import logging
import threading
import queue
import time
import os
import serial

app = Flask(__name__)

# Configuration UART
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    time.sleep(2)
except Exception as e:
    ser = None
    logging.warning(f"UART non initialisé: {e}")

# Configuration
CONF_THRESH = 0.3
NMS_THRESH = 0.4
IMG_SIZE = 320
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
MAX_QUEUE_SIZE = 2
JPEG_QUALITY = 80
DEBUG_MODE = True

logging.basicConfig(
    level=logging.DEBUG if DEBUG_MODE else logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

frame_queue = queue.Queue(maxsize=MAX_QUEUE_SIZE)
current_detections = []

try:
    model = torch.jit.load('bestmaq.torchscript', map_location='cpu')
    model.eval()
    logging.info("Modèle chargé avec succès")
    with torch.no_grad():
        test_input = torch.rand(1, 3, IMG_SIZE, IMG_SIZE)
        test_output = model(test_input)
        logging.info(f"Shape des sorties du modèle: {test_output[0].shape}")
except Exception as e:
    logging.error(f"ERREUR de chargement du modèle: {e}")
    exit(1)

def preprocess(frame):
    if frame is None:
        logging.error("Frame None dans preprocess")
        return None
    try:
        img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.transpose(img, (2, 0, 1))
        return np.ascontiguousarray(img).astype(np.float32) / 255.0
    except Exception as e:
        logging.error(f"ERREUR preprocess: {e}")
        return None

def apply_nms(detections):
    if len(detections) == 0:
        return []
    try:
        boxes = []
        scores = []
        for det in detections:
            x1, y1, x2, y2, conf = det
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(conf)
        indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESH, NMS_THRESH)
        if indices is None or len(indices) == 0:
            return []
        return [detections[i] for i in indices.flatten()]
    except Exception as e:
        logging.error(f"ERREUR NMS: {e}")
        return detections

def capture_thread():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logging.error("Échec d'ouverture de la caméra")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, DISPLAY_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DISPLAY_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 20)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.warning("Échec de lecture de la frame")
            time.sleep(0.1)
            continue
        if not frame_queue.full():
            frame_queue.put(frame)
def process_frame(frame):
    if frame is None:
        return []

    try:
        img_tensor = torch.from_numpy(preprocess(frame)).unsqueeze(0)
        with torch.no_grad():
            pred = model(img_tensor)

            detections = []

            output = pred[0].squeeze(0).cpu().numpy()

            if len(output.shape) != 2 or output.shape[1] != 6:
                logging.warning(f"Format inattendu de la sortie : {output.shape}")
                return []

            for det_np in output:

                try:
                    x_center, y_center, width, height, conf, cls_conf = det_np

                    total_conf = conf * cls_conf
                    if total_conf < CONF_THRESH:
                        continue

                    x1 = int(x_center - width / 2)
                    y1 = int(y_center - height / 2)
                    x2 = int(x_center + width / 2)
                    y2 = int(y_center + height / 2)

                    x1 = max(0, min(x1, frame.shape[1] - 1))
                    y1 = max(0, min(y1, frame.shape[0] - 1))
                    x2 = max(0, min(x2, frame.shape[1] - 1))
                    y2 = max(0, min(y2, frame.shape[0] - 1))

                    detections.append((x1, y1, x2, y2, total_conf))

                except Exception as e:
                    logging.error(f"Erreur lors de l'extraction des valeurs de détection: {e}")
                    continue

            filtered_detections = apply_nms(detections)

            if ser and filtered_detections:
                try:
                    ser.write(b'LED_ON\n')
                    logging.info("Signal UART envoyé: LED_ON")
                except Exception as e:
                    logging.error(f"Erreur UART: {e}")

            return filtered_detections

    except Exception as e:
        logging.error(f"ERREUR process_frame: {e}")
        return []

def generate_frames():
    frame_count = 0
    while True:
        try:
            frame = frame_queue.get(timeout=0.1)
            detections = process_frame(frame)
            current_detections = detections
            output_frame = frame.copy()
            for (x1, y1, x2, y2, conf) in detections:
                cv2.rectangle(output_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(output_frame, f"{conf:.2f}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            _, buffer = cv2.imencode('.jpg', output_frame,
                                    [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except queue.Empty:
            continue
        except Exception as e:
            logging.error(f"ERREUR generate_frames: {e}")

@app.route('/')
def index():
    return render_template_string("<html><body><h1>Detection de personnes en detresse</h1><img src='/video_feed' /></body></html>")

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    thread = threading.Thread(target=capture_thread)
    thread.daemon = True
    thread.start()
    app.run(host='0.0.0.0', port=5000, threaded=True)