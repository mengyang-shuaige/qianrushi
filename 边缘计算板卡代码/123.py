
from flask import Flask, Response, render_template
import cv2

app = Flask(__name__)

RTSP_URL = "rtsp://admin:Liu123456@192.168.1.64:554/h264/ch1/main/av_stream"

def generate_frames():
    cap = cv2.VideoCapture(RTSP_URL)
    while cap.isOpened():
        success, frame = cap.read()
        if not success: break
        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index3.html')  # 从模板目录加载HTML文件

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)