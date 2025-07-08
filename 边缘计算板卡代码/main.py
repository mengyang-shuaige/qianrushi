import os
import cv2
import socket
import numpy as np
import torch
import sys
import requests
import paho.mqtt.client as mqtt
import json
import base64
import hmac
import time
from urllib.parse import quote
import threading
import gradio as gr
from py_utils.coco_utils import COCO_test_helper
from paddleocr import PaddleOCR
import threading
from queue import Queue
import numpy as np
import logging

logging.basicConfig(level=logging.INFO)

# 初始化 PaddleOCR，使用轻量级模型，关闭使用 GPU
ocr = PaddleOCR(use_angle_cls=True, lang="en", use_gpu=False)

# 尝试设置 ppocr 的日志级别
for logger_name in logging.root.manager.loggerDict:
    if 'ppocr' in logger_name:
        logging.getLogger(logger_name).setLevel(logging.INFO)

OBJ_THRESH = 0.25
NMS_THRESH = 0.45


# 定义全局队列，最大容量为10帧（避免内存溢出）
frame_queue = Queue(maxsize=10)

global q
global imagecount
global a1,a2,a3
global count,rknn_lite,tcp_server_socket,client_socket,clientAddr
result=0

ServerUrl = "mqtts.heclouds.com" #服务器url
ServerPort = 1883#服务器端口
DeviceName="aaa" #设备ID
Productid = "qcyKlB2e49" #产品ID
accesskey="o6tNt73lVffzjW8dU5+BWcXx1w/r/xOdW5n5EQd/icM="

# 发布的topic
Pub_topic1 = "$sys/"+Productid+"/"+ DeviceName+"/dp/post/json"
#数据上传成功的消息
Sub_topic1 = "$sys/"+Productid+"/"+DeviceName+"/dp/post/json/accepted"
#接收数据上传失败的消息
Sub_topic2 = "$sys/"+Productid+"/"+DeviceName+"/dp/post/json/rejected"



IMG_SIZE = (640, 640)  # (width, height)


CLASSES = ("person", "bicycle", "car", "Electric bicycle ", "aeroplane ", "bus ", "train", "truck ", "boat", "traffic light",
           "fire hydrant", "stop sign ", "parking meter", "bench", "bird", "cat", "dog ", "horse ", "sheep", "cow",
           "elephant",
           "bear", "zebra ", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
           "snowboard", "sports ball", "kite",
           "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
           "fork", "knife ",
           "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza ", "donut",
           "cake", "chair", "sofa",
           "pottedplant", "bed", "diningtable", "toilet ", "tvmonitor", "laptop	", "mouse	", "remote ",
           "keyboard ", "cell phone", "microwave ",
           "oven ", "toaster", "sink", "refrigerator ", "book", "clock", "vase", "scissors ", "teddy bear ",
           "hair drier", "toothbrush ")



coco_id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 31, 32,
                33, 34,
                35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61,
                62, 63,
                64, 65, 67, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90]

    
# 总结令牌
def get_token(id,access_key):
    version = '2018-10-31'
 #   res = 'products/%s' % id  # 通过产品ID访问产品APIequi
    # res = 'userid/%s' % id  # 通过产品ID访问产品API
    res="products/"+ Productid + "/devices/" + DeviceName
    # 用户自定义token过期时间
    et = str(int(time.time()) + 72000000)
    # et = str(int(1722499200))
    # 签名方法，支持md5、sha1、sha256
    method = 'sha1'
    method1 = 'sha256'
    # 对access_key进行decode
    key = base64.b64decode(access_key)

# 计算sign
    org = et + '\n' + method+ '\n' + res + '\n' + version
    sign_b = hmac.new(key=key, msg=org.encode(), digestmod=method)
    sign = base64.b64encode(sign_b.digest()).decode()

# value 部分进行url编码
    sign = quote(sign, safe='')
    res = quote(res, safe='')

# token参数拼接
    token = 'version=%s&res=%s&et=%s&method=%s&sign=%s' % (version, res, et, method, sign)

    return token




def on_subscribe(client, userdata, mid, reason_code_list, properties):

    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")



def on_unsubscribe(client, userdata, mid, reason_code_list, properties):

    if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
        print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
    else:
        print(f"Broker replied with failure: {reason_code_list[0]}")
    client.disconnect()



# 当客户端收到来自服务器的CONNACK响应时的回调。也就是申请连接，服务器返回结果是否成功等
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        print("连接结果:" + mqtt.connack_string(reason_code))
        #连接成功后就订阅topic
        client.subscribe(Sub_topic1)
      


# 从服务器接收发布消息时的回调。
def on_message(client, userdata, message):
    print(str(message.payload,'utf-8'))


#查看向服务器发布的消息on_publish()回调将会被触发
def on_publish(client, userdata, mid):
    print(str(mid))


# 1.初步提取候选边界框的属性
def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold.
    """
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores



# 2.进一步提取边界框位置
def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.
    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep



# 计算给定位置的深度预测
def dfl(position):

    x = torch.tensor(position)
    n, c, h, w = x.shape
    p_num = 4
    mc = c // p_num
    y = x.reshape(n, p_num, mc, h, w)
    y = y.softmax(2)
    acc_metrix = torch.tensor(range(mc)).float().reshape(1, 1, mc, 1, 1)
    y = (y * acc_metrix).sum(2)
    return y.numpy()



# 处理给定位置的边界框坐标
def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1] // grid_h, IMG_SIZE[0] // grid_w]).reshape(1, 2, 1, 1)

    position = dfl(position)
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)

    return xyxy



# 输出转换的坐标和物品
def post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    defualt_branch = 3
    pair_per_branch = len(input_data) // defualt_branch
    # Python 忽略 score_sum 输出
    for i in range(defualt_branch):
        boxes.append(box_process(input_data[pair_per_branch * i]))
        classes_conf.append(input_data[pair_per_branch * i + 1])
        scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]
    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    # nms
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)
    return boxes, classes, scores

# 在图像上可视化目标检测的结果，选择识别什么物品
def draw(image,imageone, boxes, scores, classes):
    global a1,a2,a3,result
    for box, score, cl in zip(boxes, scores, classes):
        if cl in [3]:
            top, left, right, bottom = [int(_b) for _b in box]
            print("%s @ (%d %d %d %d) %.3f" % (CLASSES[cl], top, left, right, bottom, score))
            cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
            cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),(top, left - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.rectangle(imageone, (top, left), (right, bottom), (255, 0, 0), 2)
            cv2.putText(imageone, '{0} {1:.2f}'.format(CLASSES[cl], score),(top, left - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            a1=a2
            a2=a3
            a3=1
            result=a1+a2+a3
            print(result)
        else:
            a1=a2
            a2=a3
            a3=0
            result=a1+a2+a3
            print(result)


            

# 使用YOLOv10进行目标检测，并返回结果信息
def yolov10_inference(image):
    # start_time = time.time()
    global q,a1,a2,a3,imagecount,img3,img4
    from rknnlite.api import RKNNLite
    global count,rknn_lite,result
    print("线程1开始执行")
    co_helper = COCO_test_helper(enable_letter_box=True)
    img1=cv2.resize(image,[640,640])
    img1 = cv2.cvtColor(img1, cv2.COLOR_RGB2BGR)

    
    if count ==0:
        count=1

        rknn_lite = RKNNLite()
        print('--> Load RKNN model')
        ret = rknn_lite.load_rknn('./yolov10s.rknn')
        if ret != 0:
            print('Load RKNN model failed')
            exit(ret)
        print('done')
 
    img3= co_helper.letter_box(im=img1.copy(), new_shape=(IMG_SIZE[1], IMG_SIZE[0]), pad_color=(0, 0, 0))
    input_data = np.expand_dims(img3, 0)
    
    ret = rknn_lite.init_runtime()
    outputs = rknn_lite.inference([input_data])
    boxes, classes, scores = post_process(outputs)
    # 在图像上绘制边界框并打印类别信息
    if boxes is not None:
        img4 = img3.copy()
        draw(img4,img3, co_helper.get_real_box(boxes), scores, classes)
        # a1=a2
        # a2=a3
        # a3=1
        # result=a1+a2+a3
        result=3
        print(result)
                
    else:
        # a1=a2
        # a2=a3
        # a3=0
        # result=a1+a2+a3
        result=1
        print(result)

    img3 = cv2.cvtColor(img3, cv2.COLOR_BGR2RGB)
    # end_time = time.time()
    # duration = end_time - start_time
    # print(f"程序运行时长：{duration}秒")
    return img3



def sign_send():
    global client_socket,img3,result,imagecount
    print("线程2开始执行")
    while True: 
        if result==3:
            send_data1 = '1'
            client_socket.send(send_data1.encode("gbk"))
            time.sleep(1.5)
        else:
            send_data1 = '0'
            client_socket.send(send_data1.encode("gbk"))
            time.sleep(1.5)      

    
def creat_file_and_send_to_mqtt():
    global img4, result,imagecount
    print("线程3开始执行")
    while True: 
        if result==3:   
            # img4 = cv2.cvtColor(img4, cv2.COLOR_BGR2RGB)     
            cv2.imwrite(f'/home/zonesion/Desktop/RK3588/yolov10-main/static/images/{imagecount}.jpg',img4)
            # cv2.imwrite(f'/home/zonesion/Desktop/RK3588/yolov10-main/static/images/result.jpg',img4)
            cv2.waitKey(5)        
            dir_path = f'/home/zonesion/Desktop/RK3588/yolov10-main/static/images/{imagecount}.jpg'
            if os.path.exists(dir_path):
            
                url = "https://iot-api.heclouds.com/device/file-upload"

                payload = {'product_id': 'qcyKlB2e49',
                'device_name': 'aaa'}
                files=[
                ('file',(f'{imagecount}.jpg',open(f'/home/zonesion/Desktop/RK3588/yolov10-main/static/images/{imagecount}.jpg','rb'),'image/jpeg'))
                    ]
                headers = {
                'Authorization': 'version=2022-05-01&res=userid%2F389167&et=1723338208&method=sha1&sign=Y2GwCenm2Rw6DZX%2FxQPUfaVDmJQ%3D'
                    }
                
                response = requests.request("POST", url, headers=headers, data=payload, files=files)

                print(response.text)
                text1=  "  "  
                data1 = {
                    "id":13,
                    "dp": {
                        "zhuzhu": [
                            {
                                "v": text1
                            }
                        ]
                    }
                }
                text2=  "警告，系统已监测到电动车靠近！"  
                data2 = {
                    "id":14,
                    "dp": {
                        "zhizhi": [
                            {
                                "v": text2
                            }
                        ]
                    }
                }
                
                json_str1 = json.dumps(data1)
                mqttc.publish(Pub_topic1, json_str1, qos=0)
                json_str2 = json.dumps(data2)
                mqttc.publish(Pub_topic1, json_str2, qos=0)
                print("上传成功")
            imagecount=imagecount+1
            time.sleep(1)
        
        else:    

            text2= '  '
            data2 = {
                "id":14,
                "dp": {
                    "zhizhi": [
                        {
                            "v": text2
                        }
                    ]
                }
            }
            text1=  "系统运行良好，未发现异常"  
            data1 = {
            "id": 13,
            "dp": {
                "zhuzhu": [
                    {
                        "v": text1
                    }
                ]
            }
            }
    
            json_str2 = json.dumps(data2)
            mqttc.publish(Pub_topic1, json_str2, qos=0)
            json_str1 = json.dumps(data1)
            mqttc.publish(Pub_topic1, json_str1, qos=0)
            print("无图片上传")
            time.sleep(1)        
            
   
def receive():
    while True:
        received_data = client_socket.recv(1024)
        if not received_data:
            print(f"客户端 {clientAddr} 断开连接")
            break
        
        decoded_data = received_data.decode("utf-8").strip()  # 假设使用utf-8编码
        
        print(f"收到来自 {clientAddr} 的数据: {decoded_data}")
        try:
            # 分割字符串
            parts = decoded_data.split(', ')
            
            # 提取各部分数值
            flame = int(parts[0].split(': ')[1].replace(' mV', ''))
            water = int(parts[1].split(': ')[1].replace(' mV', ''))
            smoke = int(parts[2].split(': ')[1].replace(' mV', ''))
            distance_str = parts[3].split(': ')[1].replace(' cm', '')
            distance = float(distance_str)  # 转换为浮点数（或按需取整）
            print(f"解析结果: Flame={flame} mV, Water={water} mV, Smoke={smoke} mV, distance={distance}cm")
            
            # text3=  f"{flame},{water},{smoke},{distance}"  
            # data3 = {
            #     "id":13,
            #     "dp": {
            #         "chuanganqi": [
            #             {
            #                 "v": text3
            #             }
            #         ]
            #     }
            # }        
            # json_str3 = json.dumps(data3)
            # mqttc.publish(Pub_topic1, json_str3, qos=0)

                        
            text3_1=  f"{flame}"  
            data3_1 = {
                "id":13,
                "dp": {
                    "flame": [
                        {
                            "v": text3_1
                        }
                    ]
                }
            }        
            json_str3_1 = json.dumps(data3_1)
            mqttc.publish(Pub_topic1, json_str3_1, qos=0)

            text3_2=  f"{water}"  
            data3_2= {
                "id":13,
                "dp": {
                    "water": [
                        {
                            "v": text3_2
                        }
                    ]
                }
            }        
            json_str3_2 = json.dumps(data3_2)
            mqttc.publish(Pub_topic1, json_str3_2, qos=0)
            
            text3_3=  f"{smoke}"  
            data3_3 = {
                "id":13,
                "dp": {
                    "smoke": [
                        {
                            "v": text3_3
                        }
                    ]
                }
            }        
            json_str3_3 = json.dumps(data3_3)
            mqttc.publish(Pub_topic1, json_str3_3, qos=0)
            
        
            


            if distance > 2.5:
                print("距离大于 2.5cm，执行特定程序")

                text_4="无车"
                date_4= {
                    "id":13,
                    "dp": {
                        "kongbai_1": [
                            {
                                "v": text_4
                            }
                        ]
                    }
                }    
                text_5= '  '
                date_5= {
                    "id":13,
                    "dp": {
                        "kongbai": [
                            {
                                "v": text_5
                            }
                        ]
                    }
                }                               
                json_str3_6 = json.dumps(date_4)  
                mqttc.publish(Pub_topic1, json_str3_6, qos=0)            
                json_str3_7 = json.dumps(date_5)  
                mqttc.publish(Pub_topic1, json_str3_7, qos=0)  
                
            else:

                text_4= '  '
                date_4= {
                    "id":13,
                    "dp": {
                        "kongbai_1": [
                            {
                                "v": text_4
                            }
                        ]
                    }
                }    
                text_5="有车"
                date_5= {
                    "id":13,
                    "dp": {
                        "kongbai": [
                            {
                                "v": text_5
                            }
                        ]
                    }
                }                               
                json_str3_6 = json.dumps(date_4)  
                mqttc.publish(Pub_topic1, json_str3_6, qos=0)            
                json_str3_7 = json.dumps(date_5)  
                mqttc.publish(Pub_topic1, json_str3_7, qos=0)        


        except Exception as e:
            print(f"解析数据时出错: {e}")

        time.sleep(1)

def read_rtsp_stream(rtsp_url, queue):
    """
    独立线程：实时读取RTSP视频流并放入队列
    """
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("无法打开 RTSP 视频流")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取帧，尝试重新连接...")
                time.sleep(1)  # 重试间隔
                continue

            # 队列满时丢弃旧帧，保证处理最新帧
            if queue.full():
                queue.get()  # 移除队列中最旧的帧
            queue.put(frame)

    except Exception as e:
        print(f"RTSP读取线程错误: {e}")
    finally:
        cap.release()
        print("RTSP读取线程关闭")

def process_frame(queue):
    """
    独立线程：从队列中获取帧并进行OCR处理
    """

    while True:
        frame = queue.get()  # 阻塞直到队列中有帧
        if frame is None:
            break  # 退出信号

        try:
            # 裁剪图像（根据实际需求调整裁剪区域）
            cropped = frame[0:40, 60:100]
            # # 显示处理后的图像（可选，实时显示可能影响性能）
            # cv2.imshow('Processed Image', cropped)
            # cv2.waitKey(1)
            # 使用 PaddleOCR 进行字符识别
            result = ocr.ocr(cropped, cls=True)

            # 输出识别结果
            for line in result:
                for word_info in line:
                    text = word_info[1][0]
                    confidence = word_info[1][1]
                    # print(f"识别文本: {text}, 置信度: {confidence:.2f}")
                    if confidence > 0.95:
                        # send_data2 = f"温度：{text}"
                        # send_data2 = text
                        # try:
                        #     client_socket.send(send_data2.encode("gbk"))
                        #     print(f"发送完毕:{text}")
                        # send_data2 = f"TEMP:{text}"
                        send_data2 = f"{text}"
                        try:
                            client_socket.send(send_data2.encode("gbk"))
                            print(f"发送完毕:{text}")


                            text4=  f"{text}"  
                            data4 = {
                        "id":13,
                        "dp": {
                            "wendu": [
                                {
                                    "v": text4
                                }
                            ]
                        }
                    }        
                            json_str4 = json.dumps(data4)
                            mqttc.publish(Pub_topic1, json_str4, qos=0)

                        
                            text4_1= "Ａ区"
                            data4_1 = {
                        "id":13,
                        "dp": {
                            "A": [
                                {
                                    "v": text4_1
                                }
                            ]
                        }
                    }        
                            json_str4 = json.dumps(data4_1)
                            mqttc.publish(Pub_topic1, json_str4, qos=0)
                

                            text4_2= '  '
                            data4_2 = {
                        "id":13,
                        "dp": {
                            "B": [
                                {
                                    "v": text4_2
                                }
                            ]
                        }
                    }        
                            json_str4 = json.dumps(data4_2)
                            mqttc.publish(Pub_topic1, json_str4, qos=0)
                                  






                            if text>60:
                                text4_1= "0"
                                data4_1 = {
                            "id":13,
                            "dp": {
                                "A_": [
                                    {
                                        "v": text4_1
                                    }
                                ]
                            }
                        }        
                                json_str4 = json.dumps(data4_1)
                                mqttc.publish(Pub_topic1, json_str4, qos=0)
                    

                                text4_2= "B区"
                                data4_2 = {
                            "id":13,
                            "dp": {
                                "B_": [
                                    {
                                        "v": text4_2
                                    }
                                ]
                            }
                        }        
                                json_str4 = json.dumps(data4_2)
                                mqttc.publish(Pub_topic1, json_str4, qos=0)
                        except Exception as e:
                            print(f"发送数据时出错: {e}")

                 

        except Exception as e:
            # print(f"图像处理错误: {e}")
            print()
        finally:
            queue.task_done()  # 标记任务完成


def sign_send():
    global client_socket,img3,result,imagecount
    print("线程2开始执行")
    while True: 
        if result==3:
            send_data1 = '1'
            client_socket.send(send_data1.encode("gbk"))
            time.sleep(1.5)
        else:
            send_data1 = '0'
            client_socket.send(send_data1.encode("gbk"))
            time.sleep(1.5)      


# # 使用Gradio创建接口
# iface = gr.Interface(
#     fn=yolov10_inference,
#     inputs=gr.Image(sources=['webcam'], streaming=True),
#     outputs=gr.Image(type="numpy", label="Annotated Image", visible=True),
#     live=True,
#     title='智眸守卫实时监测'
# )

iface = gr.Interface(
    fn=yolov10_inference,
    inputs=gr.Image(source='webcam', streaming=True),
    outputs=gr.Image(type="numpy", label="Annotated Image", visible=True),
    live=True,
    title='智眸守卫实时监测'
)




# 创建并连接MQTT客户端
if __name__ == '__main__':
    a1=0
    a2=0
    a3=0
    imagecount=0
  
    passw=get_token(DeviceName,accesskey)
    print(passw)
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,DeviceName)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message
    mqttc.on_subscribe = on_subscribe
    mqttc.on_unsubscribe = on_unsubscribe

    mqttc.connect(ServerUrl, port=ServerPort, keepalive=120)

    mqttc.username_pw_set(Productid,passw)
   
    mqttc.loop_start()
    
    img1=np.zeros([640,640,3], np.uint8)
    count=0
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # # 本地信息
    address = ('', 7788)
    # # 绑定
    tcp_server_socket.bind(address)
    tcp_server_socket.listen(128) 
    print("服务端已启动，等待客户端连接...")
    # # 创建socket
    client_socket, clientAddr = tcp_server_socket.accept()
    print(f"客户端 {clientAddr} 已连接")

    rtsp_url = "rtsp://admin:Liu123456@192.168.1.64:554/h264/ch2/main/av_stream" 

    thread1 = threading.Thread(target=yolov10_inference)
    thread1.start()
    # thread2 = threading.Thread(target=sign_send)
    # thread2.start()
    thread3 = threading.Thread(target=creat_file_and_send_to_mqtt)
    thread3.start()
    thread4 = threading.Thread(target=receive)
    thread4.start()    

    read_thread = threading.Thread(target=read_rtsp_stream, args=(rtsp_url, frame_queue), daemon=True)
    read_thread.start()


    process_thread = threading.Thread(target=process_frame, args=(frame_queue,), daemon=True)
    process_thread.start()

    # 启动接口
    iface.launch(share=True)

    client_socket.close()
    tcp_server_socket.close()

    
        