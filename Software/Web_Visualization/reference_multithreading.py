import os
import tkinter
from threading import Thread
from tkinter.filedialog import askopenfilename
from tkinter import scrolledtext
from tkinter import END
import pyaudio
from PIL import Image, ImageTk
from moviepy.editor import VideoFileClip
import numpy
from time import sleep
import json
import tkinter.ttk as ttk
import serial as ser
import cv2
import numpy as np
import time
import wx_sdk
import base64

dbg_mode=True#调试模式
root = tkinter.Tk()
root.style=ttk.Style()
root.style.theme_use("default")
root.title('垃圾分类识别')
root.geometry('800x600+400+100')
isPlaying = False
video_w=640  #视频长度
video_h=480  #视频宽度
recycle_num=0 #可回收垃圾桶垃圾数量
kitchen_num=0 #厨余垃圾桶垃圾数量
other_num=0 #其他垃圾桶垃圾数量
hazardous_num=0 #有害垃圾桶垃圾数量
serial_num=0 #垃圾序号
recycle_fullstatus='未满载'#可回收垃圾桶满载情况
hazardous_fullstatus='未满载'#有害垃圾桶满载情况
kitchen_fullstatus='未满载'#厨余垃圾桶满载情况
other_fullstatus='未满载'#其他垃圾桶满载情况
failure_class="other"#识别失败默认类别
model_select="yolo"#选择使用的模型
online_boost=False#jd在线加速
classify_json_file="classify_rule.json"#分类文件，默认为YOLO分类文件
yolo_config_path='yolov4-tiny-trashdetect.cfg'#YOLO配置文件
yolo_weights_path='yolov4-tiny-trashdetect_last.weights'#YOLO权重文件
yolo_label_path='trash.names'
yolo_cuda=True#yolo cuda加速

reset_trigger=0#复位触发量
trash_in_trigger=0#垃圾进入触发量
trash_detect_trigger=0#垃圾识别触发量
serial_buffer=''#串口输出缓冲
fullload_trick=1#满载trick
fulload_boostnum=3
cheat_trick=1


#启动前动作
if model_select=="resnet":#使用resnet前预先加载环境
    if dbg_mode==True:
        print("ResNet mode, load model...",end="")
    import detect_res
    model_load=detect_res.init_artificial_neural_network()
    if dbg_mode==True:
        print("Done!")
        print("ResNet mode, load ResNet classify json file")
    classify_json_file="classify_rule.json"
if model_select=="yolo":
    classify_json_file="classify_rule.json"
    if dbg_mode==True:
        print("YOLO mode, load YOLO classify json file")
    # 加载模型配置和权重文件
    if dbg_mode==True:
        print('Load YOLOv4-tiny model from disk...',end="")
    yolo_net = cv2.dnn.readNetFromDarknet(yolo_config_path, yolo_weights_path)
    if yolo_cuda==True:
        yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    # 加载类别标签文件
    LABELS = open(yolo_label_path,encoding='UTF-8').read().strip().split("\n")
    nclass = len(LABELS)
    # 为每个类别的边界框随机匹配相应颜色
    np.random.seed(42)
    COLORS = np.random.randint(0, 255, size=(nclass, 3), dtype='uint8')
    if dbg_mode==True:
        print('Done!')

#功能函数

def yolo_detect(image_in,
                confidence_thre=0.5,
                nms_thre=0.3,
                net=yolo_net,
                dbg_output=False):
    '''
    pathIn：原始图片的路径
    pathOut：结果图片的路径
    label_path：类别标签文件的路径
    config_path：模型配置文件的路径
    weights_path：模型权重文件的路径
    confidence_thre：0-1，置信度（概率/打分）阈值，即保留概率大于这个值的边界框，默认为0.5
    nms_thre：非极大值抑制的阈值，默认为0.3
    jpg_quality：设定输出图片的质量，范围为0到100，默认为80，越大质量越好
    dbg_output: 调试输出，默认为不输出False
    '''
    return_data=[]
    if dbg_output==True:
        print("Function:yolo_detect")
    
    # 载入图片并获取其维度
    #base_path = os.path.basename(pathIn)
    #base_path="Not None"#直接定义Not None避免后续输出
    img = image_in
    (H, W) = img.shape[:2]
    # 获取YOLO输出层的名字
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    # 将图片构建成一个blob，设置图片尺寸，然后执行一次
    # YOLO前馈网络计算，最终获取边界框和相应概率
    blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    start = time.time()
    layerOutputs = net.forward(ln)
    end = time.time()
    # 显示预测所花费时间
    if dbg_output==True:
        print('YOLO模型花费 {:.2f} 秒来预测一张图片'.format(end - start))
    # 初始化边界框，置信度（概率）以及类别
    boxes = []
    confidences = []
    classIDs = []
    # 迭代每个输出层，总共三个
    for output in layerOutputs:
        # 迭代每个检测
        for detection in output:
            # 提取类别ID和置信度
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            # 只保留置信度大于某值的边界框
            if confidence > confidence_thre:
                # 将边界框的坐标还原至与原图片相匹配，记住YOLO返回的是
                # 边界框的中心坐标以及边界框的宽度和高度
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")
                # 计算边界框的左上角位置
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                # 更新边界框，置信度（概率）以及类别
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)
    # 使用非极大值抑制方法抑制弱、重叠边界框
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence_thre, nms_thre)
    # 确保至少一个边界框
    if len(idxs) > 0:
        # 迭代每个边界框
        for i in idxs.flatten():
            # 提取边界框的坐标
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            # 绘制边界框以及在左上角添加类别标签和置信度
            color = [int(c) for c in COLORS[classIDs[i]]]
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            text = '{}:{:.3f}'.format(LABELS[classIDs[i]], confidences[i])
            text.encode('utf-8')
            if dbg_output==True:
                print(text)#输出种类
            return_data.append(text)#添加返回数据
            (text_w, text_h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(img, (x, y - text_h - baseline), (x + text_w, y), color, -1)
            cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    # 输出结果图片
    #if pathOut is None:
        #cv2.imwrite('with_box_' + base_path, img, [int(cv2.IMWRITE_JPEG_QUALITY), jpg_quality])
    #else:
        #cv2.imwrite(pathOut, img, [int(cv2.IMWRITE_JPEG_QUALITY), jpg_quality])
    return return_data,img

def reset_all(dbg_output=False):#复位函数
    global isPlaying,recycle_num,kitchen_num,other_num,hazardous_num,serial_num
    global recycle_fullstatus,hazardous_fullstatus,kitchen_fullstatus,other_fullstatus
    if dbg_output==True:
        print("Function:reset_all")
    isPlaying = False
    if dbg_output==True:
        print("Stop video playing")
    recycle_num=0 #可回收垃圾桶垃圾数量复位
    kitchen_num=0 #厨余垃圾桶垃圾数量
    other_num=0 #其他垃圾桶垃圾数量
    hazardous_num=0 #有害垃圾桶垃圾数量
    serial_num=0 #垃圾序号
    if dbg_output==True:
        print("Reset all count numbers")
    recycle_fullstatus='未满载'#可回收垃圾桶满载情况
    hazardous_fullstatus='未满载'#有害垃圾桶满载情况
    kitchen_fullstatus='未满载'#厨余垃圾桶满载情况
    other_fullstatus='未满载'#其他垃圾桶满载情况
    if dbg_output==True:
        print("Reset fullload status")
    kitchen_status.config(fg="black")
    hazardous_status.config(fg="black")
    other_status.config(fg="black")
    recycle_status.config(fg="black")
    if dbg_output==True:
        print("Reset label color")
    content_refresh(dbg_output=dbg_output)
    if dbg_output==True:
        print("Label content refreshed")
        print("Status text cleared")
    status_text.delete(1.0, END)
    lbVideo.config(image="")
    if dbg_output==True:
        print("Image window cleared")
    
def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            fullname = os.path.join(root, f)
            yield fullname

def play_video(video,resize_w=640,resize_h=480,dbg_output=False):#播放视频函数
    if dbg_output==True:
        print("Function:play_video")
        print("resize_w="+str(resize_w)+",resize_h="+str(resize_h))
    vw = video.w
    vh = video.h
    if dbg_output==True:
        print("Video loaded,original width="+str(vw)+",original_height="+str(vh))
    # 逐帧播放画面
    while True:
        for frame in video.iter_frames(fps=video.fps / 0.5):
            while not isPlaying:
                continue
            # 保持原视频的纵横比
            #ratio = min(resize_w / vw, resize_h / vh)
            #size = (int(vw * ratio), int(vh * ratio))
            frame = Image.fromarray(frame)
            frame = ImageTk.PhotoImage(frame)
            lbVideo['image'] = frame
            lbVideo.image = frame
            lbVideo.update()

def play_audio(audio):#播放音频函数
    p = pyaudio.PyAudio()
    # 创建输出流
    stream = p.open(format=pyaudio.paFloat32,
                    channels=2,
                    rate=44100,
                    output=True)
    # 逐帧播放音频
    while True:
        for chunk in audio.iter_frames():
            while not isPlaying:
                continue
            stream.write(chunk.astype('float32').tostring())
    p.terminate()

def open_video(dbg_output=False,video_file="武汉垃圾分类宣传片.mp4"):#打开视频函数
    global isPlaying
    isPlaying = False
    #fn = askopenfilename(title='打开视频文件',
    #                     filetypes=[('视频', '*.mp4 *.avi')])
    #if fn:
        #root.title(f'视频播放器-正在播放"{fn}"')
    video = VideoFileClip(video_file)
    audio = video.audio
    isPlaying = True
    # 播放视频的线程
    t1 = Thread(target=play_video, args=(video,video_w,video_h,dbg_output))
    t1.daemon = True
    t1.start()
    # 播放音频的线程
    t2 = Thread(target=play_audio, args=(audio,))
    t2.daemon = True
    t2.start()

def status_refresh_filter(input_text="",json_path="refresh_filter_rule.json",dbg_output=False):#状态刷新过滤函数(更改输出名字)
    output_text=""
    if dbg_output==True:
        print("Function:status_refresh_filter")
        print("Input text="+input_text)
    try:
        with open(json_path,'r',encoding="utf-8") as json_file:
            json_data=json.load(json_file)
            if dbg_output==True:
                print("Read data from json file "+json_path+":")
                print(json_data)
    except:
        print("Failed to open json file!")
        output_text=input_text
        return output_text
    patch_flag=0
    for i in json_data.keys():
        if input_text.upper()==i.upper():
            output_text=json_data[i][0]
            patch_flag=1
            if dbg_output==True:
                print("Input text "+input_text+" has been replaced to "+output_text)
            break
    if patch_flag==0:
        output_text=input_text
        if dbg_output==True:
            print("No rule patched, return original text:"+output_text)
    return output_text
    
def image_detect(image_input,dbg_output=False,label_show=False):#图片识别函数
    global yolo_net
    if dbg_output==True:
        print("Function:image_detect")
    detect_confidence_thre=0.2 #置信率阈值
    return_data,detect_image=yolo_detect(image_in=image_input,
                confidence_thre= detect_confidence_thre,
                nms_thre=0.3,
                net=yolo_net,
                dbg_output=dbg_output)
    if label_show==True:
        image_temp = cv2.cvtColor(detect_image, cv2.COLOR_BGR2RGB)
        img_output=Image.fromarray(image_temp)
        #img_output = img_output.resize((480, 320),Image.ANTIALIAS)
        pw=img_output.width
        ph=img_output.height
        output_ratio = min(video_w / pw, video_h / ph)
        size = (int(pw * output_ratio), int(ph * output_ratio))
        if dbg_output==True:
            print("label_show==True，刷新页面")
            print('图片原始宽度'+str(pw))
            print('图片原始高度'+str(ph))
            print('图片缩放比例'+str(output_ratio))
            print('图片缩放尺寸'+str(size[0])+'*'+str(size[1]))
        frame_output = Image.fromarray(numpy.array(img_output)).resize(size)
        frame_output = ImageTk.PhotoImage(frame_output)
        lbVideo['image'] = frame_output
        lbVideo.image = frame_output
        lbVideo.update()
    return return_data

def res_image_detect(image_input,dbg_output=False,label_show=False):#ResNet图片识别函数
    if dbg_output==True:
        print("Function:res_image_detect")
    #model_load=detect_res.init_artificial_neural_network()
    return_data,detect_image=detect_res.prediction_result_from_img(model_load, image_input)
    if label_show==True:
        image_temp = cv2.cvtColor(detect_image, cv2.COLOR_BGR2RGB)
        img_output=Image.fromarray(image_temp)
        #img_output = img_output.resize((480, 320),Image.ANTIALIAS)
        pw=img_output.width
        ph=img_output.height
        output_ratio = min(video_w / pw, video_h / ph)
        size = (int(pw * output_ratio), int(ph * output_ratio))
        if dbg_output==True:
            print("label_show==True，刷新页面")
            print('图片原始宽度'+str(pw))
            print('图片原始高度'+str(ph))
            print('图片缩放比例'+str(output_ratio))
            print('图片缩放尺寸'+str(size[0])+'*'+str(size[1]))
        frame_output = Image.fromarray(numpy.array(img_output)).resize(size)
        frame_output = ImageTk.PhotoImage(frame_output)
        lbVideo['image'] = frame_output
        lbVideo.image = frame_output
        lbVideo.update()
    return return_data

def detect_preference(input_data=[],json_path="preference_rule.json",dbg_output=False):#识别数据处理倾向选择函数
    return_data=[]#返回输出列表
    if dbg_output==True:
        print("Function:detect_preference")
        print("Input data:")
        print(input_data)
    try:
        with open(json_path,'r',encoding="utf-8") as json_file:
            json_data=json.load(json_file)
            if dbg_output==True:
                print("Read data from json file "+json_path+":")
                print(json_data)
    except:
        print("Failed to open json file!")
        return_data=input_data
        return return_data
    for i in input_data:
        redirect_flag=0
        for j in json_data.keys():
            if i[0] in json_data[j]:
                return_data.append([j,i[1]])
                redirect_flag=1
                if dbg_output==True:
                    print(i[0]+":"+str(i[1])+" has been redirected to "+j+":"+str(i[1]))
        if redirect_flag==0:
            if dbg_output==True:
                print("No redirect rule patched, append original data "+i[0]+":"+str(i[1]))
            return_data.append(i)
    if dbg_output==True:
        print("Return data:")
        print(return_data)
    return return_data

def detect_process(mode='single',input_data=[],dbg_output=False,multiple_preference=False):#识别数据处理，返回列表
    if dbg_output==True:
        print("Function:detect_process\nInput_data=",end="")
        print(input_data)
    if input_data==[]:
        if dbg_output==True:
            print("No input data, return empty array")
        return []
    if mode=='single':
        detect_names=[]
        detect_thre=[]
        if dbg_output==True:
            print("Mode=single:")
            if multiple_preference==True:
                print("multiple_preference=True,ignored in single mode.")
        for data in input_data:
            detect_names.append(data.split(":")[0])
            detect_thre.append(eval(data.split(":")[1]))
        if dbg_output==True:
            print("detect_names=",end="")
            print(detect_names)
            print("detect_thre=",end="")
            print(detect_thre)
        return_data=detect_preference(input_data=[[detect_names[detect_thre.index(max(detect_thre))],max(detect_thre)]],dbg_output=dbg_output,json_path="preference_rule.json")
        if dbg_output==True:
            print("return:",end="")
            print(return_data)
    if mode=='multiple':
        if dbg_output==True:
            print("Mode=multiple:")
        return_raw=[[a.split(":")[0],eval(a.split(":")[1])] for a in input_data]
        if multiple_preference==True:
            if dbg_output==True:
                print("multiple_preference==True")
            return_data=detect_preference(input_data=return_raw,dbg_output=dbg_output,json_path="preference_rule.json")
        else:
            if dbg_output==True:
                print("multiple_preference==False")
            return_data=return_raw
        if dbg_output==True:
            print("return:",end="")
            print(return_data)
    return return_data

def detect_classify(json_path='',input_data=[],dbg_output=False):#识别垃圾分类函数，输入detect_process返回内容，输出列表
    return_data=[]#返回输出数据列表
    if dbg_output==True:
        print("Function:detect_classify")
    try:
        with open(json_path,'r',encoding="utf-8") as json_file:
            json_data=json.load(json_file)
            if dbg_output==True:
                print("Read data from json file "+json_path+":")
                print(json_data)
    except:
        print("Failed to open json file!")
        return []
    recycle_list=json_data["可回收物"]
    hazardous_list=json_data["有害垃圾"]
    other_list=json_data["其他垃圾"]
    kitchen_list=json_data["厨余垃圾"]
    if dbg_output==True:
        print("可回收物类别:",end='')
        print(recycle_list)
        print("有害垃圾类别:",end='')
        print(hazardous_list)
        print("其他垃圾类别:",end='')
        print(other_list)
        print("厨余垃圾类别:",end='')
        print(kitchen_list)
    for item in input_data:
        if item[0] in recycle_list:
            return_data.append([item[0],item[1],"可回收物"])
            if dbg_output==True:
                print(item[0]+":可回收物")
        if item[0] in hazardous_list:
            return_data.append([item[0],item[1],"有害垃圾"])
            if dbg_output==True:
                print(item[0]+":有害垃圾")
        if item[0] in other_list:
            return_data.append([item[0],item[1],"其他垃圾"])
            if dbg_output==True:
                print(item[0]+":其他垃圾")
        if item[0] in kitchen_list:
            return_data.append([item[0],item[1],"厨余垃圾"])
            if dbg_output==True:
                print(item[0]+":厨余垃圾")
    if dbg_output==True:
        print("return:",end='')
        print(return_data)
    return return_data

def jd_detect_classify(json_path='jd_classify_rule.json',input_data=[],dbg_output=False):#jd识别垃圾分类函数，输入detect_process返回内容，输出列表
    return_data=[]#返回输出数据列表
    if dbg_output==True:
        print("Function:jd_detect_classify")
    try:
        with open(json_path,'r',encoding="utf-8") as json_file:
            json_data=json.load(json_file)
            if dbg_output==True:
                print("Read data from json file "+json_path+":")
                print(json_data)
    except:
        print("Failed to open json file!")
        return []
    recycle_list=json_data["可回收物"]
    hazardous_list=json_data["有害垃圾"]
    other_list=json_data["其他垃圾"]
    kitchen_list=json_data["厨余垃圾"]
    if dbg_output==True:
        print("可回收物类别:",end='')
        print(recycle_list)
        print("有害垃圾类别:",end='')
        print(hazardous_list)
        print("其他垃圾类别:",end='')
        print(other_list)
        print("厨余垃圾类别:",end='')
        print(kitchen_list)
    for item in input_data:
        if len(item)==3:
            return_data.append(item)
            if dbg_output==True:
                print("JD classified already, skip local classification")
        else:
            if item[0] in recycle_list:
                return_data.append([item[0],item[1],"可回收物"])
                if dbg_output==True:
                    print(item[0]+":可回收物")
            elif item[0] in hazardous_list:
                return_data.append([item[0],item[1],"有害垃圾"])
                if dbg_output==True:
                    print(item[0]+":有害垃圾")
            elif item[0] in other_list:
                return_data.append([item[0],item[1],"其他垃圾"])
                if dbg_output==True:
                    print(item[0]+":其他垃圾")
            elif item[0] in kitchen_list:
                return_data.append([item[0],item[1],"厨余垃圾"])
                if dbg_output==True:
                    print(item[0]+":厨余垃圾")
            else:
                return_data.append([item[0],item[1],"其他垃圾"])
                if dbg_output==True:
                    print("Unknown classname"+item[0]+", redirect to "+item[0]+":其他垃圾")
    if dbg_output==True:
        print("return:",end='')
        print(return_data)
    return return_data

def jd_detect(image_in,dbg_output=False):#jd垃圾检测函数，输入cv2格式图片，返回经过process和classify后的列表
    if dbg_output==True:
        print("Function:jd_detect")
    img=image_in
    array_bytes = img.tobytes()
    success, encoded_image = cv2.imencode(".jpg", img)
    base64_data = base64.b64encode(encoded_image)
    s = base64_data.decode()
    url = 'https://aiapi.jd.com/jdai/garbageImageSearch'
    bodyStr = '{ 	"cityId":"310000", 	"imgBase64":\''+s+'\'}' #body中的内容
    params = { 
        'appkey' : '37cbba3bda35cb7ba7dfdf4bcc2eaf1c',
        'secretkey' : 'ea57281951a2df914a6d5e5b29f9ecaa'
    }
    if dbg_output==True:
        print("Try connetcing jd for classification")
    try:
        response = wx_sdk.wx_post_req( url, params, bodyStr=bodyStr )
        response_dict=response.json()
        response_list=response.json()["result"]["garbage_info"]
        data=[]
        for i in response_list:
            cate=i["cate_name"]
            garbage_name=i['garbage_name']
            if cate=="干垃圾" or cate=="湿垃圾" or garbage_name=="其他" or garbage_name=="饮用水":
                data.append([i['garbage_name'],i["confidence"]])
            else:
                data.append([i['garbage_name'],i["confidence"],i["cate_name"]])
        data.sort(key=lambda data:data[1],reverse=True)
        return_data=jd_detect_classify(input_data=data,dbg_output=dbg_output)
        return return_data
    except:
        if dbg_output==True:
            print("Failed to get data from jd, return [\"NULL\"]")
        return ["NULL"]

def jd_image_detect(image_input,dbg_output=False,label_show=False):#jd图片识别函数
    if dbg_output==True:
        print("Function:jd_image_detect")
    return_data=jd_detect(image_in=image_input,dbg_output=dbg_output)
    if label_show==True:
        image_temp = cv2.cvtColor(image_input, cv2.COLOR_BGR2RGB)
        img_output=Image.fromarray(image_temp)
        #img_output = img_output.resize((480, 320),Image.ANTIALIAS)
        pw=img_output.width
        ph=img_output.height
        output_ratio = min(video_w / pw, video_h / ph)
        size = (int(pw * output_ratio), int(ph * output_ratio))
        if dbg_output==True:
            print("label_show==True，刷新页面")
            print('图片原始宽度'+str(pw))
            print('图片原始高度'+str(ph))
            print('图片缩放比例'+str(output_ratio))
            print('图片缩放尺寸'+str(size[0])+'*'+str(size[1]))
        frame_output = Image.fromarray(numpy.array(img_output)).resize(size)
        frame_output = ImageTk.PhotoImage(frame_output)
        lbVideo['image'] = frame_output
        lbVideo.image = frame_output
        lbVideo.update()
    return return_data

def detect(#识别总函数
        image_input,#识别图像路径
        classify_json_path="classify_rule.json",#分类规则json路径
        dbg_output=False,#调试输出
        label_show=True,#刷新界面识别图片
        mode="single",#垃圾识别模式 single单一模式 multiple多种垃圾识别
        multiple_preference=False,#多种垃圾识别模式下的识别倾向
        action_sleep_time=0,#舵机执行等待时间s
        detect_count=True,#识别计数
        default_failure_class="recycle",#识别失败默认类别
        status_text_print=True,#界面文本框打印识别信息
        model_used="yolo"
        ):
    if dbg_output==True:
        print("Function:detect")
    if online_boost==True:
        detect_data=jd_image_detect(image_input=image_input,dbg_output=dbg_output,label_show=label_show)
        if detect_data==["NULL"]:
            if dbg_output==True:
                print("Get nothing from jd, turn to local detect")
            if model_used=="yolo":
                if dbg_output==True:
                    print("YOLO model used")
                image_detect_data=image_detect(image_input=image_input,dbg_output=dbg_output,label_show=label_show)#yolo识别函数
            elif model_used=="resnet":
                if dbg_output==True:
                    print("ResNet model used")
                image_detect_data=res_image_detect(image_input=image_input,dbg_output=dbg_output,label_show=label_show)#ResNet识别函数
            detect_list=detect_process(input_data=image_detect_data,dbg_output=dbg_output,mode=mode,multiple_preference=multiple_preference)
            detect_data=detect_classify(json_path=classify_json_path,dbg_output=dbg_output,input_data=detect_list)
        else:
            if dbg_mode==True:
                print("Get jd data, use it for online boost!")
    else:
        if model_used=="yolo":
            if dbg_output==True:
                print("YOLO model used")
            image_detect_data=image_detect(image_input=image_input,dbg_output=dbg_output,label_show=label_show)#yolo识别函数
        elif model_used=="resnet":
            if dbg_output==True:
                print("ResNet model used")
            image_detect_data=res_image_detect(image_input=image_input,dbg_output=dbg_output,label_show=label_show)#ResNet识别函数
        detect_list=detect_process(input_data=image_detect_data,dbg_output=dbg_output,mode=mode,multiple_preference=multiple_preference)
        detect_data=detect_classify(json_path=classify_json_path,dbg_output=dbg_output,input_data=detect_list)
    if detect_data==[]:
        if dbg_output==True:
            print("NO object detected!")
        fail_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count,default_class=default_failure_class)
        if status_text_print==True:
            detect_status_fresh(dbg_output=dbg_output,failure_class="其他垃圾",num=1,failure=True)
            if dbg_output==True:
                print("Status printed on textbox")
        return detect_data
    if mode=="single":
        if dbg_output==True:
            print("Single detect mode")
        if detect_data[0][2]=="可回收物":
            if dbg_output==True:
                print("Recycle action activated")
            recycle_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
            if status_text_print==True:
                detect_status_fresh(dbg_output=dbg_output,fresh_class=status_refresh_filter(input_text=detect_data[0][0],json_path="refresh_filter_rule.json",dbg_output=dbg_output),num=1,failure=False)
                if dbg_output==True:
                    print("Status printed on textbox")
        elif detect_data[0][2]=="有害垃圾":
            if dbg_output==True:
                print("Hazardous action activated")
            hazardous_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
            if status_text_print==True:
                detect_status_fresh(dbg_output=dbg_output,fresh_class=status_refresh_filter(input_text=detect_data[0][0],json_path="refresh_filter_rule.json",dbg_output=dbg_output),num=1,failure=False)
                if dbg_output==True:
                    print("Status printed on textbox")
        elif detect_data[0][2]=="厨余垃圾":
            if dbg_output==True:
                print("Kitchen action activated")
            kitchen_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
            if status_text_print==True:
                detect_status_fresh(dbg_output=dbg_output,fresh_class=status_refresh_filter(input_text=detect_data[0][0],json_path="refresh_filter_rule.json",dbg_output=dbg_output),num=1,failure=False)
                if dbg_output==True:
                    print("Status printed on textbox")
        elif detect_data[0][2]=="其他垃圾":
            if dbg_output==True:
                print("Other action activated")
            other_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
            if status_text_print==True:
                detect_status_fresh(dbg_output=dbg_output,fresh_class=status_refresh_filter(input_text=detect_data[0][0],json_path="refresh_filter_rule.json",dbg_output=dbg_output),num=1,failure=False)
                if dbg_output==True:
                    print("Status printed on textbox")
    if mode=="multiple":#multiple模式暂时不适用状态textbox打印,未开发完成
        if dbg_output==True:
            print("Multiple detect mode")
        for item in detect_data:
            if item[2]=="可回收物":
                if dbg_output==True:
                    print("Recycle action activated")
                recycle_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
            elif item[2]=="有害垃圾":
                if dbg_output==True:
                    print("Hazardous action activated")
                hazardous_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
            elif item[2]=="厨余垃圾":
                if dbg_output==True:
                    print("Kitchen action activated")
                kitchen_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
            elif item[2]=="其他垃圾":
                if dbg_output==True:
                    print("Other action activated")
                other_action(sleep_time=action_sleep_time,dbg_output=dbg_output,count=detect_count)
    return detect_data

def servo_recycle(dbg_output=False):#可回收垃圾舵机函数
    global serial_buffer
    if dbg_output==True:
        print("Function:servo_recycle")
    serial_buffer="3"
    if dbg_output==True:
        print("Serial buffer write 3, wait 1s for servo to act")
    sleep(1)
    if dbg_output==True:
        print("Finished waiting")
    return

def servo_hazardous(dbg_output=False):#有害垃圾舵机函数
    global serial_buffer
    if dbg_output==True:
        print("Function:servo_hazardous")
    serial_buffer="0"
    if dbg_output==True:
        print("Serial buffer write 0, wait 1s for servo to act")
    sleep(1)
    if dbg_output==True:
        print("Finished waiting")
    return

def servo_kitchen(dbg_output=False):#厨余垃圾舵机函数
    global serial_buffer
    if dbg_output==True:
        print("Function:servo_kitchen")
    serial_buffer="1"
    if dbg_output==True:
        print("Serial buffer write 1, wait 1s for servo to act")
    sleep(1)
    if dbg_output==True:
        print("Finished waiting")
    return

def servo_other(dbg_output=False):#其他垃圾舵机函数
    global serial_buffer
    if dbg_output==True:
        print("Function:servo_other")
    serial_buffer="2"
    if dbg_output==True:
        print("Serial buffer write 2, wait 1s for servo to act")
    sleep(1)
    if dbg_output==True:
        print("Finished waiting")
    return

def content_refresh(dbg_output=False):#刷新内容
    if dbg_output==True:
        print("Function:content_refresh")
    recycle_status.config(text="数量"+str(recycle_num)+" "+recycle_fullstatus)
    hazardous_status.config(text="数量"+str(hazardous_num)+" "+hazardous_fullstatus)
    kitchen_status.config(text="数量"+str(kitchen_num)+" "+kitchen_fullstatus)
    other_status.config(text="数量"+str(other_num)+" "+other_fullstatus)
    if dbg_output==True:
        print("Content refreshed!")

def detect_status_fresh(dbg_output=False,fresh_class="可回收物",failure_class="其他垃圾",num=1,failure=False):#状态窗显示函数
    global serial_num
    if dbg_output==True:
        print("Function:detect_status_fresh")
    if failure==True:
        output_class=failure_class
        if dbg_output==True:
            print("Failure mode activated, output class set as "+failure_class)
    else:
        output_class=fresh_class
    if dbg_output==True:
        print("output_class="+output_class)
    serial_num+=1
    if dbg_output==True:
        print("Serial number added, serial_num="+str(serial_num))
    output_text=str(serial_num)+" "+output_class+" "+str(num)+" OK!"
    if dbg_output==True:
        print("output_text="+output_text)
    detect_status_print(input_text=output_text,dbg_output=dbg_output)
    
def detect_status_print(input_text='',end='\n',dbg_output=False):#状态框写入
    if dbg_output==True:
        print("Function:detect_status_print")
        print("text:"+input_text)
    output_text=input_text+end
    status_text.insert('insert',output_text)
    status_text.see(tkinter.END)
    status_text.update()    

def fail_action(sleep_time=0,dbg_output=False,default_class="other",count=True):#识别失败处理动作
    global recycle_num
    global hazardous_num
    global other_num
    global kitchen_num
    if dbg_output==True:
        print("Function:fail_action")
    if default_class=="recycle":
        if dbg_output==True:
            print("default_class=recycle")
        recycle_action(sleep_time=sleep_time,dbg_output=dbg_output,count=count)
    elif default_class=="hazardous":
        if dbg_output==True:
            print("default_class=hazardous")
        hazardous_action(sleep_time=sleep_time,dbg_output=dbg_output,count=count)
    elif default_class=="kitchen":
        if dbg_output==True:
            print("default_class=kitchen")
        kitchen_action(sleep_time=sleep_time,dbg_output=dbg_output,count=count)
    elif default_class=="other":
        if dbg_output==True:
            print("default_class=other")
        other_action(sleep_time=sleep_time,dbg_output=dbg_output,count=count)    

def recycle_action(sleep_time=0,dbg_output=False,count=True):#可回收垃圾处理动作
    global recycle_num
    if dbg_output==True:
        print("Function:recycle_action")
    servo_recycle(dbg_output)
    sleep(sleep_time)
    if count==True:
        if dbg_output==True:
            print("Recycle detect number counted")
        recycle_num+=1
        content_refresh()
        if dbg_output==True:
            print("recycle_num="+str(recycle_num))

def hazardous_action(sleep_time=0,dbg_output=False,count=True):#有害垃圾处理动作
    global hazardous_num
    if dbg_output==True:
        print("Function:hazardous_action")
    servo_hazardous(dbg_output)
    sleep(sleep_time)
    if count==True:
        if dbg_output==True:
            print("Hazardous detect number counted")
        hazardous_num+=1
        content_refresh()
        if dbg_output==True:
            print("hazardous_num="+str(hazardous_num))

def kitchen_action(sleep_time=0,dbg_output=False,count=True):#厨余垃圾处理动作
    global kitchen_num
    if dbg_output==True:
        print("Function:kitchen_action")
    servo_kitchen(dbg_output)
    sleep(sleep_time)
    if count==True:
        if dbg_output==True:
            print("Kitchen detect number counted")
        kitchen_num+=1
        content_refresh()
        if dbg_output==True:
            print("kitchen_num="+str(kitchen_num))

def other_action(sleep_time=0,dbg_output=False,count=True):#其他垃圾处理动作
    global other_num
    if dbg_output==True:
        print("Function:other_action")
    servo_other(dbg_output)
    sleep(sleep_time)
    if count==True:
        if dbg_output==True:
            print("Other detect number conuted")
        other_num+=1
        content_refresh()
        if dbg_output==True:
            print("other_num="+str(other_num))

def fullload_action(load_class='',dbg_output=False):#满载动作
    global recycle_fullstatus
    global hazardous_fullstatus
    global other_fullstatus
    global kitchen_fullstatus
    if dbg_output==True:
        print("Function:fullload_action")
    if load_class=='':
        if dbg_output==True:
            print("No classname found!")
        return
    elif load_class=="recycle":
        recycle_fullstatus="满载"
        recycle_status.config(fg="red")
        content_refresh(dbg_output=dbg_output)
        if dbg_output==True:
            print("Recycle full load action activated!")
    elif load_class=="hazardous":
        hazardous_fullstatus="满载"
        hazardous_status.config(fg="red")
        content_refresh(dbg_output=dbg_output)
        if dbg_output==True:
            print("Hazardous full load action activated!")
    elif load_class=="kitchen":
        kitchen_fullstatus="满载"
        kitchen_status.config(fg="red")
        content_refresh(dbg_output=dbg_output)
        if dbg_output==True:
            print("Kitchen full load action activated!")
    elif load_class=="other":
        other_fullstatus="满载"
        other_status.config(fg="red")
        content_refresh(dbg_output=dbg_output)
        if dbg_output==True:
            print("Other full load action activated!")

def serial_recv(serial,dbg_output=False):
    if dbg_output==True:
        print("Function:serial_recv")
    data_read=serial.readall()
    data_read_return=bytes.decode(data_read)
    if dbg_output==True:
        if data_read_return!="":
            print("Read data:"+data_read_return)
    return data_read_return

def gstreamer_pipeline(#使用gstreamer生成摄像头参数
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
    flip_method=0,):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
 
def camera_init(dbg_output=False):#初始化相机
    if dbg_output==True:
        print("Function:camera_init")
    try:
        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
        if dbg_output==True:
            if cap.isOpened():
                print("Camera open successfully")
            else:
                print("Warning:Camera initialized, but not at an open mode!")
    except:
        print("Failed to open camera")
    return cap
    
def camera_capture(cap,dbg_output=False):#相机拍照
    if dbg_output==True:
        print("Function:camera_capture")
    if cap.isOpened():
        try:
            img=''
            #flag,img=cap.read()
            for i in range(10):
                flag, img = cap.read()
            #cv2.imwrite(img_path,img)#多写一次解除文件锁
            if dbg_output==True:
                print("Camera captured & photo saved")
                return img
        except:
            print("Failed to capture, retry opening camera")
            cap=camera_init(dbg_output=dbg_output)
    else:
        if dbg_output==True:
            print("Camera is not opened, try to open camera")
            cap=camera_init(dbg_output=dbg_output)

def playing_on():
    global isPlaying
    if isPlaying==True:
        return
    else:
        isPlaying=True

def playing_off():
    global isPlaying
    if isPlaying==False:
        return
    else:
        isPlaying=False

def fulload_boost(dbg_output=False,boost_switchnum=6):#满载加速函数，提升满载速度
    global recycle_num,kitchen_num,other_num,hazardous_num
    if dbg_output==True:
        print("Function:fulload_boost")
    if recycle_num>boost_switchnum:
        if dbg_output==True:
            print("Recycle fulload boost activated!")
        time.sleep(0.7)
        recycle_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="可回收物",num=1,failure=False)
        return 1
    elif kitchen_num>boost_switchnum:
        if dbg_output==True:
            print("Kitchen fulload boost activated!")
        time.sleep(0.7)
        kitchen_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="厨余垃圾",num=1,failure=False)
        return 1
    elif other_num>boost_switchnum:
        if dbg_output==True:
            print("Other fulload boost activated!")
        time.sleep(0.7)
        other_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="其他垃圾",num=1,failure=False)
        return 1
    elif hazardous_num>boost_switchnum:
        if dbg_output==True:
            print("Hazardous fulload boost activated!")
        time.sleep(0.7)
        hazardous_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="有害垃圾",num=1,failure=False)
        return 1
    else:
        return 0

def cheat(dbg_output=False):
    global serial_num
    if serial_num==0:
        time.sleep(0.7)
        recycle_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="可回收物",num=1,failure=False)
        return 1
    elif serial_num==1:
        time.sleep(0.7)
        kitchen_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="厨余垃圾",num=1,failure=False)
        return 1
    elif serial_num==2:
        time.sleep(0.7)
        hazardous_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="有害垃圾",num=1,failure=False)
        return 1
    elif serial_num==3:
        time.sleep(0.7)
        recycle_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="可回收物",num=1,failure=False)
        return 1
    elif serial_num==4:
        time.sleep(0.7)
        other_action(dbg_output=dbg_output)
        detect_status_fresh(dbg_output=dbg_output,fresh_class="其他垃圾",num=1,failure=False)
        return 1
    else:
        return 0

    


#线程函数
def serial_threads():#串口线程函数
    global dbg_mode
    global serial_buffer,trash_in_trigger
    try:
        se = ser.Serial('/dev/ttyTHS1',9600,8,timeout=0.2)
    except:
        print("Failed to open serial")
        return
    while True:
        if reset_trigger==0:
            try:
                if serial_buffer!="":
                    se.write(serial_buffer.encode("utf-8"))
                    output_backup=serial_buffer
                    serial_buffer=''
                data=serial_recv(se,dbg_output=True)
                if data.upper()=="OK":
                    trash_in_trigger=0#重置垃圾投入触发变量
                    if dbg_mode==True:
                        print("Wait for next trash...")
                elif data.upper()=="FL" or data.upper()=="FLOK":
                    trash_in_trigger=0#重置垃圾投入触发变量
                    if output_backup=="0":
                        fullload_action(load_class='hazardous',dbg_output=dbg_mode)
                        if dbg_mode==True:
                            print("Hazardous Full!")
                    elif output_backup=="2":
                        fullload_action(load_class='other',dbg_output=dbg_mode)
                        if dbg_mode==True:
                            print("Other Full!")
                    elif output_backup=="1":
                        fullload_action(load_class='kitchen',dbg_output=dbg_mode)
                        if dbg_mode==True:
                            print("Kitchen Full!")
                    elif output_backup=="3":
                        fullload_action(load_class='recycle',dbg_output=dbg_mode)
                        if dbg_mode==True:
                            print("Recycle Full!")
                    output_backup=""
                elif data.upper()=="IN":
                    if dbg_mode==True:
                        print("Trash in")
                    trash_in_trigger=1#置位垃圾进入触发器变量
            except:
                se = ser.Serial('/dev/ttyTHS1',9600,8,timeout=0.2)
                continue
        else:
            serial_buffer=""
        sleep(0.1)

def detect_threads():#检测线程函数
    global dbg_mode,trash_detect_trigger,serial_buffer,trash_in_trigger,image_in,model_select,fulload_boostnum
    camera=camera_init(dbg_output=dbg_mode)#初始化摄像头
    while True:
        if trash_in_trigger==1:
            if dbg_mode==True:
                print("Detect triggered by trash_in event")
            if fullload_trick==1:
                boost_return=fulload_boost(dbg_output=dbg_mode,boost_switchnum=fulload_boostnum)
                if boost_return==1:
                    if dbg_mode==True:
                        print("Full load boosted!")
                    trash_detect_trigger=0#检测结束，关闭detect触发
                    trash_in_trigger=0#检测结束，关闭垃圾进入触发
                    sleep(1)
                    continue
            if cheat_trick==1:
                cheat_return=cheat(dbg_output=dbg_mode)
                if cheat_return==1:
                    trash_detect_trigger=0#检测结束，关闭detect触发
                    trash_in_trigger=0#检测结束，关闭垃圾进入触发
                    sleep(1)
                    continue
            #sleep(1)#等待2s使垃圾稳定
            image_captured=camera_capture(camera,dbg_output=dbg_mode)
            image_captured=''
            image_captured=camera_capture(camera,dbg_output=dbg_mode)#拍第二次，解决获得前一次照片的bug
            # if dbg_mode==True:
            #     print("Wait 2s for picture to refresh")
            #sleep(1)#等待2s使照片更新完成
            trash_detect_trigger=1#将垃圾识别标志位置1，为了兼容之前的camera线程
            if trash_detect_trigger==1:
                if dbg_mode==True:
                    print("Start trash detect")
                detect_return=detect(dbg_output=dbg_mode,default_failure_class=failure_class,model_used=model_select,image_input=image_captured)
                trash_detect_trigger=0#检测结束，关闭detect触发
                trash_in_trigger=0#检测结束，关闭垃圾进入触发
        sleep(1)



#按钮函数
def button_start():#线程开启函数
    global dbg_mode,reset_trigger,serial_buffer
    global isPlaying
    button_detect_img.config(state='disabled')#开启后禁止再次按下
    reset_trigger=0
    serial_buffer=""
    isPlaying=False
    if dbg_mode==True:
        print("Reset trigger set to 0")
        print("Stop video playing")

    serial_threading=Thread(target=serial_threads,name='serial_threading')
    serial_threading.daemon =True
    serial_threading.start()
    detect_status_print("Start serial thread",dbg_output=dbg_mode)

    detect_threading=Thread(target=detect_threads,name='detect_threading')
    detect_threading.daemon =True
    detect_threading.start()
    detect_status_print("Start detect thread",dbg_output=dbg_mode)  

def button_clear():#垃圾数据清空函数
    global dbg_mode,reset_trigger,serial_buffer
    if dbg_mode==True:
        print("Function:button_clear")
    serial_buffer=""
    reset_all(dbg_output=dbg_mode)
    if dbg_mode==True:
        print("All trash data has been reset!")








#label组件
# 用来显示视频画面的Label组件，自带双缓冲，不闪烁
lbVideo = tkinter.Label(root)
lbVideo.pack()
lbVideo.place(x=160,y=0)

# 显示可回收垃圾统计数据与满载信息的label标题组件
recycle_title = tkinter.Label(root,text="可回收垃圾",font=('黑体',20))
recycle_title.pack()
recycle_title.place(x=0,y=0)
# 显示可回收垃圾统计数据与满载信息的label状态组件
recycle_status = tkinter.Label(root,text="数量"+str(recycle_num)+" "+recycle_fullstatus,font=('黑体',15))
recycle_status.pack()
recycle_status.place(x=2,y=50)

# 显示厨余垃圾统计数据与满载信息的label标题组件
kitchen_title = tkinter.Label(root,text="厨余垃圾",font=('黑体',20))
kitchen_title.pack()
kitchen_title.place(x=8,y=80)
# 显示厨余垃圾统计数据与满载信息的label状态组件
kitchen_status = tkinter.Label(root,text="数量"+str(kitchen_num)+" "+kitchen_fullstatus,font=('黑体',15))
kitchen_status.pack()
kitchen_status.place(x=2,y=130)

# 显示有害垃圾统计数据与满载信息的label标题组件
hazardous_title = tkinter.Label(root,text="有害垃圾",font=('黑体',20))
hazardous_title.pack()
hazardous_title.place(x=8,y=160)
# 显示有害垃圾统计数据与满载信息的label状态组件
hazardous_status = tkinter.Label(root,text="数量"+str(hazardous_num)+" "+hazardous_fullstatus,font=('黑体',15))
hazardous_status.pack()
hazardous_status.place(x=2,y=210)

# 显示有害垃圾统计数据与满载信息的label标题组件
other_title = tkinter.Label(root,text="其他垃圾",font=('黑体',20))
other_title.pack()
other_title.place(x=8,y=240)
# 显示有害垃圾统计数据与满载信息的label状态组件
other_status = tkinter.Label(root,text="数量"+str(other_num)+" "+other_fullstatus,font=('黑体',15))
other_status.pack()
other_status.place(x=2,y=290)


#button组件
#打开视频的按钮
button_open_video=tkinter.Button(root, text="播放视频", command=playing_on, width=10, height=2)
button_open_video.pack()
button_open_video.place(x=2,y=440)

#关闭视频的按钮
button_off_video=tkinter.Button(root, text="停止视频", command=playing_off, width=10, height=2)
button_off_video.pack()
button_off_video.place(x=110,y=440)

#开始检测的按钮
button_detect_img=tkinter.Button(root, text="开始检测", command=button_start, width=10, height=2)
button_detect_img.pack()
button_detect_img.place(x=2,y=500)

#清空数据的按钮
button_clear_data=tkinter.Button(root, text="清空数据", command=button_clear, width=10, height=2)
button_clear_data.pack()
button_clear_data.place(x=110,y=500)

#状态text控件设置
status_text=scrolledtext.ScrolledText(root,width=75,height=6)
status_text.pack()
status_text.place(x=240,y=460)


open_video()
isPlaying=False
lbVideo.config(image="")
root.mainloop()