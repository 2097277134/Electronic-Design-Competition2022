import cv2
import numpy as np 
import ipywidgets
class ShapeDetector:
    def __init__(self):
        pass
 
    def detect(self, c):
        # 初始化图片名称与大概的形状
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
         # 如果形状有3个顶点，则它是三角形
        if len(approx) == 3:
            shape = "triangle"

        # 如果形状有4个顶点，则它是正方形或矩形
        elif len(approx) == 4:
            
            # 计算轮廓的边界框，并使用
            # 计算纵横比的边界框
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # 正方形的纵横比大约为
            # 等于1，否则，形状为矩形
            # shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
            shape = "square"

        # 否则，我们假设形状是圆形
        else:
            shape = "circle"

        # 返回形状的名称
        return shape
    
    # 找到最大轮廓
    def maxContour(self,cnts):
        area=[]
        if cnts :
            for k in range(len(cnts)):
                area.append(cv2.contourArea(cnts[k]))
            max_idx = np.argmax(np.array(area))
            return max_idx
    def openCamera(self,fps=30,width=120,heigh=80):
        #打开摄像头 
        cap=cv2.VideoCapture(0)
        if not cap.isOpened():
            cap=cv2.VideoCapture(1)
        if not cap.isOpened():
            raise IOError('Can not open video')
        # 设置摄像头参数
        if cap.isOpened():    
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT,heigh)   
        wid = ipywidgets.Image()
        display(wid)
        return cap,wid
    # 显示输出图像
    def showImage(self,mask,image,wid):
        newblurred = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # 拼接图像
        newimage = np.hstack((image,newblurred))
        wid.value = cv2.imencode('.jpg',newimage)[1].tobytes()
    