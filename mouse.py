import cv2
import numpy as np
import win32api
from ultralytics import YOLO
from PIL import Image
from mss import mss
from colorama import Fore
from pyautogui import size
import torch
from termcolor import colored
import sys
import time

class Aimbot:
    def __init__(self,time_delay=0.5,mouse_value=0,toggle_value=0,) -> None:
        global model
        model = YOLO("model/apex.pt") #modelの読み込み(yolov8)

    def target_lock(self,target_X,target_Y):
            win32api.mouse_event(0x01,int(target_X),int(target_Y))
            time.sleep(float(0.01))

    def enemy_capture(self,object_x1,object_y1,object_x2,object_y2):
        global img_bgr
        global conf_box
        
        cv2.putText(img_bgr,'Capture',(0,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0))
        cv2.rectangle(img_bgr,(object_x1,object_y1),(object_x2,object_y2),(0,0,255))
        if conf_box is not None:
            cv2.putText(img_bgr,str(round(conf_box,3)),(object_x1,object_y1),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,0))
        cv2.imshow("test",img_bgr)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    

    def print(self,condition_x1,condition_y1,condition_x2,condition_y2):
        print(Fore.BLACK+f'{condition_x1},{condition_y1},{condition_x2},{condition_y2}')

    def main(self):
        global model
        global img_bgr
        global conf_box
        conf_box = 0 

        screen_width,screen_height  = size()
        region_top = (screen_height - 400) // 2
        region_left = (screen_width - 400) // 2
        mon = {'top': region_top, 'left': region_left, 'width': 400, 'height': 400}
        sct = mss()
    
        while True:
            sct_img = sct.grab(mon)
            img = Image.frombytes('RGB', (sct_img.size.width, sct_img.size.height), sct_img.rgb)
            img_bgr = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

            results = model(img_bgr, verbose=False)
            conf = results[0].boxes.conf.cpu().numpy()
            boxes = results[0].boxes.xyxy.cpu().numpy()[:, :4]

            if boxes is not None and len(boxes) > 0:
                for box, conf_box in zip(boxes, conf):
                    if conf_box > 0.65:
                        x1, y1, x2, y2 = box.astype(int)
                        object_center_x = int((x1 + x2) / 2) + region_left
                        object_center_y = int((y1 + y2) / 2) + region_top
                        dx = object_center_x - screen_width // 2
                        dy = object_center_y - screen_height // 2 
                        self.target_lock(dx,dy)
                        self.enemy_capture(x1,y1,x2,y2)
                        self.print(x1,y1,x2,y2)
            elif boxes is not None and len(boxes) == 0: 
                self.enemy_capture(0,0,0,0)

if __name__ == "__main__":
    t1 = Aimbot()
    t1.main()

