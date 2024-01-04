from turtle import color
import cv2
import numpy as np
import matplotlib.pyplot as plt
import skvideo.io
import skvideo.datasets
from skimage.color import rgb2gray
from skimage.measure import label, regionprops
from skimage.morphology import dilation
from tkinter import *
from collections import deque
from PIL import Image, ImageTk


class MotionDetector:
    def __init__(self, a, m_threshold, d_threshold, skip_frames, N,KF):
        self.a = a
        self.m_threshold = m_threshold
        self.d_threshold = d_threshold
        self.skip_frames = skip_frames
        self.N = N
        self.KF = KF 
        self.centers_history = {}
        self.trails = {}
        
    def detect(self, frames, i):
        if (i+ 1 > len(frames)):
            return []
        
        ppframe = rgb2gray(frames[i])
        pframe = rgb2gray(frames[i+self.skip_frames])
        cframe = rgb2gray(frames[i+2*self.skip_frames])
        
        diff1 = np.abs(cframe - pframe)
        diff2 = np.abs(pframe - ppframe)
        motion_frame = np.minimum(diff1, diff2)
        
        thresh_frame = motion_frame > 0.05
        dilated_frame = dilation(thresh_frame, np.ones((9, 9)))
        label_frame = label(dilated_frame)
        regions = regionprops(label_frame)
        
        cen=[]
        for region in regions:
            if len(cen) >= self.N:
                break
            minr, minc, maxr, maxc = region.bbox
            area = (maxr-minr)*(maxc-minc)
            
            if 100 <= area <= 1000:
                predict_kf = self.KF.predict()
                update_kf = self.KF.update([[(minr+maxr)/2], [(minc+maxc)/2]])
                
                if abs(update_kf[0]-predict_kf[0]) <= self.d_threshold and abs(update_kf[1]-predict_kf[1]) <= self.d_threshold:
                    if region.label in self.centers_history:
                        self.centers_history[region.label].append([(minc+maxc)//2, (minr+maxr)//2])
                    else:
                        self.centers_history[region.label] = [[(minc+maxc)//2, (minr+maxr)//2]]
                        self.trails[region.label] = deque() 
                        
                    self.trails[region.label].append((minc, minr, maxc, maxr))
                    
                    if len(self.trails[region.label]) > 1:
                        for j in range(1, len(self.trails[region.label])):
                            pt1 = tuple(self.trails[region.label][j-1][:2])
                            pt2 = tuple(self.trails[region.label][j][:2])
                            
                cv2.rectangle(frames[i], (minc, minr), (maxc, maxr), (102, 0, 102), 2)
                cen.append(np.array(region.bbox))
        
        return cen

class KalmanFilter(object):

    def __init__(self, x, E, D, B, H, R, u):
        self.x = x
        self.E = E
        self.D = D
        self.B = B
        self.H = H
        self.R = R
        self.u = u

    def predict(self):
        
        self.x = np.dot(self.D, self.x) + np.dot(self.B, self.u)
        self.E = np.dot(np.dot(self.D, self.E), self.D.T)
        return [int(self.x[0]), int(self.x[1])]

    def update(self, z):
        S = np.linalg.inv(np.dot(self.H, np.dot(self.E, self.H.T)) + self.R)
        K = np.dot(np.dot(self.E, self.H.T), S)
        self.x += np.dot(K, (z - np.dot(self.H, self.x)))
        self.E -=  K*self.H*self.E
        return [int(self.x[0]), int(self.x[1])]   
    
    
class VideoPlayer:
    def __init__(self, video_file):
        self.video_file = video_file
        self.frames = skvideo.io.vread(self.video_file)
        self.kf = KalmanFilter(x=np.matrix([[0], [0], [0], [0]]),
                  E=np.eye(4),
                  D=np.matrix([[1, 0, 0.1, 0],
                               [0, 1, 0, 0.1],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]]),
                  B=np.matrix([[0.005, 0],
                               [0, 0.005],
                               [0.1, 0],
                               [0, 0.1]]),
                  H=np.matrix([[1, 0, 0, 0],
                               [0, 1, 0, 0]]),
                  R=np.matrix([[0.1, 0],
                               [0, 0.1]]),
                  u=np.matrix([[0], [0]]))
        self.motion_detector = MotionDetector(1, 1, 100, 3, 25, self.kf)
        
        self.root = Tk()
        self.root.title("Video Player")
        
        self.pause = False
        self.skip = False
        self.current_frame = 0
        
        self.create_widgets()
        self.play_video()
        self.root.mainloop()
    
    def create_widgets(self):
        self.canvas = Canvas(self.root, width=self.frames.shape[2], height=self.frames.shape[1])
        self.canvas.pack()
            
        self.rewind_button = Button(self.root, text="Rewind", command=self.rewind_frame, font=("Helvetica", 14))
        self.rewind_button.pack(side=LEFT, padx=10)
        Label(self.root).pack(side=LEFT, expand=True)
        
        self.play_button = Button(self.root, text="Play", command=self.toggle_play, font=("Helvetica", 14), fg="green", bg='black')
        self.play_button.pack(side=LEFT, padx=5)
        
        self.pause_button = Button(self.root, text="Pause", command=self.toggle_pause, font=("Helvetica", 14), bg='black', fg="red")
        self.pause_button.pack(side=LEFT, padx=5)
        
        Label(self.root).pack(side=LEFT, expand=True) 
        self.skip_button = Button(self.root, text="Skip", command=self.skip_frame, font=("Helvetica", 14))
        self.skip_button.pack(side=RIGHT, padx=10)
        

      
    def set_frame(self, frame):
        self.current_frame = int(frame)
    
    def play_video(self):
        if not self.pause:
            self.motion_detector.detect(self.frames, self.current_frame)
            frame = cv2.cvtColor(self.frames[self.current_frame], cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            photo = ImageTk.PhotoImage(img)
            self.canvas.create_image(0, 0, anchor=NW, image=photo)
            self.canvas.image = photo
            # self.slider.set(self.current_frame)
            self.current_frame += 1
            if self.current_frame >= len(self.frames) or self.skip:
                self.skip = False
                self.current_frame = 0
            self.root.after(30, self.play_video)
        else:
            self.root.after(30, self.play_video)
    
    def toggle_play(self):
        self.pause = False
    
    def toggle_pause(self):
        self.pause = True
    
    def skip_frame(self):
        self.current_frame += 60
        if self.current_frame >= len(self.frames):
            self.current_frame = len(self.frames) - 1
            
    def rewind_frame(self):
        self.current_frame -= 60
        if self.current_frame < 0:
            self.current_frame = 0

def main():
    filename = input("Enter the video file name: ")
    player = VideoPlayer(filename)


if __name__ == "__main__":
    main()
    
    
