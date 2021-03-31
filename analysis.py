#!/usr/bin/env python
from __future__ import division,print_function
import cv2
import os
import numpy as np

import matplotlib
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.transforms import blended_transform_factory
from matplotlib.collections import PatchCollection

class Analysis():
    def __init__(self):
        self.eye_level = 50
        self.safe_boundary = 50
        roi_width = 160
        roi_height = 20
        roi_raw = np.array([[240,347],[400,347],[400,367],[240,367]])
        self.roi_vertical_list = [roi_raw]
        [left_x,upper_y],[right_x,_],[_,lower_y],_ = roi_raw
        while upper_y-roi_height>0:
            upper_y-=roi_height
            lower_y-=roi_height
            upper_left = [left_x,upper_y]
            upper_right = [right_x,upper_y]
            lower_right = [right_x,lower_y]
            lower_left = [left_x,lower_y]
            self.roi_vertical_list.append(np.array([upper_left,upper_right,lower_right,lower_left]))

        reduce_factor = 5  
        self.roi_reduce_list = [roi_raw]
        [left_x,upper_y],[right_x,_],[_,lower_y],_ = roi_raw
        while right_x - left_x >10 and upper_y-roi_height>0:
            upper_y-=roi_height
            lower_y-=roi_height
            left_x+=reduce_factor
            right_x-=reduce_factor
            upper_left = [left_x,upper_y]
            upper_right = [right_x,upper_y]
            lower_right = [right_x,lower_y]
            lower_left = [left_x,lower_y]
            self.roi_reduce_list.append(np.array([upper_left,upper_right,lower_right,lower_left])) 

        expand_factor = 5
        self.roi_expand_list = [roi_raw]
        [left_x,upper_y],[right_x,_],[_,lower_y],_ = roi_raw
        while left_x - expand_factor >0 and right_x + expand_factor < 640 \
                                                    and upper_y-roi_height>0:
            upper_y-=roi_height
            lower_y-=roi_height
            left_x-=expand_factor
            right_x+=expand_factor
            upper_left = [left_x,upper_y]
            upper_right = [right_x,upper_y]
            lower_right = [right_x,lower_y]
            lower_left = [left_x,lower_y]
            self.roi_expand_list.append(np.array([upper_left,upper_right,lower_right,lower_left]))

    def euclidean_distance(self,image_src,image_dst):
        h,w = image_src.shape[:2]
        sad = [0]*3
        for i in range(h):
            for j in range(w):
                sad[0]+=abs(int(image_src[i][j][0]) - int(image_dst[i][j][0]))
                sad[1]+=abs(int(image_src[i][j][1]) - int(image_dst[i][j][1]))
                sad[2]+=abs(int(image_src[i][j][2]) - int(image_dst[i][j][2]))
        return sad

    def run_compare(self,image,roi_list):
        fixSrc = True
        result_list = []
        # use fixed roi
        if fixSrc:
            [left_x,upper_y],[right_x,_],[_,lower_y],_ = roi_list[0]
            src = image[upper_y:lower_y+1,left_x:right_x+1,:]            

        for i in range(1,len(roi_list)):
            # segment current roi
            if not fixSrc:
                [left_x,upper_y],[right_x,_],[_,lower_y],_ = roi_list[i-1]
                src = image[upper_y:lower_y+1,left_x:right_x+1,:]
            # segment next roi
            [left_x_1,upper_y_1],[right_x_1,_],[_,lower_y_1],_ = roi_list[i]    
            dst = image[upper_y_1:lower_y_1+1,left_x_1:right_x_1+1,:]
            if src.shape != dst.shape:
                if src.shape[1]>dst.shape[1]:
                    dst = cv2.resize(dst,None,fx = src.shape[1]/dst.shape[1],fy=1)
                elif src.shape[1]<dst.shape[1]:
                    src = cv2.resize(src,None,fx=dst.shape[1]/src.shape[1],fy=1)
            result = self.euclidean_distance(src,dst)
            result_list.append(result)
            
        return result_list     
    
    def interpolation(self,data,value):
        res=[]
        x,y=data
        for idx,(i,j) in enumerate(zip(x,x[1:])):
            if i<=value<=j or j<=value<=i:
                res.append([idx,idx+1])
        hline=[]
        inter=lambda x1,x2,y1,y2,v:(v-x1)/(x2-x1)*(y2-y1)+y1
        if len(res)>0:
            for i in res:
                x1,x2 = x[i[0]],x[i[1]]
                y1,y2 = y[i[0]],y[i[1]]
                hline.append(inter(x1,x2,y1,y2,value))

            return hline
        else:
            return 0
            
    def max_travel_dist(self,hline,data_y):
        n = len(hline)
        cut_off = self.eye_level + self.safe_boundary
        if n == 0:
            return False
        for i,j in zip(hline,hline[1:]):
            pass
                
        
        for i in range(n-1,-1,-1):
            if hline[i]<cut_off:
                return cut_off
            if hline[i]>cut_off:
                return hline[i]
        else:
            return cut_off
        
        # assert False,"ERROR: "+ str(hline) 
        
    def make_analysis(self,image):
        raw_result=[]
        for i,r in enumerate([self.roi_vertical_list,self.roi_reduce_list,self.roi_expand_list]):
            raw_result.append(self.run_compare(image.copy(),r))
            
        mean_result = [map(np.mean,i) for i in raw_result]
        
        minimum_mean=[]


        for row in range(3):
            data_x=mean_result[row]
            mean=np.mean(data_x)
            minimum_mean.append(mean)

        median=np.argsort(minimum_mean)[len(minimum_mean)//2]
        data_x = mean_result[median]
        data_y=list(range(346,346-len(data_x)*20,-20))


        roi = [self.roi_vertical_list,self.roi_reduce_list,self.roi_expand_list][median]
        mean=minimum_mean[median]
        std=np.std(data_x)
        hline = self.interpolation([data_x,data_y],mean)
        ########################################################
        # find new mean if hline > 3
        ########################################################
        interval=(mean-min(data_x))//100
        counter = interval
        if len(hline)>3:
            while len(hline)>3:
                hline = self.interpolation([data_x,data_y],mean-counter)
                counter+=interval

            mean = mean-counter-interval
        ########################################################
        # combine line that is close to each other
        ########################################################
        if len(hline)==3:
            if abs(hline[0]-hline[1])<20:
                if abs(hline[1]-hline[2])<20:
                    hline = hline[0]
                else:
                    hline = [hline[0],hline[2]]
            elif abs(hline[1]-hline[2])<20:
                hline = [hline[0],hline[1]]
        elif len(hline)==2:
            if abs(hline[0]-hline[1])<20:
                hline=hline[0]


        ########################################################
        # Plot data
        ########################################################
        travel_dist = self.max_travel_dist(hline,data_y)
        info_image = self.draw(image,roi,hline,mean,std,data_x,data_y,"median",travel_dist)
        # cv2.imshow("test",self.draw(image,hline,mean,std,data_x,data_y,"median",travel_dist))
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return hline,info_image
    
    def draw(self,image,roi,hline,mean,std,data_x,data_y,title="default",travel_dist=None):
        # fig,(ix,ax) = plt.subplots(2,1,sharey=False,figsize=(6,7),gridspec_kw={'wspace':0, 'hspace':0}, squeeze=True)
        fig,(ix,ax) = plt.subplots(1,2,sharey=False,figsize=(12,3),gridspec_kw={'wspace':0, 'hspace':0}, squeeze=True)

        canvas = FigureCanvas(fig)
        ix.set_ylim(367,0)

        patches = []
        for i in range(len(roi)):
            polygon = Polygon(roi[i], True)
            patches.append(polygon)
        # print(help(PatchCollection))
        p = PatchCollection(patches,facecolor = "none", edgecolor = ((0,1,1,1)))
        
        ix.add_collection(p)

        ix.imshow(image)

        linewidth = 2
        ix.axhline(self.eye_level,linewidth=linewidth,color="red")
        ax.axhline(self.eye_level,linewidth=linewidth,color="red")
        for hl in hline:
            ix.axhline(hl,linestyle="--",linewidth=linewidth,color="blue")
            ax.axhline(hl,linestyle="--",linewidth=linewidth,color="blue")

        
        ax.axvline(mean,linestyle="--",linewidth=linewidth,color="yellow")
        ax.set_ylim(367,0)
        ax.set_title(title)        
        ax.plot(data_x,data_y,marker='x')

        # asp = -np.diff(ax.get_xlim())[0]/np.diff(ax.get_ylim())[0]
        # asp = np.diff(ax.get_xlim())[0]/np.diff(ix.get_ylim())[0]
        # ax.set_aspect(asp)
        # print(asp)

        tform = blended_transform_factory(ax.transData, ax.transAxes)
        lo, hi = ax.get_ylim()
        lox,hix = ax.get_xlim()

        loy_i,hiy_i = ix.get_ylim()
        lox_i,hix_i = ix.get_xlim()

        fontsize = 'large'
        label="eye_level"
        ax.annotate(label, xy=(lox, (hi+lo)/2+1), xycoords='data', transform=tform,
                    xytext=(lox, self.eye_level), textcoords='data', fontsize=fontsize,
                    ha='left', va='bottom', color='r')

        ix.annotate(label, xy=(lox_i, (hiy_i+loy_i)/2+1), xycoords='data', transform=tform,
                    xytext=(lox_i, self.eye_level), textcoords='data', fontsize=fontsize,
                    ha='left', va='bottom', color='r')

        for hl in hline:
            label="{:.2f}".format(hl)
            ax.annotate(label, xy=(lox, (hi+lo)/2+1), xycoords='data', transform=tform,
                        xytext=(lox,hl), textcoords='data', fontsize=fontsize,
                        ha='left', va='bottom', color='y')

            ix.annotate(label, xy=(lox_i, (hiy_i+loy_i)/2+1), xycoords='data', transform=tform,
                    xytext=(lox_i,hl), textcoords='data', fontsize=fontsize,
                    ha='left', va='bottom', color='red')

        label="mean:{:.2f}     std:{:.2f}".format(mean,std)
        ax.annotate(label, xy=(mean, (hi+lo)/2+1), xycoords='data', transform=tform,
                    xytext=(mean,367), textcoords='data', fontsize=fontsize,
                    ha='left', va='bottom', color='y')

        if travel_dist is not None:
            label="{:d}".format(int(travel_dist))
            ax.axhline(travel_dist,linestyle="--",linewidth=linewidth,color="green")
            ax.annotate(label, xy=(lox, (hi+lo)/2+1), xycoords='data', transform=tform,
                        xytext=(lox,travel_dist), textcoords='data', fontsize=fontsize,
                        ha='left', va='bottom', color='y')

            ix.axhline(travel_dist,linestyle="--",linewidth=linewidth,color="green")
            ix.annotate(label, xy=(lox_i, (hiy_i+loy_i)/2+1), xycoords='data', transform=tform,
                    xytext=(lox_i,travel_dist), textcoords='data', fontsize=fontsize,
                    ha='left', va='bottom', color='red') 



        width, height = fig.get_size_inches() * fig.get_dpi()
        canvas=FigureCanvas(fig)
        canvas.draw()
        image = np.fromstring(canvas.tostring_rgb(), dtype='uint8').reshape(int(height), int(width), 3)
        return image

if __name__=="__main__":
    d=Analysis()
    image=cv2.imread("image1-1.jpg")
    travel_dist,info_image = d.make_analysis(image)
    print(travel_dist)
    cv2.namedWindow("image")
    cv2.moveWindow("image",100,200)
    cv2.imshow("image",info_image)
    cv2.waitKey(0)
    # cv2.imwrite("image_segmentation.jpg",info_image)
    cv2.destroyAllWindows()
    


