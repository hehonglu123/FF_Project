import dxfgrabber
import math, yaml
import matplotlib

from dxfwrite import DXFEngine as dxf

import matplotlib.pyplot as plt
import numpy as np
import copy

import time
import sys
import os
from PIL import Image, ImageDraw, ImageEnhance, ImageOps
import cv2


class dxf_robot_motion:
    def __init__(self):
        
        self.seam_point_dict={}
        self.plan_seam_dict={}
        self.speed=1.0
        self.display=True
        self.multi_pants_data={}
    

    def dxf_grabber_readfile(self,filename):
        #filename=data.data
        dxf=dxfgrabber.readfile(filename)
        layer_count = len(dxf.layers)
        self.seam_point_dict={}
        front_34=None
        back_34=None
        back_38=None
        front_38=None
        #print(layer_count)
        #print(len(dxf.blocks))
        for entity in dxf.entities:
            print(entity.dxftype)
        all_blocks= [entity for entity in dxf.entities if entity.dxftype == 'INSERT']
        all_splines=[entity for entity in dxf.entities if entity.dxftype == 'SPLINE']
        print(all_splines)
        if(len(all_blocks)!=0):
            test_block=dxf.blocks[all_blocks[0].name]
            all_polylines= [entity for entity in test_block if entity.dxftype == 'POLYLINE']
        else:
            all_polylines=[entity for entity in dxf.entities if entity.dxftype == 'POLYLINE']

        q=1
        
        all_polylines= [entity for entity in dxf.entities if entity.dxftype == 'POLYLINE']
        #print(len(all_polylines))
        x=0
        last_in=False
        removed=[]
        flag=False
        vertices=[]
        sine_vals=[]

        for i in range(len(all_polylines)):
            start=all_polylines[i].points[0]
            end=all_polylines[i].points[-1]

            print(start)
            print(end)
            
            for e in range(len(all_polylines)):
                if(i==e):
                    continue
                else:

                    if(all_polylines[e].points[0]==start and all_polylines[e].points[-1]==end):

                        if(len(all_polylines[i].points)==len(all_polylines[e].points) and not flag):
                            flag=True
                            removed.append(all_polylines[e])
                        if(len(all_polylines[i].points)>len(all_polylines[e].points)):
                            
                            #print(len(all_polylines[i].points))
                            #print(len(all_polylines[e].points))
                            removed.append(all_polylines[e])
                            
            #if(not matched2):
            #    removed.append(all_polylines[i])
        #print(vertices)
        outside=0
        for entity in all_polylines:
            if(len(entity.points)<3):
                print("Found Center Line")
                removed.append(entity)

        for entry in removed:
            
            all_polylines.remove(entry)
        #print(len(all_polylines))    
        for i in range(len(all_polylines)):
            for point in all_polylines[i].points:
                #vertices.append(point)
                vertices.append(point[0:2])
                
        #try:
        #    shape=matplotlib.path.Path(vertices,closed=True)       
        #except:
            #self.dxf_file_response_pub.publish(String("dxf file chosen does not contain a closed path, are you sure the file is valid?"))         
        colors=['b','g','r','c','m','y','k']
        seam_counter=0
        maxx=0
        minx=100
        maxy=0
        miny=100
        count=0
        remove_points=[]
        for entity in all_polylines:
            for i in range(len(entity.points)-1):
                for e in range(len(entity.points)-1):
                    if(i==e):
                        continue
                    else:
                        if(entity.points[i]==entity.points[e]):
                            if(i>e):
                                remove_points.append(entity.points[i-1])
                                #print(i)
                                #print(e)
                                count+=1
            for point in remove_points:
                entity.points.remove(point)
        print("Removing %i small lines"%count)
        
        
        

                
        for entity in all_polylines:
            color=colors[x]
            sine_vals=[]
            motion_points=[]
            for i in range(len(entity.points)-1):
                #print(entity.points[i][0])
                #print(entity.points[i][1])
                #print(len(entity.points))
                plt.plot([entity.points[i][0],entity.points[i+1][0]],[entity.points[i][1],entity.points[i+1][1]],color)
                #spamwriter.writerow("%f %f"%(entity.points[i][0],entity.points[i][1]))
                if(entity.points[i][0]>maxx):
                    maxx=entity.points[i][0]
                if(entity.points[i][0]<minx):
                    minx=entity.points[i][0]
                if(entity.points[i][1]>maxy):
                    maxy=entity.points[i][1]
                if(entity.points[i][1]<miny):
                    miny=entity.points[i][1]
            
        # plt.show()

        print("Max x=%f"%maxx)
        print("Min x=%f"%minx)
        print("Max y=%f"%maxy)
        print("Min y=%f"%minx)
        
        return(all_polylines,maxx,maxy)

def main():
    dxf_planner=dxf_robot_motion()
    fabric_name="PD19_016C-FR-LFT-LWR HICKEY V2 36"
    filename="fabric_dxf/"+fabric_name+".dxf"
    points,maxx,maxy=dxf_planner.dxf_grabber_readfile(filename)
    
    draw = dxf.drawing(name='test.dxf')
    tupled=[]
    scale=50
    draw.add_layer('LINES')
    for entity in points:
        for point in entity.points:
            # print(point)
            x=tuple([point[0]*scale,point[1]*scale])
            tupled.append(x)
        draw.add(dxf.polyline(entity.points, color=7, layer='LINES'))
    draw.save()
    intmaxx=math.ceil(maxx*scale)
    intmaxy=math.ceil(maxy*scale)
    

    img = Image.new('RGB', (intmaxx,intmaxy), color='black')
    draw = ImageDraw.Draw(img)
    draw.polygon(tupled,outline='white')



    enhancer = ImageEnhance.Sharpness(img)
    output=ImageOps.flip(enhancer.enhance(1))

    output = np.array(output) 
    output = output[:, :, ::-1].copy() 

    if maxy>maxx:

        output=cv2.rotate(output,cv2.cv2.ROTATE_90_CLOCKWISE)
        temp=maxx
        maxx=maxy
        maxy=temp

    cv2.imwrite('templates/'+fabric_name+'.jpg',output)

    with open('fabric.yaml') as file:
        fabric_yaml = yaml.load(file, Loader=yaml.FullLoader)

    with open('fabric.yaml','w') as file:
        fabric_yaml[fabric_name]=[25.4*maxx,25.4*maxy]
        yaml.dump(fabric_yaml,file)

if __name__ == "__main__":
	main()
