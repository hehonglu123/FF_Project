import dxfgrabber
import math
import matplotlib

from dxfwrite import DXFEngine as dxf

import matplotlib.pyplot as plt
import numpy as np
import csv
import io

import copy

import time
import csv
import sys
import os
from PIL import Image, ImageDraw, ImageEnhance
#from robodk import *
#sys.path.append(os.path.abspath("/home/ubuntu/robodk_postprocessors")) # temporarily add the path to POSTS folder




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
        
        #for q in range(len(all_blocks)):
        #test_block=dxf.blocks[all_blocks[q].name]
        #all_polylines= [entity for entity in test_block if entity.dxftype == 'POLYLINE']
        q=1
        
        all_polylines= [entity for entity in dxf.entities if entity.dxftype == 'POLYLINE']
        #print(len(all_polylines))
        x=0
        last_in=False
        removed=[]
        flag=False
        vertices=[]
        sine_vals=[]
        #with open('eggs.csv', 'w') as csvfile:
        #    spamwriter = csv.writer(csvfile, delimiter=' ',
        #                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(all_polylines)):
            start=all_polylines[i].points[0]
            end=all_polylines[i].points[-1]
            #matchedone=False
            #matched2=False
            print(start)
            print(end)
            
            for e in range(len(all_polylines)):
                if(i==e):
                    continue
                else:
                    '''
                    if(all_polylines[e].points[0]==start or all_polylines[e].points[0]==end):
                        matchedone=True
                        if(matchedone):
                            matched2=True
                    if(all_polylines[e].points[-1]==end or all_polylines[e].points[-1]==end):
                        matchedone=True
                        if(matchedone):
                            matched2=True
                    '''   
                    if(all_polylines[e].points[0]==start and all_polylines[e].points[-1]==end):
                        #print("identical start found")
                        #print(start)
                        #print(i)
                        #print(e)
                    #if(all_polylines[e].points[-1]==end):
                        #print("identical end found")
                        #print(end)
                        #print(i)
                        #print(e)
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
                
                
                #bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)
                
                #m=(entity.points[i+1][1]-entity.points[i][1])/(entity.points[i+1][0]-entity.points[i][0])
                #print(entity.points[i+1][0]-entity.points[i][0])
                
                #print(magnitude/bestfitlineslope)
                
                #bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)
                #print("best fit line:")
                #print(bestfitlineslope)
                #print(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))]))
                #plt.arrow(entity.points[i][0],entity.points[i][1],bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1)))
                #plt.plot(entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1)),'bo')
                #vector1=np.array([bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1))])
            
        plt.savefig('hello.png')    
        plt.show()
        #plt.title("test")
        #buf = io.BytesIO()
        #plt.savefig(buf, format='png')
        #buf.seek(0)
        
        #im = Image.open(buf)
        #im.show()
        #buf.close()
        print("Max x=%f"%maxx)
        print("Min x=%f"%minx)
        print("Max y=%f"%maxy)
        print("Min y=%f"%minx)
        
        return(all_polylines,maxx,maxy)

def main():
    #rospy.init_node('dxf_parser')
    dxf_planner=dxf_robot_motion()
    filename="fabric_dxf/PD19_016C-FR-LFT-UP HICKEY V2 36.dxf"
    #filename="38rear-R12.dxf"
    points,maxx,maxy=dxf_planner.dxf_grabber_readfile(filename)
    
    draw = dxf.drawing(name='test.dxf')
    tupled=[]
    draw.add_layer('LINES')
    for entity in points:
        for point in entity.points:
            print(point)
            x=tuple([point[0]*50,point[1]*50])
            tupled.append(x)
        draw.add(dxf.polyline(entity.points, color=7, layer='LINES'))
    draw.save()
    intmaxx=math.ceil(maxx)*50
    intmaxy=math.ceil(maxy)*50
    img = Image.new('RGB', (intmaxx,intmaxy), color='white')
    draw = ImageDraw.Draw(img)
    draw.polygon(tupled,outline='black')
    enhancer = ImageEnhance.Sharpness(img)
    output=enhancer.enhance(1)
    output.save('output.jpg')

if __name__ == "__main__":
	main()
