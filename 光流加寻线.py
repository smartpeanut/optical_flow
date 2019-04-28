import sensor, image, time, math,pyb
from pyb import Pin, Timer,UART

uart = pyb.UART(3,115200,timeout_char = 1000)#串口初始化

# Tracks a black line. Use [(128, 255)] for a tracking a white line.
GRAYSCALE_THRESHOLD = [(0, 50)]

#ROIS = [                      #[ROI, weight]越近，权重越大，在这里权值暂时不考虑
#        (7, 22,   50, 8, 0),#下面1
#        (7, 12,   50, 8, 0),#中间2
#        (7, 1,    50, 8, 0),#上面3
#        (0, 0,    8, 31, 0),#左边4
#        (56,0 ,   8, 31, 0) #右边5
#       ]

ROIS = [                            #[ROI, weight]越近，权重越大，在这里权值暂时不考虑
               (0, 55,   64, 8, 0), #下面1
               (0, 28,   64, 8, 0), #中间2
               (0, 0,    64, 8, 0), #上面3
               (0, 0,    8, 64, 0), #左边4
               (56,0 ,   8, 64, 0)  #右边5
       ]
weight_sum = 0
for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.


#----------------------------------------寻找直线变量----------------------------------------#
enable_lens_corr = False # turn on for straighter lines...
min_degree = 0
max_degree = 179
a1=0;
b1=0;
c1=0;
a2=0;
b2=0;
c2=0;
last_x=0;
last_y=0;
#---------------------------------------摄像头初始化-----------------------------------------#

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.B64X64)                  # 颜色追踪:160*120  光流检测:40*30
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)                         # 颜色追踪关闭自动增益
sensor.set_auto_whitebal(False)                     # 颜色追踪关闭白平衡
delta_x=0;
delta_y=0;
flag=0;
i=0;  #记录第几行数据
j=0;  #记录直线数量
led=pyb.LED(3)#必要的时候进行红外补光


#检测圆形中心点的坐标
center_x=0;
center_y=0;
center_update=1;#中心圆位置更新的标志
center_x_old=0;
center_y_old=0;
center_pos_old=0;

center_x_down=0;
center_y_down=0;

center_x_up=0;
center_y_up=0;

center_x_mid=0;
center_y_mid=0;

center_y_left=0;
center_x_left=0;

center_y_right=0;
center_x_right=0;

center_flag1=0;#上下
center_flag2=0;#左右
center_flag3=0;#通过roll来调整黑线的位置  通过yaw来调整机头方向  矩形1和2=0;
center_flag4=0;
center_flag5=0;

turn_flag=0;#转弯的标志

yaw_angle=0;
out_str1='';
clock = time.clock()
#帧率
old_img= sensor.snapshot().mean_pooled(4,4)        # 160x120 -> 40x30,前一张照片
#定义一个定时发送数据的函数
def tick(timer):#we will receive the timer object when being called
        global flag
        flag=1
tim = Timer(4,freq=20)            # create a timer object using timer 4 - trigger at 1Hz
tim.callback(tick)                # set the callback to our tick function
#--------------------------------------while循环开始-----------------------------------------#

while(True):
    #led.on()
    if(flag==1):
        img=sensor.snapshot()
        img_old=img.copy()
        img.lens_corr(1.5) # for 2.8mm lens...摄像头畸变纠正
        #--------------------------------------光流定点-----------------------------------------#
        old = sensor.alloc_extra_fb(16, 16, sensor.GRAYSCALE)
        old.replace(sensor.snapshot().mean_pooled(4,4))
        new_img = sensor.snapshot().mean_pooled(4,4)
        displacement = old.find_displacement(new_img)
        old_img.replace(new_img)
        delta_x0 = int(displacement.x_translation() * 5) / 5.0
        delta_y0 = int(displacement.y_translation() * 5) / 5.0
        delta_x = 10*delta_x0
        delta_y = 10*delta_y0
        #--------------------------------------检测直线交点的位置---------------------------------------#
        lines = img_old.find_lines(threshold=1000, theta_margin = 50, rho_margin = 50)
        for i in range(0,len(lines)-1):
            for j in range(i+1,len(lines)):
                l0x1 = lines[i].x1()
                l0x2 = lines[i].x2()
                l0y2 = lines[i].y2()
                l0y1 = lines[i].y1()
                if(l0x1 == l0x2):
                    l0x1 = l0x1 + 0.1
                a0 = (l0y2 - l0y1)/(l0x2 - l0x1)
                b0 = l0y1 - a0*l0x1
                l1x1 = lines[j].x1()
                l1y1 = lines[j].y1()
                l1x2 = lines[j].x2()
                l1y2 = lines[j].y2()
                if(l1x1 == l1x2):
                    l1x1 = l1x1 + 0.1
                a1 = (l1y2 - l1y1)/(l1x2 - l1x1)
                b1 = l1y1 - a1*l1x1
                if(a0==a1):
                    a0 = a0+ 0.1
                intersectionx = (b1-b0)/(a0-a1)

                intersectiony = a0*intersectionx + b0

                #if(a0*a1 > -1.5 and a0*a1 < 0.7 ):
                #img.draw_circle(int(intersectionx), int(intersectiony),10)
                if((intersectionx-last_x)>2 or (intersectionx-last_x)<-2):
                    last_x=intersectionx;
                if((intersectiony-last_y)>2 or (intersectiony-last_y)<-2):
                    last_y=intersectiony;
        #img.binary(GRAYSCALE_THRESHOLD,invert=1);
        #--------------------------------------寻找黑线和圆心的位置--------------------------------------#
        centroid_sum=0#暂时用不到这个变量
        #检测圆形位置
        for r in ROIS:
            i=i+1;
            blobs=img_old.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True,pixels_area=10) # r[0:4] is roi tuple.
            if blobs:#如果找到了颜色块
                # Find the blob with the most pixels.
                largest_blob = max(blobs, key=lambda b: b.pixels())
                if(i==1):#下面矩形
                    if(largest_blob[2]<=15 and largest_blob[2]>=2):#排除瑕疵点
                        if(largest_blob[3]>=2):
                            center_x_down=largest_blob.cx();
                            center_y_down=largest_blob.cy();
                            center_flag1=1;#下面的矩形找到的标志
                            img.draw_rectangle(largest_blob.rect())
                            img.draw_cross(largest_blob.cx(),largest_blob.cy(),2)

                elif(i==2):#中间矩形
                    if(largest_blob[2]<=15 and largest_blob[2]>=2):
                        if(largest_blob[3]>=2):
                            center_x_mid=largest_blob.cx();
                            center_y_mid=largest_blob.cy();
                            center_flag2=1;
                            img.draw_rectangle(largest_blob.rect())
                            img.draw_cross(largest_blob.cx(),largest_blob.cy(),2)
                elif(i==3):#上面的矩形
                    center_flag3=2;#有找到，但是不符合要求
                    if(largest_blob[2]<=10 and largest_blob[2]>=2):
                        if(largest_blob[3]>=2):
                            center_x_up=largest_blob.cx();
                            center_y_up=largest_blob.cy();
                            center_flag3=1;
                            img.draw_rectangle(largest_blob.rect())
                            img.draw_cross(largest_blob.cx(),largest_blob.cy(),2)
                elif(i==4):#左边的矩形找到了
                    if(largest_blob[3]<=15 and largest_blob[3]>=2):
                        if(largest_blob[2]>=2):
                            center_y_left=largest_blob.cy();
                            center_x_left=largest_blob.cx();
                            center_flag4=1;
                            img.draw_rectangle(largest_blob.rect())
                            img.draw_cross(largest_blob.cx(),largest_blob.cy(),2)
                elif(i==5):
                    if(largest_blob[3]<=15 and largest_blob[3]>=2):
                        if(largest_blob[2]>=2):
                            center_y_right=largest_blob.cy();
                            center_x_right=largest_blob.cx();
                            center_flag5=1;
                            img.draw_rectangle(largest_blob.rect())
                            img.draw_cross(largest_blob.cx(),largest_blob.cy(),2)
        for l in lines:#画出所有的直线
            img.draw_line(l.line())
        img.draw_cross(int(last_x), int(last_y),3,color=0)
        img.draw_circle(int(last_x), int(last_y),3,color=0)
        #print(last_x,last_y)
        #1开始计算中心圆x坐标
        if(center_flag1==1 and center_flag3!=1):
            center_x=center_x_down;
        elif(center_flag1==0 and center_flag3==1):
            center_x=center_x_up;
        elif(center_flag1==1 and center_flag3==1):
            center_x=int((center_x_up+center_x_down)/2)#这样计算即使机头偏离了十字中心，中心点位置也是准确地
        else:
            center_x=center_x_old;#如果没找到就返回上一次的值

        #2开始计算中心圆y坐标
        if(center_flag4==1   and center_flag5==0):
            center_y=center_y_left;
        elif(center_flag4==0 and center_flag5==1):
            center_y=center_y_right;
        elif(center_flag4==1 and center_flag5==1):
            center_y=int((center_y_left+center_y_right)/2)
        else:
            center_y=center_y_old;#如果没找到就返回上一次的值
        center_x_old=center_x;
        center_y_old=center_y;
        img.draw_circle(center_x,center_y,3);
        img.draw_cross(center_x,center_y,3);

        #3开始检测直角
        #if(center_flag3==0 and center_flag1==1 and center_flag4==1):
        if(center_flag3==0 and center_flag1==1 and center_flag4==1):
            turn_flag=1;#左转
        if(center_flag3==0 and center_flag1==1 and center_flag5==1):
            turn_flag=1;#右转
        if(center_flag1==0):
            center_x_down=32;
        if(center_flag2==0):
            center_x_mid=32;
        if(center_flag3!=1):
            center_x_up=32;
        center_pos=int(center_x_down+center_x_mid+center_x_up)/3

        if(center_flag1==0 and center_flag2==0 and center_flag3==0):
            center_pos=center_pos_old
        center_pos_old=center_pos
        #距离中心点位置不能超过+-5cm
        #标记出来圆形，便于查看中心坐标，用于进行定点起飞和降落

        #开始进行YAW纠正,三条线都在视野内开始进行偏航纠正
        if(center_flag1==1 and center_flag2==1 and center_flag3==1):

            yaw_angle =math.atan((center_pos-32)/32)
            yaw_angle =math.degrees(yaw_angle)

        center_flag1=0;#标志清零
        center_flag2=0;#标志清零
        center_flag3=0;
        center_flag4=0;
        center_flag5=0;
        i=0;

        #50ms发送一次数据到飞控
        if(yaw_angle<0):
            out_str1='-'
            out_str1+= '%.2d'% int(-yaw_angle)    #寻找黑线中心位置计算出偏转角度
        else:
            out_str1='+'
            out_str1+= '%.2d'% int(yaw_angle)     #寻找黑线中心位置计算出偏转角度
        if(delta_x<0):
            out_str1+='-'
            out_str1+='%.2d'%  int(-delta_x);       #光流数据
        else:
            out_str1+='+'
            out_str1+='%.2d'%  int(delta_x)         #寻找黑线中心位置计算出偏转角度
        if(delta_y<0):
            out_str1+='-'
            out_str1+= '%.2d'% int(-delta_y);       #光流数据
        else:
            out_str1+='+'
            out_str1+= '%.2d'% int(delta_y);        #光流数据

        out_str1+='%.2d'%      int(center_pos);
        out_str1+='%.2d'%      int(center_x);    #圆心的位置
        out_str1+='%.2d'%      int(center_y);
        out_str1+='%.2d'%      int(turn_flag);   #直角标志位
        out_str1+='%.2d'%      int(last_x);      #直角交点位置
        out_str1+='%.2d'%      int(last_y);
        uart.write('s'+out_str1+'#')
        print(out_str1)
        #像素位移之和清零
        turn_flag=0;
        yaw_angle=0;
        delta_x=0
        delta_y=0
        #print("%0.1f Xcm  %0.1f Ycm  %0.2fQ\t" %(delta_x,delta_y,response))
        #数组清零
        out_str1=''#清除之前的数据
        flag=0;
        #-----------------------------------串口打印数据-----------------------------------------#

