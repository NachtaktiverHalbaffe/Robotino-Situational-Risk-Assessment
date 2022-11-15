#!/usr/bin/env python3
import os
import rospkg
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt



rospack = rospkg.RosPack()

text_path = os.path.join(rospack.get_path("digital_twin"),'22_06_t19_02.txt')
path1 = os.path.join(rospack.get_path("digital_twin"),'logs','testWithoutQtable.txt')#'deviation_check22.txt')#'logs','plot22_06_21_20_33_straight.txt')
path2 = os.path.join(rospack.get_path("digital_twin"),'logs','test_sat1_learning.txt')
pathw = os.path.join(rospack.get_path("digital_twin"),'logs','analysis','real_WITH_qtable.txt')
pathn = os.path.join(rospack.get_path("digital_twin"),'logs','report','17_07_extended_final_with1cm_offset','log_17_07_2021_2cm_learning_07_extend.txt')
path12 = os.path.join(rospack.get_path("digital_twin"),'logs','report','17_07_good','log_17_07_2021_2cm_learning_07.txt')

pathName = path12#pathn
v=[]
r=[]
s=[]
d=[]
p_s=[]
p_r=[]
p_s_r=[]
rev_p_s=[]
rev_p_r=[]
real = []
sim=[]

reward = []
odom=[]
with open(pathName) as fp:
    Lines = fp.readlines()
    count= 0
    cc_odom = 0
    cc = 0
    cc_reward = 0
    for line in Lines:
        
            #line = line.replace(" ","")
            
            if 'velTwist' in line:
                v.append(line)
            if 'sim_data' in line:
                dd = line.split(',') 
                # for time 
                # nn = int(''.join(filter(str.isdigit, x[4])))
                nn = int(''.join(filter(str.isdigit, dd[3])))
                s.append((nn,float(dd[1]),float(dd[2])))
                sim.append((float(dd[1]),float(dd[2])))
            if 'real_data' in line:
                dd = line.split(',') 
                # for time 
                nn = int(''.join(filter(str.isdigit, dd[3])))
                if nn < 0.5:
                    r.append((nn,float(dd[1]),float(dd[2])))
                real.append((float(dd[1]),float(dd[2])))
            if 'deviation' in line:
                line = line.replace(' ',',')
                dd = line.split('deviation')
                dd = dd[1].split('\n')
                dd = dd[0].split('rotationDiff')
                # for time
                #print(dd)
                #nn = int(''.join(filter(str.isdigit, dd[3])))
                if dd[0] and float(dd[0].split(',')[2])<1:
                    #print(dd[0])
                    d.append((count,float(dd[0].split(',')[2])))
                    count = count +1 
            if 'publish' in line:
                dd = line.split('publish velocity')
                dd = dd[1].split('\n')[0].split(' ')
                print(dd)
                p_r.append((cc,float(dd[3])))
                p_s.append((cc,float(dd[1])))


                p_s_r.append((float(dd[1]),float(dd[3])))
                cc +=1

            if 'totalReward' in line:
                dd = line.split('totalReward')
                
                dd = dd[1].split('\n')[0].split(' ')
                #print('reward',dd[2])
                reward.append((cc_reward,float(dd[2])))
                #p_s.append((cc,float(dd[1])))
                

                cc_reward +=1
            if 'odom' in line:
                dd = line.split(',')
                #dd = dd[1].split('\n')[0].split(' ')
                if round(float(dd[1]),5)> 1:
                    print('odom',dd[1])
                odom.append((cc_odom,float(dd[1])))
                #p_s.append((cc,float(dd[1])))
                

                cc_odom +=1

#print(p_s_r)

'''['velTwist', 0.0, -1.0, rospy.Time[714543000000]]
['sim_data', 3.7300734402470965, 2.235498894135037, -1.2561777588229963, genpy.Time[714557000000]]
['real_data', 1.9854677283379272, 0.7634290535717652, 2.6918642787974005, genpy.Time[714567000000]]
['sim_data', 3.730073312426959, 2.2354991385857748, -1.3061736541163294, genpy.Time[714607000000]]'''


print('444444444444444',os.path.isfile(text_path), 'lenS: ',len(s), 'lenR: ', len(r), 'lenC:', len(v))
import numpy as np
import matplotlib.pyplot as plt
plt.title(pathName[-50:])
#zip(*r)

plt.scatter(*zip(*odom),color='green',linewidths=1)
#plt.scatter(*zip(*reward),color='red',linewidths=1)
#plt.scatter(*zip(*s),linewidths=1)
#deviation
#plt.scatter(*zip(*d),linewidths=1)
plt.scatter(*zip(*p_r),color='red',linewidths=1)
plt.scatter(*zip(*p_s),color='blue',linewidths=1)

print('cc_odom,cc,cc_reward',cc_odom,cc,cc_reward)
#plt.scatter(*zip(*p_s_r),color='red',linewidths=0.1)


#plt.scatter(rev_p_r,rev_p_s,color='red',linewidths=0.1)

#plt.plot(rev_p_r,rev_p_s,  color='black')
#plt.plot(*rev_p_s, color='red')
#plt.scatter(*zip(*real),color='red',linewidths=1)
#plt.scatter(*zip(*sim),linewidths=1)
plt.xlabel("steps")
plt.ylabel("velocity")


plt.show()


#plt.plot(rev_p_r,rev_p_s,  color='black')
#plt.plot(*rev_p_s, color='red')
#plt.scatter(*zip(*real),color='red',linewidths=1)
#plt.scatter(*zip(*sim),linewidths=1)
#plt.xlabel("x-label, real")
#plt.ylabel("y-label, sim")


#plt.show()

