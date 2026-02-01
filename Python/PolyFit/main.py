from my_package import polyfit, Data_Loader_CSV, Print_Poly_eq,Poly_eq_test
from my_package import LQR_calc,forwardKinematics,LQR_Poly_eq
from my_package import Generate_content,Write_cpp_content
import numpy as np

L1 = 282/1000#ED
L2 = 285.6/1000#AD
L3 = 292.81/1000#BC
L4 = 130.67/1000#AB
L23 = 57.02/1000#DC
Wheel_R = 60/1000
PI = np.pi   
M =10.113 - 0.173520730578001 * 2#13.9 - 0.173520730578001 * 2 #10.113 - 0.173520730578001 * 2
m = 0.173520730578001 * 2
I =1.154577949- 0.000468745184860407 * 2#0.870168809 - 0.000468745184860407 * 2
i = 0.000468745184860407 * 2
g = 9.8

Q = np.array([40, 20, 40, 20.0])#np.array([1.0,0.1,5,0.2]) #q qd x xd
R = 40#0.01

input_angle_list = []
angle_to_leg_List = []
for num in range(0,100):
    input_angle_list.append(num*(-1/100.0))
for num in range(0,100):
    angle_to_leg_List.append(forwardKinematics(L1,L2,L3,L4,L23,input_angle_list[num])['y'])
print("-------------------------------------------------")
z1 = polyfit(angle_to_leg_List,input_angle_list,3)
Print_Poly_eq(z1)
Poly_eq_test(z1,-0.42710447)
#print(forwardKinematics(L1,L2,L3,L4,L23,-0.6021940975614611)['y'])

alpha = 0.8235338295316227
FN = (M+m)*g/2.0
angle_data = forwardKinematics(L1, L2, L3, L4, L23, -alpha)
#print(alpha)
#print(angle_data['x'])
print(angle_data['y'])
#print(angle_data['belta'])
K = (-L2*np.sin(alpha) + L23*np.sin(angle_data['belta']) - np.sqrt(2)/2*L4)/(L2*np.cos(alpha)+L23*np.cos(angle_data['belta']) - np.sqrt(2)/2*L4)
x0 = angle_data['x'] 
y0 = K*x0 +np.sqrt(2)/2*L4*(1-K) #->得到(x0,y0)
K1 = (y0+L2*np.sin(alpha)) /(x0-L2*np.cos(alpha))
costheta1 = np.abs(1+(K1-K1*x0+y0)*(K+np.sqrt(2)/2*L4*(1-K)))/(np.sqrt(1+(K1-K1*x0+y0)*(K1-K1*x0+y0))*np.sqrt(1+(K+np.sqrt(2)/2*L4*(1-K))*(K+np.sqrt(2)/2*L4*(1-K))))
costheta2 = np.abs(L2*np.cos(alpha)+K1-K1*x0+y0)/(np.sqrt(1+L2*np.cos(alpha)*L2*np.cos(alpha))*np.sqrt(1+(K1-K1*x0+y0)*(K1-K1*x0+y0)))
theta1 = np.acos(costheta1)
theta2 = np.acos(costheta2)
d = np.abs(-K1*x0 + y0)/np.sqrt(K1*K1 + 1)
F_joint = (FN*np.sin(theta2))/(np.sin(theta1))
T = F_joint*d
print(T)
print("-------------------------------------------------")

Power1 = 3 #拟合次方
Interval = 0.001 #步长
print("总共采样点数量:",end="")
print(0.5/Interval)
print("每隔",end=" ")
print(0.05/Interval,end="")
print("个点拟合一次")

#测试
h = 0.3

#生成文件属性
Header_name = "LegH_KPoly.h"
Source_name = "LegH_KPoly.cpp"
Namespace  = "Poly_List"
Class_name = "LegH_KPoly"
Header_save_path = "../RC2026_H7/Python_Generation/Inc"
Source_save_path = "../RC2026_H7/Python_Generation/Src"

#用于导入CSV文件进行数据拟合，VOFA+可以将数据保存至CSV格式，这里x是CSV第一列数据，y是第二列数据
# x = []
# y = []
# Data_Loader_CSV(x,y,'../Data_Collect/test.csv')
# Power = 2
# z1 = polyfit(x, y, Power)
# Print_Poly_eq(z1) #打印出拟合后的方程

#腿长范围0.0~0.5,分成十段拟合
x1 = [[] for _ in range(10)]
y1 = [[] for _ in range(10)]
K1 =[]
K2 =[]
K3 = []
K4 = []
K1_ = [[] for _ in range(10)]
K2_ = [[] for _ in range(10)]
K3_ = [[] for _ in range(10)]
K4_ = [[] for _ in range(10)]
#0.01 ~ 0.5

for num in range(0,(int)(0.5/Interval)):
    x1[(int)(num/(0.05/Interval))].append((num+1)*Interval)
    y1[(int)(num/(0.05/Interval))].append(LQR_calc(M,m,I,i,(num+1)*Interval,Wheel_R,Q,R,g))
for num in range(0,10):
    K1.append([item[0] for item in y1[num]])
    K2.append([item[1] for item in y1[num]])
    K3.append([item[2] for item in y1[num]])
    K4.append([item[3] for item in y1[num]])

for num in range(0,10):
    K1_[num].append(polyfit(x1[num],K1[num],Power1))
    K2_[num].append(polyfit(x1[num],K2[num],Power1))
    K3_[num].append(polyfit(x1[num],K3[num],Power1))
    K4_[num].append(polyfit(x1[num],K4[num],Power1))
K1_out = []
K2_out = []
K3_out = []
K4_out = []
func = []
func2 = []

#测试打印
# print("K1:",end="")
# Print_Poly_eq(K1_[(int)(h/0.05)][0])
# Poly_eq_test(K1_[(int)(h/0.05)][0],h)
# print("K2:",end="")
# Print_Poly_eq(K2_[(int)(h/0.05)][0])
# Poly_eq_test(K2_[(int)(h/0.05)][0],h)
# print("K3:",end="")
# Print_Poly_eq(K3_[(int)(h/0.05)][0])
# Poly_eq_test(K3_[(int)(h/0.05)][0],h)
# print("K4:",end="")
# Print_Poly_eq(K4_[(int)(h/0.05)][0])
# Poly_eq_test(K4_[(int)(h/0.05)][0],h)
# print("实际K:",end="")
# print(LQR_calc(M,m,I,i,h,Wheel_R,Q,R,g))

for num in range(0,10):
    K1_out.append(LQR_Poly_eq(K1_[num][0],"Leg_H"))
    K2_out.append(LQR_Poly_eq(K2_[num][0],"Leg_H"))
    K3_out.append(LQR_Poly_eq(K3_[num][0],"Leg_H"))
    K4_out.append(LQR_Poly_eq(K4_[num][0],"Leg_H"))
    func2.insert(num,(num,f"Poly_{num}"))
    func.insert(num,(f"Poly_{num}",K1_out[num],K2_out[num],K3_out[num],K4_out[num]))

#生成C++,H文件
head_content,source_content,Header_File_name = Write_cpp_content(Header_name,Namespace,Class_name,func2,func)

Generate_content(head_content,source_content,Header_File_name,Source_name,Header_save_path,Source_save_path)



