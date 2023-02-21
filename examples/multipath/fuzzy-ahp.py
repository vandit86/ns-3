#!/usr/bin/python3

import sys

# Required Libraries
import numpy as np

# Fuzzy AHP 
from pyDecision.algorithm import fuzzy_ahp_method

# plot data 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting
from matplotlib import cm
from matplotlib.ticker import LinearLocator

# Dataset
# dataset = list([
#     [ (  1,   1,   1), (  4,   5,   6), (  3,   4,   5), (  6,   7,   8) ],   #g1
#     [ (1/6, 1/5, 1/4), (  1,   1,   1), (1/3, 1/2, 1/1), (  2,   3,   4) ],   #g2
#     [ (1/5, 1/4, 1/3), (  1,   2,   3), (  1,   1,   1), (  2,   3,   4) ],   #g3
#     [ (1/8, 1/7, 1/6), (1/4, 1/3, 1/2), (1/4, 1/3, 1/2), (  1,   1,   1) ]    #g4
#     ])


# WEB 
# RSS [dBm]      Delay [ms]   Jitter [ms]   Loss Rate [%]    Cost [up to 50]    dweel time [s] 
#      0.243          0.073         0.032          0.375             0.160             0.117   
# rc: 0.16 
# metrics_wlan = list ([-75,    70,   15,   (30)  ,  5,   (60)  ]);  PLR vs Dwel time image 
# metrics_wlan = list ([(-75),    70,   15,   (30)  ,  5,   60  ]);  PLR vs RSS       image

dataset_web = list([
    #   RSS                Delay           Jitter              Loss Rate          Cost               dweel time
    [ (  1,  1,  3 )     , (  3, 5, 7 ),    (  5,  7,  9 ),   ( 0.2, 0.33, 1 ),   ( 1, 1, 3),         ( 1 ,3 ,5) ],         #g1  RSS
    [ ( 0.14, 0.2, 0.33 ), (  1, 1, 3 ),    (  1, 3, 5  ),    ( 0.17, 0.25, 0.5), ( 0.2, 0.33, 1),    ( 0.2, 0.33, 1) ],    #g3  Delay
    [ ( 0.11, 0.14, 0.2 ), ( 0.2, 0.33, 1), (  1,  1,  3 ),   ( 0.11, 0.14, 0.2), ( 0.14, 0.2, 0.33), ( 1/7, 1/5, 1/3) ],   #g4  Jitter  
    [ ( 1, 3, 5    ),      ( 2, 4, 6 ),     (  5,  7,  9 ),   ( 1, 1, 3),     ( 1,   3,   5),     ( 3, 5, 7) ],         #g5  Loss Rate
    [ (1, 1, 0.33),        ( 1, 3, 5),      (  3,  5,  7 ),   ( 0.2, 0.33, 1),    ( 1,   1,   3),     (  1, 2,  4) ],       #g6  Cost
    [ (0.2, 0.33, 1),      ( 1, 3, 5),      (  3,  5,  7 ),   ( 1/7, 1/5, 1/3),   ( 0.25,  0.5,  1),  ( 1,  1,  3) ]        #g6  dwell time
    ])

# VOICE
#    RSS [dBm]      Delay [ms]   Jitter [ms]   Loss Rate [%]    Cost [up to 50]    dweel time [s] 
#      0.176          0.137         0.312          0.067             0.036             0.272      
# RC: 0.15
# metrics_wlan = list ([-75,    70,   (15),   3  ,  5,  (60) ]);  Jitter  (0:150) vs Dwel time (0:150) image  
# metrics_wlan = list ([(-75),  70,   (15),   3  ,  5,  60 ]);  Jitter  (0:150) vs RSS  (-100:-20)   image

dataset_voice = list([
    #   RSS                Delay           Jitter              Loss Rate          Cost         dweel time
    [ (  1,  1,  3 ),   ( 1, 2, 4),       ( 1/5, 1/3, 1),   ( 1, 3, 5),     ( 3, 5, 7),   ( 1/4, 1/2, 1) ],     #g1  RSS
    [ ( 1/4, 1/2, 1),   ( 1, 1, 3 ),      ( 1/5,  1/3, 1),  ( 1, 3, 5),     ( 3, 5, 7),   ( 1/5, 1/3, 1) ],     #g3  Delay
    [ ( 1, 3, 5),       ( 1, 3, 5),       ( 1,  1,  3 ),    ( 3, 5, 7),     ( 3, 5, 7),   ( 1, 1, 3) ],         #g4  Jitter  
    [ ( 1/5, 1/3, 1),   ( 1/5, 1/3, 1),   ( 1/7, 1/5, 1/3), ( 1, 1, 3),     ( 1, 2, 4),   ( 1/7, 1/5, 1/3) ],   #g5  Loss Rate
    [ (1/7, 1/5, 1/3),  ( 1/7, 1/5, 1/3), ( 1/7, 1/5, 1/3), ( 1/4, 1/2, 1), ( 1, 1, 3),   ( 1/9, 1/7, 1/5) ],   #g6  Cost
    [ (1, 2, 4),        ( 1, 3, 5),       ( 1/3, 1,  1 ),   ( 3, 5, 7),     ( 5, 7, 9),   ( 1,  1,  3) ]        #g6  dwell time
    ])


# Call Fuzzy AHP Function        
fuzzy_weights, defuzzified_weights, normalized_weights, rc = fuzzy_ahp_method(dataset_voice)

# # Fuzzy Weigths
# for i in range(0, len(fuzzy_weights)):
#     print('g'+str(i+1)+': ', np.around(fuzzy_weights[i], 3))
# print()

# # Crisp Weigths
# for i in range(0, len(defuzzified_weights)):
#   print('g'+str(i+1)+': ', round(defuzzified_weights[i], 3))
# print()

# print in table format 
# Assuming that 'weights' is a list with 6 values
#   RSS [dBm]  Delay [ms]  Jitter [ms]  Loss Rate [%]  Cost [up to 50]   dweel time [s]   
def print_table_format(weights):
    print("{:^16} {:^12} {:^13} {:^16} {:^18} {:^16}".format(
        "RSS [dBm]", "Delay [ms]", "Jitter [ms]", "Loss Rate [%]", "Cost [up to 50]", "dweel time [s]"
    ))

    print("{:^16.3f} {:^12.3f} {:^13.3f} {:^16.3f} {:^18.3f} {:^16.3f}".format(
        weights[0], weights[1], weights[2],
        weights[3], weights[4], weights[5]
    ))

print_table_format(normalized_weights)



# Consistency Ratio
print('RC: ' + str(round(rc, 2)))
if (rc > 0.10):
    print('The solution is inconsistent, the pairwise comparisons must be reviewed')
else:
    print('The solution is consistent')

# normalized functions 
def bilateral_ben (val, a, b):
    e =  2.718281828459045
    p = -a * (val-b); 
    res = 1 / (1 + pow (e,p))
    return res;

def bilateral_cost (val, a, b):
    return 1 - bilateral_ben(val, a, b) 

def unilateral_ben (val, g):
    if val == 0:
        return -1
    return 1 - (g/val)

def unilateral_cost (val, g):
    return 1- (g*val)

# Normalization functions 

def calc_norm_web (metrics_wlan, metrics_lte): 

    metrics_wlan [0] = bilateral_ben    (metrics_wlan[0], 0.15, -80)    # rss   [-100 : -30 dBm]
    metrics_wlan [1] = bilateral_cost   (metrics_wlan[1], 0.03, 250)    # delay [ 100: 400 ms]
    metrics_wlan [2] = bilateral_cost   (metrics_wlan[2], 0.07, 80)     # jitter  [10 : 150 ms]
    metrics_wlan [3] = unilateral_cost  (metrics_wlan[3], 1/20)         # plr [ <20 %]
    metrics_wlan [4] = unilateral_cost  (metrics_wlan[4], 1/50)         # cost max cost is 50
    metrics_wlan [5] = unilateral_ben   (metrics_wlan[5], 10)            # dweel  [> 10 s]

    metrics_lte [0] = bilateral_ben     (metrics_lte[0], 0.15, -80)     # RSRP [-100 : -30 dBm]
    metrics_lte [1] = bilateral_cost    (metrics_lte[1], 0.03, 250)     # delay
    metrics_lte [2] = bilateral_cost    (metrics_lte[2], 0.07, 80)      # jitter
    metrics_lte [3] = unilateral_cost   (metrics_lte[3], 1/20)          # plr
    metrics_lte [4] = unilateral_cost   (metrics_lte[4], 1/50)          # cost
    metrics_lte [5] = unilateral_ben    (metrics_lte[5], 10)             # dweel

def calc_norm_voice (metrics_wlan, metrics_lte): 
    metrics_wlan [0] = bilateral_ben    (metrics_wlan[0], 0.15, -80)    # rss   [-100 : -30 dBm]
    metrics_wlan [1] = bilateral_cost   (metrics_wlan[1], 0.1, 70)      # delay [ 100: 400 ms]
    metrics_wlan [2] = bilateral_cost   (metrics_wlan[2], 0.15, 40)     # jitter  [10 : 150 ms]
    metrics_wlan [3] = unilateral_cost  (metrics_wlan[3], 1/20)         # plr [ <20 %]
    metrics_wlan [4] = unilateral_cost  (metrics_wlan[4], 1/50)         # cost max cost is 50
    metrics_wlan [5] = unilateral_ben   (metrics_wlan[5], 30)           # dweel  [> 10 s]

    metrics_lte [0] = bilateral_ben     (metrics_lte[0], 0.15, -80)     # RSRP [-100 : -30 dBm]
    metrics_lte [1] = bilateral_cost    (metrics_lte[1], 0.1, 70)       # delay
    metrics_lte [2] = bilateral_cost    (metrics_lte[2], 0.15, 40)      # jitter
    metrics_lte [3] = unilateral_cost   (metrics_lte[3], 1/20)          # plr
    metrics_lte [4] = unilateral_cost   (metrics_lte[4], 1/50)          # cost
    metrics_lte [5] = unilateral_ben    (metrics_lte[5], 30)             # dweel > 30 s

def calc_norm_video (metrics_wlan, metrics_lte): 
    metrics_wlan [0] = bilateral_ben    (metrics_wlan[0], 0.15, -80)    # rss   [-100 : -30 dBm]
    metrics_wlan [1] = bilateral_cost   (metrics_wlan[1], 0.07, 120)    # delay [ 100: 400 ms]
    metrics_wlan [2] = bilateral_cost   (metrics_wlan[2], 0.12, 50)     # jitter  [10 : 150 ms]
    metrics_wlan [3] = unilateral_cost  (metrics_wlan[3], 1/20)         # plr [ <20 %]
    metrics_wlan [4] = unilateral_cost  (metrics_wlan[4], 1/50)         # cost max cost is 50
    metrics_wlan [5] = unilateral_ben   (metrics_wlan[5], 20)           # dweel  [> 10 s]

    metrics_lte [0] = bilateral_ben     (metrics_lte[0], 0.15, -80)     # RSRP [-100 : -30 dBm]
    metrics_lte [1] = bilateral_cost    (metrics_lte[1], 0.07, 120)     # delay
    metrics_lte [2] = bilateral_cost    (metrics_lte[2], 0.12, 50)      # jitter
    metrics_lte [3] = unilateral_cost   (metrics_lte[3], 1/20)          # plr
    metrics_lte [4] = unilateral_cost   (metrics_lte[4], 1/50)          # cost
    metrics_lte [5] = unilateral_ben    (metrics_lte[5], 20)            # dweel > 20 



#   RSS [dBm]  Delay [ms]  Jitter [ms]  Loss Rate [%]  Cost [up to 50]   dweel time [s]   
metrics_wlan = list ([-75, 70, 15, 3 , 5 , 60  ]);  
metrics_lte = list ([-75, 15, 5, 5, 40, 100]); 

calc_norm_voice(metrics_wlan, metrics_lte) # normalizing metrics for voice

# print('WLAN: ' + str(metrics_wlan))
# print('LTE: ' + str(metrics_lte))

# multiplied_wlan = list(np.multiply(metrics_wlan, normalized_weights))
# S_wlan = sum(multiplied_wlan)
# # print S_wlan
# print('S_wlan: ' + str(round(S_wlan, 3)))


# remove plr and dwell from metrics
#metrics_wlan [0] = 0          # rss  [-100 : -30 dBm]
metrics_wlan [2] = 0          # jitter 
# metrics_wlan [3] = 0        # plr [ <15 %]
metrics_wlan [5] = 0          # dweel  [> 10 s]

multiplied_wlan = list(np.multiply(metrics_wlan, normalized_weights))
multiplied_lte = list(np.multiply(metrics_lte, normalized_weights)) 

S_wlan = sum(multiplied_wlan)
S_let =  sum(multiplied_lte)

###
# Make data.
x_val = np.linspace(0.001, 100, num = 300)    # jitter
y_val = np.linspace(0.1, 120, num = 300)   # dwell time

# x_val = np.linspace(0.01, 30, num = 300)    # plr
# y_val = np.linspace(1, 60., num = 300)

#print without exponential notation
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

X, Y = np.meshgrid(x_val, y_val)

#all elements of x_val pass through unilateral_cost function
# x_val = unilateral_cost(x_val, 1/10)    # plr [ <15 %]

for i in range(0, len(y_val)):
    y_val[i] = unilateral_ben(y_val[i], 30)           # dweel  [> 30 s]
    #y_val[i] = bilateral_ben  (y_val[i], 0.15, -80)   # rss   [-100 : -30 dBm]
    x_val [i] =  bilateral_cost (x_val[i], 0.15, 40)      # jitter  

x_val, y_val = np.meshgrid(x_val, y_val)
# each element of Z is the result of the function
# Z = S_wlan + x_val*normalized_weights[3] + y_val*normalized_weights[5] 
Z = S_wlan + x_val*normalized_weights[2] + y_val*normalized_weights[5]    # jitter and dweel

# for each element of Z, if it is lower than 0, then it is set to 0
Z[Z < 0] = 0

# print size of Z in bytes
#print (sys.getsizeof(Z))
# print (X)
# print (Y)
# print ()
# print (Z)

# Plot the surface
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=True)

# Customize the z axis.
ax.set_zlim(-0.01, 1.01)
ax.set_xlabel('Jitter (ms)')
ax.set_ylabel('Dwell Time (s)')
ax.set_zlabel('Access Probability')
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
#ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()



