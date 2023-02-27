#!/usr/bin/python3

import sys

# Required Libraries
import numpy as np

#import random module
import random

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
#    RSS [dBm]      Delay [ms]   Jitter [ms]   Loss Rate [%]    Cost [up to 50]    dweel time [s] 
#      0.221          0.106         0.031          0.328             0.179             0.134      
# RC: 0.16
# metrics_wlan = list ([-75,    70,   15,   (30)  ,  5,   (60)  ]);  PLR vs Dwel time image 
# metrics_wlan = list ([(-75),    70,   15,   (30)  ,  5,   60  ]);  PLR vs RSS       image

dataset_web = list([
    #   RSS                Delay           Jitter              Loss Rate          Cost               dweel time
    [ ( 1,  1,  3 ),      ( 1, 3, 5 ),     (  3,  5,  7 ),   ( 1/5, 1/3, 1 ),    ( 1, 1, 3),        ( 1, 3 , 5) ],        #g1  RSS
    [ ( 1/5, 1/3, 1 ),    ( 1, 1, 3 ),     (  3,  5, 7  ),   ( 1/5, 1/3, 1),     ( 1/4, 1/2, 1),    ( 1/4, 1/2, 1) ],     #g3  Delay
    [ ( 1/7, 1/5, 1/3),   ( 1/7, 1/5, 1/3),(  1,  1,  3 ),   ( 1/9, 1/7, 1/5),   ( 1/7, 1/5, 1/3),  ( 1/7, 1/5, 1/3) ],   #g4  Jitter  
    [ ( 1, 3, 5 ),        ( 1, 3, 5 ),     (  5,  7,  9 ),   ( 1, 1, 3),         ( 1,   2,   4),    ( 1, 3, 5) ],         #g5  Loss Rate
    [ (1/3, 1, 1),        ( 1, 2, 4),      (  3,  5,  7 ),   ( 1/4, 1/2, 1),     ( 1,   1,   3),    ( 1, 2,  4) ],        #g6  Cost
    [ (1/5, 1/3, 1),      ( 1, 2, 4),      (  3,  5,  7 ),   ( 1/5, 1/3, 1),     ( 1/4,  1/2, 1),   ( 1,  1,  3) ]        #g6  dwell time
    ])

# VOICE
#    RSS [dBm]      Delay [ms]   Jitter [ms]   Loss Rate [%]    Cost [up to 50]    dweel time [s] 
#      0.160          0.314         0.210          0.071             0.048             0.198      
# RC: 0.17
# metrics_wlan = list ([-75,    70,   (15),   3  ,  5,  (60) ]);  Jitter  (0:150) vs Dwel time (0:150) image  
# metrics_wlan = list ([(-75),  70,   (15),   3  ,  5,  60 ]);  Jitter  (0:150) vs RSS  (-100:-20)   image

dataset_voice = list([
    #   RSS                Delay           Jitter             Loss Rate       Cost          dweel time
    [ ( 1,  1,  3 ),    ( 1/4, 1/2, 1),   ( 1/3, 1, 1),     ( 1, 3, 5),     ( 3, 5, 7),   ( 1/3, 1, 1) ],       #g1  RSS
    [ ( 1, 2, 4),       ( 1, 1, 3 ),      ( 1, 3, 5),       ( 1, 3, 5),     ( 3, 5, 7),   ( 1, 2, 4) ],         #g3  Delay
    [ ( 1, 1, 3),       ( 1/5, 1/3, 1),   ( 1,  1,  3 ),    ( 3, 5, 7),     ( 1, 3, 5),   ( 1, 1, 3) ],         #g4  Jitter  
    [ ( 1/5, 1/3, 1),   ( 1/5, 1/3, 1),   ( 1/7, 1/5, 1/3), ( 1, 1, 3),     ( 1, 2, 4),   ( 1/7, 1/5, 1/3) ],   #g5  Loss Rate
    [ (1/7, 1/5, 1/3),  ( 1/7, 1/5, 1/3), ( 1/5, 1/3, 1),   ( 1/4, 1/2, 1), ( 1, 1, 3),   ( 1/7, 1/5, 1/3) ],   #g6  Cost
    [ (1, 1, 3),        ( 1/4, 1/2, 1),   ( 1/3, 1,  1 ),   ( 3, 5, 7),     ( 3, 5, 7),   ( 1,  1,  3) ]        #g6  dwell time
    ])

# VIDEO (interative)
#    RSS [dBm]      Delay [ms]   Jitter [ms]   Loss Rate [%]    Cost [up to 50]    dweel time [s] 
#      0.187          0.178         0.304          0.133             0.080             0.117      
# RC: 0.17
# metrics_wlan = list ([-75,    70,   (15),   3  ,  5,  (60) ]);  Jitter  (0:150) vs Dwel time (0:150) image  
# metrics_wlan = list ([(-75),  70,   (15),   3  ,  5,  60 ]);  Jitter  (0:150) vs RSS  (-100:-20)   image

dataset_video = list([
    #   RSS                Delay           Jitter             Loss Rate          Cost        dweel time
    [ ( 1,  1,  3 ),    ( 1, 1, 3),       ( 1/5, 1/3, 1),   ( 1, 1, 3),       ( 1, 3, 5),   ( 1, 1, 3) ],         #g1  RSS
    [ ( 1/3, 1, 1),     ( 1, 1, 3 ),      ( 1/3, 1, 1),     ( 1, 1, 3),       ( 1, 3, 5),   ( 1, 2, 4) ],         #g3  Delay
    [ ( 1, 3, 5),       ( 1, 1, 3),       ( 1,  1,  3 ),    ( 1, 3, 5),       ( 1, 3, 5),   ( 1, 3, 5) ],         #g4  Jitter  
    [ ( 1/3, 1, 1),     ( 1/3, 1, 1),     ( 1/5, 1/3, 1),   ( 1, 1, 3),       ( 1, 2, 4),   ( 1, 1, 3) ],       #g5  Loss Rate
    [ (1/5, 1/3, 1),    ( 1/5, 1/3, 1),   ( 1/5, 1/3, 1),   ( 1/5, 1/3, 1),   ( 1, 1, 3),   ( 1/5, 1/3, 1) ],     #g6  Cost
    [ (1/3, 1, 1),      ( 1/4, 1/2, 1),   ( 1/5, 1/3, 1),   ( 1/3, 1, 1),     ( 1, 3, 5),   ( 1,  1,  3) ]        #g6  dwell time
    ])



# Call Fuzzy AHP Function (normalized weights are important for the next step)       
fuzzy_weights_web, defuzzified_weights_web, normalized_weights_web, rc_web = fuzzy_ahp_method(dataset_web)
fuzzy_weights_voice, defuzzified_weights_voice, normalized_weights_voice, rc_voice = fuzzy_ahp_method(dataset_voice)
fuzzy_weights_video, defuzzified_weights_video, normalized_weights_video, rc_video = fuzzy_ahp_method(dataset_video)

# # # Fuzzy Weigths
# for i in range(0, len(fuzzy_weights)):
#     print('g'+str(i+1)+': ', np.around(fuzzy_weights[i], 3))
# print()

# # # Crisp Weigths
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

print_table_format(normalized_weights_web)
print_table_format(normalized_weights_voice)
print_table_format(normalized_weights_video)


# Consistency Ratio  (...UPS.. our solution is not consistent)
# print('RC: ' + str(round(rc, 2)))
# if (rc > 0.10):
#     print('The solution is inconsistent, the pairwise comparisons must be reviewed')
# else:
#     print('The solution is consistent')

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
    
    m_lte = [0,0,0,0,0,0]
    m_wlan = [0,0,0,0,0,0]

    m_wlan [0] = bilateral_ben    (metrics_wlan[0], 0.15, -80)    # rss   [-100 : -30 dBm]
    m_wlan [1] = bilateral_cost   (metrics_wlan[1], 0.03, 250)    # delay [ 100: 400 ms]
    m_wlan [2] = bilateral_cost   (metrics_wlan[2], 0.07, 80)     # jitter  [10 : 150 ms]
    m_wlan [3] = unilateral_cost  (metrics_wlan[3], 1/20)         # plr [ <20 %]
    m_wlan [4] = unilateral_cost  (metrics_wlan[4], 1/50)         # cost max cost is 50
    m_wlan [5] = unilateral_ben   (metrics_wlan[5], 10)            # dweel  [> 10 s]

    m_lte [0] = bilateral_ben     (metrics_lte[0], 0.15, -80)     # RSRP [-100 : -30 dBm]
    m_lte [1] = bilateral_cost    (metrics_lte[1], 0.03, 250)     # delay
    m_lte [2] = bilateral_cost    (metrics_lte[2], 0.07, 80)      # jitter
    m_lte [3] = unilateral_cost   (metrics_lte[3], 1/20)          # plr
    m_lte [4] = unilateral_cost   (metrics_lte[4], 1/50)          # cost
    m_lte [5] = unilateral_ben    (metrics_lte[5], 10)             # dweel

    return m_wlan, m_lte

def calc_norm_voice (metrics_wlan, metrics_lte): 
    
    m_lte = [0,0,0,0,0,0]
    m_wlan = [0,0,0,0,0,0]

    m_wlan [0] = bilateral_ben    (metrics_wlan[0], 0.15, -80)    # rss   [-100 : -30 dBm]
    m_wlan [1] = bilateral_cost   (metrics_wlan[1], 0.1, 70)      # delay [ 100: 400 ms]
    m_wlan [2] = bilateral_cost   (metrics_wlan[2], 0.15, 40)     # jitter  [10 : 150 ms]
    m_wlan [3] = unilateral_cost  (metrics_wlan[3], 1/20)         # plr [ <20 %]
    m_wlan [4] = unilateral_cost  (metrics_wlan[4], 1/50)         # cost max cost is 50
    m_wlan [5] = unilateral_ben   (metrics_wlan[5], 30)           # dweel  [> 10 s]

    m_lte [0] = bilateral_ben     (metrics_lte[0], 0.15, -80)     # RSRP [-100 : -30 dBm]
    m_lte [1] = bilateral_cost    (metrics_lte[1], 0.1, 70)       # delay
    m_lte [2] = bilateral_cost    (metrics_lte[2], 0.15, 40)      # jitter
    m_lte [3] = unilateral_cost   (metrics_lte[3], 1/20)          # plr
    m_lte [4] = unilateral_cost   (metrics_lte[4], 1/50)          # cost
    m_lte [5] = unilateral_ben    (metrics_lte[5], 30)             # dweel > 30 s
    
    return m_wlan, m_lte

def calc_norm_video (metrics_wlan, metrics_lte):

    m_lte = [0,0,0,0,0,0]
    m_wlan = [0,0,0,0,0,0]

    m_wlan [0] = bilateral_ben    (metrics_wlan[0], 0.15, -80)    # rss   [-100 : -30 dBm]
    m_wlan [1] = bilateral_cost   (metrics_wlan[1], 0.07, 120)    # delay [ 100: 400 ms]
    m_wlan [2] = bilateral_cost   (metrics_wlan[2], 0.12, 50)     # jitter  [10 : 150 ms]
    m_wlan [3] = unilateral_cost  (metrics_wlan[3], 1/20)         # plr [ <20 %]
    m_wlan [4] = unilateral_cost  (metrics_wlan[4], 1/50)         # cost max cost is 50
    m_wlan [5] = unilateral_ben   (metrics_wlan[5], 20)           # dweel  [> 10 s]

    m_lte [0] = bilateral_ben     (metrics_lte[0], 0.15, -80)     # RSRP [-100 : -30 dBm]
    m_lte [1] = bilateral_cost    (metrics_lte[1], 0.07, 120)     # delay
    m_lte [2] = bilateral_cost    (metrics_lte[2], 0.12, 50)      # jitter
    m_lte [3] = unilateral_cost   (metrics_lte[3], 1/20)          # plr
    m_lte [4] = unilateral_cost   (metrics_lte[4], 1/50)          # cost
    m_lte [5] = unilateral_ben    (metrics_lte[5], 20)            # dweel > 20 

    return m_wlan, m_lte

#   RSS [dBm]  Delay [ms]  Jitter [ms]  Loss Rate [%]  Cost [up to 50]   dweel time [s]   
metrics_wlan = list ([-75, 70, 15, 3 , 5 , 60  ]);  
metrics_lte = list ([-75, 15, 5, 5, 40, 100]); 

# [0] = LTE, [1] = LTE-BACKUP , [2] = WLAN , [3] = WLAN-BACKUP
prob_list_web   = list ([0, 0, 0, 0 ]) # list of probabilities for web service  
prob_list_voice = list ([0, 0, 0, 0 ]) # list of probabilities for voice  
prob_list_video = list ([0, 0, 0, 0 ]) # list of probabilities for video  

def check_prob (prob_list, s_wlan, s_lte):
    if (s_lte >= 0.75):
        prob_list [0] = prob_list [0] + 1
    elif (s_lte >= 0.50): 
        prob_list [1] = prob_list [1] + 1

    if (s_wlan >= 0.75):
        prob_list [2] = prob_list [2] + 1
    elif (s_wlan >= 0.50): 
        prob_list [3] = prob_list [3] + 1 


# generate random metrics and calculate provbability for each service
SIM_TIMES = 100000
for i in range (0, SIM_TIMES):
    # genetare random metrics for wlan 
    metrics_wlan [0] = random.randint(-100, -30)    # rss    [-100 : -30 dBm]
    metrics_wlan [1] = random.randint(20, 250)      # delay  [20: 250 ms]
    metrics_wlan [2] = random.randint(5, 100)       # jitter [5 : 100 ms]
    metrics_wlan [3] = random.randint(2, 20)        # plr    [2 : 20 %]
    metrics_wlan [4] = random.randint(0, 20)        # cost   [0 : 20]
    metrics_wlan [5] = random.randint(5, 120)       # dweel  [5 : 120 s]

    # generate random metrics for lte
    metrics_lte [0] = random.randint(-100, -30)     # RSRP   [-100 : -30 dBm]
    metrics_lte [1] = random.randint(5, 80)         # delay  [5: 80 ms]
    metrics_lte [2] = random.randint(2, 50)         # jitter [2 : 50 ms]
    metrics_lte [3] = random.randint(1, 10)         # plr    [2 : 20 %]
    metrics_lte [4] = random.randint(15, 45)        # cost   [15 : 45]
    metrics_lte [5] = random.randint(30, 600)       # dweel  [30 : 600 s]

    # print ("WLAN metrics: ", metrics_wlan)
    # print ("LTE metrics: ", metrics_lte)

    # calculate prob for each service, normalize metrics first 
    norm_wlan, norm_lte = calc_norm_web(metrics_wlan, metrics_lte) # normalizing metrics for web
    s_wlan = sum(list(np.multiply(norm_wlan, normalized_weights_web)))
    s_lte = sum(list(np.multiply(norm_lte, normalized_weights_web)))
    check_prob(prob_list_web, s_wlan, s_lte)

    norm_wlan, norm_lte = calc_norm_voice(metrics_wlan, metrics_lte) # normalizing metrics for voice
    s_wlan = sum(list(np.multiply(norm_wlan, normalized_weights_voice)))
    s_lte = sum(list(np.multiply(norm_lte, normalized_weights_voice)))
    check_prob(prob_list_voice, s_wlan, s_lte)

    norm_wlan, norm_lte = calc_norm_video(metrics_wlan, metrics_lte) # normalizing metrics for video
    s_wlan = sum(list(np.multiply(norm_wlan, normalized_weights_video)))
    s_lte = sum(list(np.multiply(norm_lte, normalized_weights_video)))
    check_prob(prob_list_video, s_wlan, s_lte)

#devede each element of prob_list_web by SIM_TIMES
prob_list_web = np.divide(prob_list_web, SIM_TIMES)
prob_list_voice = np.divide(prob_list_voice, SIM_TIMES)
prob_list_video = np.divide(prob_list_video, SIM_TIMES)

# print prob lists
print('Web: ' + str(prob_list_web))
print('Voice: ' + str(prob_list_voice))
print('Video: ' + str(prob_list_video))


# print('WLAN: ' + str(metrics_wlan))
# print('LTE: ' + str(metrics_lte))

# # print S_wlan
# print('S_wlan: ' + str(round(S_wlan, 3)))


# # remove plr and dwell from metrics
# #metrics_wlan [0] = 0          # rss  [-100 : -30 dBm]
# metrics_wlan [2] = 0          # jitter 
# # metrics_wlan [3] = 0        # plr [ <15 %]
# metrics_wlan [5] = 0          # dweel  [> 10 s]

# multiplied_wlan = list(np.multiply(metrics_wlan, normalized_weights))
# multiplied_lte = list(np.multiply(metrics_lte, normalized_weights)) 

# S_wlan = sum(multiplied_wlan)
# S_let =  sum(multiplied_lte)

# ###
# # Make data.
# x_val = np.linspace(0.001, 100, num = 300)    # jitter
# y_val = np.linspace(0.1, 120, num = 300)   # dwell time

# # x_val = np.linspace(0.01, 30, num = 300)    # plr
# # y_val = np.linspace(1, 60., num = 300)

# #print without exponential notation
# np.set_printoptions(precision=3)
# np.set_printoptions(suppress=True)

# X, Y = np.meshgrid(x_val, y_val)

# #all elements of x_val pass through unilateral_cost function
# # x_val = unilateral_cost(x_val, 1/10)    # plr [ <15 %]

# for i in range(0, len(y_val)):
#     y_val[i] = unilateral_ben(y_val[i], 30)           # dweel  [> 30 s]
#     #y_val[i] = bilateral_ben  (y_val[i], 0.15, -80)   # rss   [-100 : -30 dBm]
#     x_val [i] =  bilateral_cost (x_val[i], 0.15, 40)      # jitter  

# x_val, y_val = np.meshgrid(x_val, y_val)
# # each element of Z is the result of the function
# # Z = S_wlan + x_val*normalized_weights[3] + y_val*normalized_weights[5] 
# Z = S_wlan + x_val*normalized_weights[2] + y_val*normalized_weights[5]    # jitter and dweel

# # for each element of Z, if it is lower than 0, then it is set to 0
# Z[Z < 0] = 0

# # print size of Z in bytes
# #print (sys.getsizeof(Z))
# # print (X)
# # print (Y)
# # print ()
# # print (Z)

# # Plot the surface
# fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
# surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
#                        linewidth=0, antialiased=True)

# # Customize the z axis.
# ax.set_zlim(-0.01, 1.01)
# ax.set_xlabel('Jitter (ms)')
# ax.set_ylabel('Dwell Time (s)')
# ax.set_zlabel('Access Probability')
# ax.zaxis.set_major_locator(LinearLocator(10))
# # A StrMethodFormatter is used automatically
# #ax.zaxis.set_major_formatter('{x:.02f}')

# # Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)

# plt.show()



