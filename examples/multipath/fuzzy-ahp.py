#!/usr/bin/python3

# Required Libraries
import numpy as np

# Fuzzy AHP 
from pyDecision.algorithm import fuzzy_ahp_method

# Dataset
# dataset = list([
#     [ (  1,   1,   1), (  4,   5,   6), (  3,   4,   5), (  6,   7,   8) ],   #g1
#     [ (1/6, 1/5, 1/4), (  1,   1,   1), (1/3, 1/2, 1/1), (  2,   3,   4) ],   #g2
#     [ (1/5, 1/4, 1/3), (  1,   2,   3), (  1,   1,   1), (  2,   3,   4) ],   #g3
#     [ (1/8, 1/7, 1/6), (1/4, 1/3, 1/2), (1/4, 1/3, 1/2), (  1,   1,   1) ]    #g4
#     ])


# dataset = list([
#     #   RSS                Delay           Jitter              Loss Rate          Cost               dweel time
#     [ (  1,  1,  3 ),    (  3, 5, 7 ),    (  5,  7,  9 ),   ( 1, 3, 5 ),      ( 2, 4, 6),         ( 3,  5,  7) ],       #g1  RSS
#     [ ( 1/7, 1/5, 1/3),  (  1, 1, 3 ),    (  3,  5,  7 ),   ( 1/7, 1/5, 1/3), ( 1, 1, 3),         ( 1, 1, 3) ],         #g3  Delay
#     [ ( 1/9, 1/7, 1/5),  ( 1/7, 1/5, 1/3),(  1,  1,  3 ),   ( 1/9, 1/7, 1/5), ( 1/7, 1/5, 1/3),   ( 1/7, 1/5, 1/3) ],   #g4  Jitter  
#     [ ( 1/5, 1/3, 1),    ( 3, 5, 7 ),     (  5,  7,  9 ),   ( 1,   1,   3),   ( 3,   5,   7),     ( 3, 5, 7) ],         #g5  Loss Rate
#     [ (1/6, 1/4, 1/2),   ( 1/3, 1, 1),    (  3,  5,  7 ),   ( 1/7, 1/5, 1/3), ( 1,   1,   3),     ( 1/5,  1/3,  1) ],   #g6  Cost
#     [ (1/7, 1/5, 1/3),   ( 1/3, 1, 1),    (  3,  5,  7 ),   ( 1/7, 1/5, 1/3), ( 1, 3,  5),        ( 1,  1,  3) ]        #g6  dwell time
#     ])

dataset = list([
    #   RSS                Delay           Jitter              Loss Rate          Cost               dweel time
    [ (  1,  1,  3 ),    (  3, 5, 7 ),    (  5,  7,  9 ),   ( 1/5, 1/3, 1 ),      ( 2, 4, 6),         ( 3,  5,  7) ],       #g1  RSS
    [ ( 1/7, 1/5, 1/3),  (  1, 1, 3 ),    (  3,  5,  7 ),   ( 1/7, 1/5, 1/3), ( 1, 1, 3),         ( 1, 1, 3) ],         #g3  Delay
    [ ( 1/9, 1/7, 1/5),  ( 1/7, 1/5, 1/3),(  1,  1,  3 ),   ( 1/9, 1/7, 1/5), ( 1/7, 1/5, 1/3),   ( 1/7, 1/5, 1/3) ],   #g4  Jitter  
    [ ( 1, 3, 5),        ( 3, 5, 7 ),     (  5,  7,  9 ),   ( 1,   1,   3),   ( 3,   5,   7),     ( 3, 5, 7) ],         #g5  Loss Rate
    [ (1/6, 1/4, 1/2),   ( 1/3, 1, 1),    (  3,  5,  7 ),   ( 1/7, 1/5, 1/3), ( 1,   1,   3),     ( 1/5,  1/3,  1) ],   #g6  Cost
    [ (1/7, 1/5, 1/3),   ( 1/3, 1, 1),    (  3,  5,  7 ),   ( 1/7, 1/5, 1/3), ( 1, 3,  5),        ( 1,  1,  3) ]        #g6  dwell time
    ])


# Call Fuzzy AHP Function        
fuzzy_weights, defuzzified_weights, normalized_weights, rc = fuzzy_ahp_method(dataset)

# # Fuzzy Weigths
# for i in range(0, len(fuzzy_weights)):
#     print('g'+str(i+1)+': ', np.around(fuzzy_weights[i], 3))
# print()

# # Crisp Weigths
# for i in range(0, len(defuzzified_weights)):
#   print('g'+str(i+1)+': ', round(defuzzified_weights[i], 3))
# print()

# Normalized Weigths
for i in range(0, len(normalized_weights)):
    print('g'+str(i+1)+': ', round(normalized_weights[i], 3))
print()

# Consistency Ratio
print('RC: ' + str(round(rc, 2)))
if (rc > 0.10):
    print('The solution is inconsistent, the pairwise comparisons must be reviewed')
else:
    print('The solution is consistent')

def bilateral_ben (val, a, b):
    e =  2.718281828459045
    p = -a * (val-b); 
    res = 1 / (1 + pow (e,p))
    return res;

def bilateral_cost (val, a, b):
    return 1 - bilateral_ben(val, a, b) 

def unilateral_ben (val, g):
    return 1 - (g/val)

def unilateral_cost (val, g):
    return 1- (g*val)

#   RSS [dBm]  Delay [ms]  Jitter [ms]  Loss Rate [%]  Cost [up to 50]   dweel time [s]   
metrics_wlan = list ([-76.78, 70, 3.25, 0, 5, 14]);  
metrics_lte = list ([-74.15, 11.67, 0.53, 4.91, 40, 100]); 

metrics_wlan [0] = bilateral_ben (metrics_wlan[0], 0.15, -80) # rss   [-100 : -30 dBm]
metrics_wlan [1] = bilateral_cost (metrics_wlan[1], 0.03, 250)# delay [ 100: 400 ms]
metrics_wlan [2] = bilateral_cost (metrics_wlan[2], 0.07, 60)  # jitter  [10 : 150 ms]
metrics_wlan [3] = unilateral_cost (metrics_wlan[3], 1/15)    # plr [ <15 %]
metrics_wlan [4] = unilateral_cost (metrics_wlan[4], 1/50)    # cost max cost is 50
metrics_wlan [5] = unilateral_ben (metrics_wlan[5], 5)       # dweel  [> 10 s]

metrics_lte [0] = bilateral_ben (metrics_lte[0], 0.15, -80)
metrics_lte [1] = bilateral_cost (metrics_lte[1], 0.03, 250)    # delay
metrics_lte [2] = bilateral_cost (metrics_lte[2], 0.07, 60)      # jitter
metrics_lte [3] = unilateral_cost (metrics_lte[3], 1/15)        # plr
metrics_lte [4] = unilateral_cost (metrics_lte[4], 1/50)        # cost
metrics_lte [5] = unilateral_ben (metrics_lte[5], 5)           # dweel

print (metrics_wlan)
print (metrics_lte)

multiplied_wlan = list(np.multiply(metrics_wlan, normalized_weights))
multiplied_lte = list(np.multiply(metrics_lte, normalized_weights)) 

print (multiplied_wlan)

print (sum(multiplied_wlan))
print (sum(multiplied_lte))