import numpy as np

left_should = np.array([-.101731, -.0520700, -.650879, 1])
right_should = np.array([-.287711, .169372, -.801845, 1])
upper_spine = np.array([.209333, .252782, -.918178, 1])
lower_spine = np.array([ .304443, .270415, -.988594, 1])
# upper and lower spine values should be at approx the same height above the couch
center_chest = np.array([-.217534, 0.053130, -.708382, 1])
center_shoulder_side = np.array([-.079892, -.160967, -.652748, 1]) # can use this value to offset centre chest

#we added the 1s as 4th element for matrix purposes

points = np.vstack((left_should, right_should, upper_spine, lower_spine, center_chest, center_shoulder_side))
a, b = np.shape(points)

print(a)
print(b)

#want to do it all with matrices

trans_mat = np.array([[1, 0, 0, -center_chest[0]], 
                      [0, 1, 0, -center_chest[1]],
                      [0, 0, 1, -center_chest[2]],
                      [0, 0, 0, 1]])
                    
print('eeee')
print(np.shape(points[1, :]))

for i in range(a):
    points[i, :] = np.matmul(trans_mat, points[i, :])

print(points)

points_transd = points[:, :3]
print(points_transd)



#first we want to shift everything so that centre_chest is at 