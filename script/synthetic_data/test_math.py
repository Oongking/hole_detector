import numpy as np

# mats = np.array([[1,2,3],[4,5,6]])

# y = np.array([7,8,9])

# test = []
# for mat in mats:
#     test.append(np.dot(mat,y))

# print(f"test : {test}")

# print(f"test2 : {np.tensordot(mats,y,axes=1)}")

# print(f"mul : {(mats.T * test).T}")

# print(f"norm : {np.linalg.norm([5,5,5])}")
# full = np.full([5,3],[5,5,5])
# print(full)

# d  = np.arange(5.0)
# print(d)

# print(np.multiply(full,d.reshape(-1,1)))


# x1 = np.array([[1,1,1],
#                [2,2,2],
#                [3,3,3]])

# x2 = np.array([[1,1,1],
#                [2,2,2],
#                [3,3,3]])

# print(np.add(x1,x2))

selectpoint= np.array([[0.96233966, 0.74743093, 0.63071893],[1.06424259, 0.65211087, 0.67947769]])

print(np.subtract(selectpoint[0],selectpoint[1]))