import os, sys
import numpy as numpy
import cv2

path = sys.argvs[1]

imgs = os.listdir(path)
r = []
g = []
b = []
eps = 1e-8
for img in imgs:
  if '.png' not in img:
    continue

  mat = cv2.imread(os.path.join(path, img))
  mat = mat.mean(0).mean(0)
  b.append(mat[0])
  g.append(mat[1])
  r.append(mat[2])
B, G, R = np.mean(b), np.mean(g), np.mean(r)

print('mean: ', B, G, R)

M = np.array([B, G, R])
r = []
g = []
b = []
eps = 1e-8
for img in imgs:
  if '.png' not in img:
    continue

  mat = cv2.imread(os.path.join(img_path, img))
  mat = mat - M
  mat = mat**2 + eps
  
  mat = mat.mean(0).mean(0)
  b.append(mat[0])
  g.append(mat[1])
  r.append(mat[2])
B, G, R = np.mean(b), np.mean(g), np.mean(r)
print('std: ', np.sqrt(B+eps), np.sqrt(G+eps), np.sqrt(R+eps))
