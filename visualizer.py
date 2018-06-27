#!/usr/bin/python

from PIL import Image, ImageDraw
import cv2
import numpy

dir_path = "./"
filename = "result.png"

red = (255,0,0)
blue = (0, 0, 255)
green = (0, 255, 0)
white = (255, 255, 255)

image = Image.new("RGB", (1000, 1000), white)
draw = ImageDraw.Draw(image)

with open(dir_path + "input.txt") as f:
    drones = int(f.readline())

    cx = [None] * drones
    cy = [None] * drones
    for i in range(0, drones):
        cx[i], cy[i] = map(float, f.readline().split(" "))
    mxX = mnX = cx[0]
    mxY = mnY = cy[0]
    
    for i in range(1, drones):
        mnX = min(mnX, cx[i])
        mxX = max(mxX, cx[i])
        mnY = min(mnY, cx[i])
        mxY = max(mxY, cy[i])
    
    _, r = map(float, f.readline().split(" "));
    ellipse_radius = r
    
    n = int(f.readline())
    p = [None] * n
    for i in range(0, n):
        x, y = map(float, f.readline().split(" "))
        p[i] = (x, y)
        mnX = min(mnX, x)
        mxX = max(mxX, x)
        mnY = min(mnY, y)
        mxY = max(mxY, y)
        
    p.append(p[0])
        
scale = 960 / max(mxX - mnX, mxY - mnY)

for i in range(0, drones):
    cx[i] = 20 + scale * (cx[i] - mnX)
    cy[i] = 20 + scale * (cy[i] - mnY)

for i in range(0, n + 1):
    (x, y) = p[i]
    x = 20 + scale * (x - mnX) 
    y = 20 + scale * (y - mnY)
    p[i] = (x, y)
    
r = scale * r
for i in range(0, drones):
    draw.ellipse((cx[i] - r, cy[i] - r, cx[i] + r, cy[i] + r), red)
draw.line(p, blue, width = 10)

opencvImage = cv2.cvtColor(numpy.array(image), cv2.COLOR_RGB2BGR)
cv2.imshow("Press any key to show next step", opencvImage)
cv2.waitKey()

with open(dir_path + "output.txt") as f:
    f.readline()
    
    for i in range(0, drones):
        m = int(f.readline())
        path = [None] * m
        for j in range(0, m):
            x, y = map(float, f.readline().split(" "))
            x = 20 + scale * (x - mnX) 
            y = 20 + scale * (y - mnY)
            path[j] = (x, y)
            
        for j in range(0, m - 1):
            draw.line(path[j:j+2], fill = ((30 + 50 * i) % 256, (30 + 150 * i) % 256, (30 + 50 * i) % 256), width = 5)
            opencvImage = cv2.cvtColor(numpy.array(image), cv2.COLOR_RGB2BGR)
            cv2.imshow("Press any key to show next step", opencvImage)
            cv2.waitKey()

image.save(filename)
cv2.destroyAllWindows()

