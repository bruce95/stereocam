from cv2 import *
import numpy as np
from distutils.version import LooseVersion
p1=VideoCapture(2)
p2=VideoCapture(3)
#objr_is_the_points_in_chessboard
numDisparities=16
blockSize=15
obj_points=np.array([
                     [0.8,0.8,0],[1.6,0.8,0],[2.4,0.8,0],[3.2,0.8,0],[4,0.8,0],[4.8,0.8,0],[5.6,0.8,0],
                     [0.8,1.6,0],[1.6,1.6,0],[2.4,1.6,0],[3.2,1.6,0],[4,1.6,0],[4.8,1.6,0],[5.6,1.6,0],
                     [0.8,2.4,0],[1.6,2.4,0],[2.4,2.4,0],[3.2,2.4,0],[4,2.4,0],[4.8,2.4,0],[5.6,2.4,0],
                     [0.8,3.2,0],[1.6,3.2,0],[2.4,3.2,0],[3.2,3.2,0],[4,3.2,0],[4.8,3.2,0],[5.6,3.2,0],
                     [0.8,4.0,0],[1.6,4.0,0],[2.4,4.0,0],[3.2,4.0,0],[4,4.0,0],[4.8,4.0,0],[5.6,4.0,0],
                     [0.8,4.8,0],[1.6,4.8,0],[2.4,4.8,0],[3.2,4.8,0],[4,4.8,0],[4.8,4.8,0],[5.6,4.8,0],
                     [0.8,5.6,0],[1.6,5.6,0],[2.4,5.6,0],[3.2,5.6,0],[4,5.6,0],[4.8,5.6,0],[5.6,5.6,0],
                     ],'float32')

def capture_video(p1,p2):
 p1c=p1.read()[1]
 p2c=p2.read()[1]
 return(p1c,p2c)

def chessboard_calibrate_alone(camera_floder,obj_points,wide_points,high_points,img_size):
  img=[]
  obj=[]
  import glob
  i=glob.glob(camera_floder+'/*.jpg')
  print i
  for igrab in i:
      im=imread(igrab)
      ret,corners=findChessboardCorners(im,(wide_points,high_points))
      if ret:
       img.append(corners)
       obj.append(obj_points)
      else:print'False'
  ret,mtx,dist,rvecs,tvecs=calibrateCamera(obj,img,img_size,None,None)
  return(ret,mtx,dist,rvecs,tvecs,img,obj)
def chessboard_calibrate_alone_with_data_saving(camera_floder,obj_points,wide_points,high_points,img_size,save_name):
  counter=0
  img=[]
  obj=[]
  import glob
  i=glob.glob(camera_floder+'/*.jpg')
  print i
  for igrab in i:
      im=imread(igrab)
      ret,corners=findChessboardCorners(im,(wide_points,high_points))
      if ret:
       print 'True'
       counter+=1
       img.append(corners)
       obj.append(obj_points)
      else:print'False'
  print counter
  ret,mtx,dist,rvecs,tvecs=calibrateCamera(obj,img,img_size,None,None)
  np.save(save_name+'ret',ret)
  np.save(save_name+'mtx',mtx)
  np.save(save_name+'dist',dist)
  np.save(save_name+'img',img)
  np.save(save_name+'obj',obj)
def chessboard_distort_calibrate_alone_with_data_saving(img_size,camera_floder,load=False,load_name=None,data=None):
    if data!=None:
     mtx=data[1]
     dist=data[2]
    if load==True:
     mtx=np.load(load_name+'mtx.npy')
     dist=np.load(load_name+'dist.npy')
    newcameramtx,roi=getOpticalNewCameraMatrix(mtx,dist,img_size,img_size)
    i=glob.glob(camera_floder+'/*.jpg')
    for igrab in i:
        im=imread(igrab)
        dst=undistort(im,mtx,dist,None,newcameramtx)
        imwrite(igrab,dst)
def stereo_calibrate(img1,img2,img_size,data1=None,data2=None,load1=False,load_name1=None,load2=False,load_name2=None):
    if data1!=None:
        c1mtx=data1[1]
        c1dist=data1[2]
        c1img=data1[5]
        c1obj=data1[6]
    if data2!=None:
        c2mtx=data2[1]
        c2dist=data2[2]
        c2img=data2[5]
        c2obj=data2[6]
    if load1==True:
        c1mtx=np.load(load_name1+'mtx.npy')
        c1dist=np.load(load_name1+'dist.npy')
        c1img=np.load(load_name1+'img.npy')
        c1obj=np.load(load_name1+'obj.npy')
        print c1obj
    if load2==True:
        c2mtx=np.load(load_name2+'mtx.npy')
        c2dist=np.load(load_name2+'dist.npy')
        c2img=np.load(load_name2+'img.npy')
        c2obj=np.load(load_name2+'obj.npy')
    print c1img,'!',c2img    
    retral,cameraMatrix1,distCoeffs1,cameraMatrix2,distCoeffs2,R,T,E,F=stereoCalibrate(c1obj,c1img,c2img,img_size,c1mtx,c1dist,c2mtx,c2dist,criteria=stereocalib_criteria, flags=flags)
    R1,R2,P1,P2,Q,t,tl=stereoRectify(cameraMatrix1,distCoeffs1,cameraMatrix2,distCoeffs2,img_size,R,T)
    newcameramtx,roi=getOptimalNewCameraMatrix(c1mtx,c1dist,img_size,1,img_size)
    #img1=undistort(img1,c1mtx,c1dist,None,newcameramtx)
    map1,map2=initUndistortRectifyMap(c1mtx,c1dist,R1,P1,img_size,0)
    img1=remap(imread(img1),map1,map2,0)
    #newcameramtx,roi=getOptimalNewCameraMatrix(c2mtx,c2dist,img_size,1,img_size)
    #img1=undistort(img1,c1mtx,c1dist,None,newcameramtx)
    map1,map2=initUndistortRectifyMap(c2mtx,c2dist,R2,P2,img_size,0)
    img2=remap(imread(img2),map1,map2,0)
    np.save('Q.npy',Q)
    from matplotlib import pyplot as plt
    plt.imshow(img1)
    plt.show()
    plt.imshow(img2)
    plt.show()
    return(img1,img2)
def stereo_match(img_sequence):
    from matplotlib import pyplot as plt
    c=StereoBM(0,numDisparities,blockSize)
    img1=cvtColor(img_sequence[0],COLOR_BGR2GRAY)
    img2=cvtColor(img_sequence[1],COLOR_BGR2GRAY)
    k=c.compute(img1,img2)
    plt.imshow(k,'gray')
    plt.show()
    imwrite('current.jpg',k)
def to2dpoints(k):
    Q=np.load('Q.npy')
    arc=(k,Q)
    return(arc)
def to3dpoints(arc):
    dst=perspectiveTransform(arc,Q)
def takeseq(times):
    i=1
    while i<=times:
     im=p1.read()[1]
     imwrite('cam1/'+str(i)+'.jpg',im)
     im=p2.read()[1]
     imwrite('cam2/'+str(i)+'.jpg',im)
     i+=1
     import time
     time.sleep(0.2)
if __name__=='__main__':
    import code
    code.InteractiveConsole(globals()).interact('')

     
