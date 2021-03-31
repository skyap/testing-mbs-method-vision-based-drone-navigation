import cv2
import os
import glob

folder = '190703'
image_folder = glob.glob(os.path.join(folder,"*"))

for i in image_folder:

    video_name = folder+"/"+'video_'+i.split("/")[-1]+'.avi'
    info_image_folder = os.path.join(i,"info")
    
    images = sorted([img for img in os.listdir(info_image_folder) if img.endswith(".jpg")])
    if len(images)==0:
        continue
    frame = cv2.imread(os.path.join(info_image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 1, (width,height))

    for image in images:
        video.write(cv2.imread(os.path.join(info_image_folder, image)))

# cv2.destroyAllWindows()
    video.release()