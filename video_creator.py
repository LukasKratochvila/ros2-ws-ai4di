import cv2
import os

image_folder = './images'
video_name = 'DEMO_0.avi'

images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
images.sort()
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

fourcc = cv2.VideoWriter_fourcc(*'XVID')

video = cv2.VideoWriter(video_name,
			fourcc,
			fps=15,
			frameSize = (width,height)
			)

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()

