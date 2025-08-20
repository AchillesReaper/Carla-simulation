import os, cv2

def gen_video(img_dir='./_out/normal_48/img'):
    img_array = []
    for filename in sorted(os.listdir(img_dir)):
        img = cv2.imread(os.path.join(img_dir, filename))
        img_array.append(img)

    out = cv2.VideoWriter(
        img_dir.replace('img', 'video.mp4'),
        cv2.VideoWriter_fourcc(*'mp4v'), 
        20, 
        (img.shape[1], img.shape[0])
    )
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

# gen_video()