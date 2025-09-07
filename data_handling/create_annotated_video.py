import os
import re
import cv2
import json
import argparse

def natural_sort_key(s):
    # split on digits so "1.jpg", "2.jpg", "10.jpg" sort in human order
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split(r'(\d+)', s)]

def find_json_file(root):
    # look in root, then root/annotations
    for fname in os.listdir(root):
        if fname.lower().endswith('.json'):
            return os.path.join(root, fname)
    ann_dir = os.path.join(root, 'annotations')
    if os.path.isdir(ann_dir):
        for fname in os.listdir(ann_dir):
            if fname.lower().endswith('.json'):
                return os.path.join(ann_dir, fname)
    raise FileNotFoundError(f"No .json annotation found in {root}")

def find_images_dir(root):
    imgs = os.path.join(root, 'images')
    return imgs if os.path.isdir(imgs) else root

def load_annotations(path):
    with open(path) as f:
        return json.load(f)

def main():
    p = argparse.ArgumentParser(
        description="Overlay CVAT export boxes onto frames → MP4")
    p.add_argument('root', help="Path to unzipped CVAT export folder")
    p.add_argument('-o','--output', default='annotated.mp4',
                   help="Output video filename")
    p.add_argument('--fps', type=float, default=30.0,
                   help="Output video frame rate")
    p.add_argument('--thickness', type=int, default=2,
                   help="Line thickness of boxes")
    p.add_argument('--color', default='0,255,0',
                   help="Box color in B,G,R (e.g. 255,0,0)")
    args = p.parse_args()

    ann_path = find_json_file(args.root)
    ann = load_annotations(ann_path)

    imgs_dir = find_images_dir(args.root)
    files = [f for f in os.listdir(imgs_dir)
             if f.lower().endswith(('.jpg','.jpeg','.png'))]
    files.sort(key=natural_sort_key)
    if not files:
        raise RuntimeError(f"No image files in {imgs_dir}")

    # Read first frame to get size
    sample = cv2.imread(os.path.join(imgs_dir, files[0]))
    h, w = sample.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(args.output, fourcc, args.fps, (w,h))

    # parse color
    color = tuple(int(c) for c in args.color.split(','))

    for idx, fname in enumerate(files):
        img = cv2.imread(os.path.join(imgs_dir, fname))
        # try numeric-index key first, then filename key
        shapes = ann.get(str(idx), ann.get(fname, []))
        for obj in shapes:
            # expects obj['bbox'] = [x,y,width,height]
            x,y,ww,hh = map(int, obj.get('bbox', [0,0,0,0]))
            cv2.rectangle(img, (x,y), (x+ww,y+hh), color, args.thickness)
            label = obj.get('label') or obj.get('category_id')
            if label is not None:
                cv2.putText(img, str(label), (x, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, args.thickness)
        writer.write(img)
        if idx>0 and idx%100==0:
            print(f"  → processed {idx}/{len(files)} frames")

    writer.release()
    print(f"✅  Done: saved annotated video to {args.output}")

if __name__=='__main__':
    main()
