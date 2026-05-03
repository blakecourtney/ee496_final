# SARD dataset is very unbalanced, create more datapoints through HERIDAL dataset
# images are a lot more zoomed out so we can crop them and label them depending on the bounding box information
# also compress to 96x96 pixels

import os
import pandas as pd
from PIL import Image
import random

INPUT_IMG_DIR = r'C:\Users\sonia\esp\EE496\drone\tinyML\heridal\train'
INPUT_CSV = r'C:\Users\sonia\esp\EE496\drone\tinyML\heridal\train\_annotations.csv'
OUTPUT_DIR = r'C:\Users\sonia\esp\EE496\drone\tinyML\heridal\extracted_images'
TILE_SIZE = 400 
FINAL_RESIZE = (128, 128)
BG_KEEP_PROB = 0.05

def tile_dataset():
    df = pd.read_csv(INPUT_CSV, names=['filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax'])

    img_out_dir = os.path.join(OUTPUT_DIR, 'images0')    
    # img_out_dir = os.path.join(OUTPUT_DIR, 'images1')
    os.makedirs(img_out_dir, exist_ok=True)
    
    new_data = []
    unique_images = df['filename'].unique()

    for img_name in unique_images:
        img_path = os.path.join(INPUT_IMG_DIR, img_name)
        if not os.path.exists(img_path):
            continue

        with Image.open(img_path) as img:
            w, h = img.size
            img_boxes = df[df['filename'] == img_name]

            for y in range(0, h - TILE_SIZE, TILE_SIZE):
                for x in range(0, w - TILE_SIZE, TILE_SIZE):
                    
                    # 1. Check if any human box center falls in this tile
                    is_human = 0
                    for _, box in img_boxes.iterrows():
                        mid_x = (int(box['xmin']) + int(box['xmax'])) / 2
                        mid_y = (int(box['ymin']) + int(box['ymax'])) / 2
                        
                        if (x <= mid_x < x + TILE_SIZE) and (y <= mid_y < y + TILE_SIZE):
                            is_human = 1
                            # if random.random() > BG_KEEP_PROB:
                            #     break

                            # tile_name = f"{os.path.splitext(img_name)[0]}_{x}_{y}.jpg"
                            # tile = img.crop((x, y, x + TILE_SIZE, y + TILE_SIZE))
                            
                            # tile = tile.resize(FINAL_RESIZE, Image.Resampling.LANCZOS)
                            # tile.save(os.path.join(img_out_dir, tile_name), quality=90)
                            
                            # new_data.append([tile_name, is_human])
                            break
                        if random.random() > BG_KEEP_PROB:
                            break
                        
                        tile_name = f"{os.path.splitext(img_name)[0]}_{x}_{y}.jpg"
                        tile = img.crop((x, y, x + TILE_SIZE, y + TILE_SIZE))
                        
                        tile = tile.resize(FINAL_RESIZE, Image.Resampling.LANCZOS)
                        tile.save(os.path.join(img_out_dir, tile_name), quality=90)
                        
                        new_data.append([tile_name, is_human])

    # Save the new Classification CSV
    pd.DataFrame(new_data, columns=['image_id', 'label']).to_csv(
        os.path.join(OUTPUT_DIR, 'images0','tiled_classification.csv'), index=False
    )
    print(f"\nSUCCESS: Saved to {OUTPUT_DIR}")


if __name__ == "__main__":
    tile_dataset()