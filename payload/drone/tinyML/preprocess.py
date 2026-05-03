# preprocess SARD data
# first compress to 96x96 pixels and see if it will work, still RGB

import pandas as pd
import os
from PIL import Image
import csv
import cv2
import matplotlib.pyplot as plt


INPUT_FOLDER = r'C:\Users\sonia\esp\EE496\drone\tinyML\search-and-rescue\train\images'
LABEL_FOLDER = r'C:\Users\sonia\esp\EE496\drone\tinyML\search-and-rescue\train\labels'
OUTPUT_FOLDER = r'C:\Users\sonia\esp\EE496\drone\tinyML\search-and-rescue\train\images_compressed'
HERICAL_FOLDER = r'C:\Users\sonia\esp\EE496\drone\tinyML\heridal\extracted_images\images1'
OUTPUT_CSV   = r'C:\Users\sonia\esp\EE496\drone\tinyML\search-and-rescue\train\classification.csv'
TARGET_SIZE = (128, 128)
QUALITY = 85

def process_images():
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
        print(f"Created folder: {OUTPUT_FOLDER}")

    for filename in os.listdir(INPUT_FOLDER):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            try:
                img_path = os.path.join(INPUT_FOLDER, filename)
                with Image.open(img_path) as img:
                    
                    img = img.convert('RGB')
                    
                    img_resized = img.resize(TARGET_SIZE, Image.Resampling.LANCZOS)
                    
                    save_path = os.path.join(OUTPUT_FOLDER, filename)
                    img_resized.save(save_path, 'JPEG', quality=QUALITY, optimize=True)
                    
                print(f"Processed: {filename}")
                
            except Exception as e:
                print(f"Could not process {filename}: {e}")

    print("\nBatch Processing Complete!")

def create_classification_csv():
    dataset = []
    
    image_files = [f for f in os.listdir(INPUT_FOLDER) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    # print(f"Found {len(image_files)} images. Processing labels...")

    for img_name in image_files:
        basename = os.path.splitext(img_name)[0]
        label_path = os.path.join(LABEL_FOLDER, basename + '.txt')
        
        is_human = 0
        
        if os.path.exists(label_path):
            with open(label_path, 'r') as f:
                content = f.read().strip()
                if content:
                    is_human = 1
        
        dataset.append([img_name, is_human])

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['image_id', 'label']) # Header
        writer.writerows(dataset)

    print(f"Done CSV saved as: {OUTPUT_CSV}")


def check_balance():
    if not os.path.exists(OUTPUT_CSV):
        print(f"Error: Could not find {OUTPUT_CSV}")
        return

    # Load the CSV
    df = pd.read_csv(OUTPUT_CSV)

    # Check if 'label' column exists
    if 'label' not in df.columns:
        print("Error: 'label' column not found in CSV.")
        return

    # Count values
    counts = df['label'].value_counts()
    
    # Calculate percentages
    total = len(df)
    zeros = counts.get(0, 0)
    ones = counts.get(1, 0)
    
    perc_0 = (zeros / total) * 100
    perc_1 = (ones / total) * 100

    print(f"Class 0 (No Human): {zeros:d} ({perc_0:.2f}%)")
    print(f"Class 1 (Human):    {ones:d} ({perc_1:.2f}%)")
    print(f"Total Images:       {total:d}")


def play_with_preprocessing(directory_path, num_images=5):
    # 1. Update to your specific tiled CSV name
    csv_path = os.path.join(directory_path, 'classification.csv')
    if not os.path.exists(csv_path):
        print(f"Error: Could not find classification.csv in {directory_path}")
        return
        
    df = pd.read_csv(csv_path)
    sample_df = df.head(num_images)
    
    # Setup plotting: num_images rows, 8 columns (increased from 5)
    fig, axes = plt.subplots(len(sample_df), 8, figsize=(24, 3 * len(sample_df)))
    
    for i, (idx, row) in enumerate(sample_df.iterrows()):
        img_filename = row['image_id']
        img_path = os.path.join(directory_path, 'images', img_filename)
        
        label_val = str(row['label'])
        label_text = "HUMAN" if label_val in ['1', '1.0'] else "BACKGROUND"
        label_color = 'red' if label_text == "HUMAN" else 'black'

        img = cv2.imread(img_path)
        if img is None:
            img_path = os.path.join(directory_path, img_filename)
            img = cv2.imread(img_path)
            if img is None:
                continue
            
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # --- FILTERS ---
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        clahe_obj = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        contrast = clahe_obj.apply(gray)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(gray, 100, 200)

        # --- RGB SPLITTING ---
        # R, G, and B components as separate grayscale-style visualizations
        R = img_rgb[:, :, 0]
        G = img_rgb[:, :, 1]
        B = img_rgb[:, :, 2]

        # --- PLOTTING ---
        row_images = [img_rgb, gray, contrast, blurred, edges, R, G, B]
        titles = ['Original', 'Grayscale', 'CLAHE', 'Blurred', 'Edges', 'Red Ch', 'Green Ch', 'Blue Ch']
        
        for j in range(8):
            ax = axes[i, j] if len(sample_df) > 1 else axes[j]
            
            # Use 'gray' colormap for everything except the first 'Original' RGB image
            cmap = 'gray' if j > 0 else None
            ax.imshow(row_images[j], cmap=cmap)
            
            # Label on the far left
            if j == 0:
                ax.set_ylabel(label_text, fontsize=12, fontweight='bold', color=label_color)
            
            # Titles on the top row
            if i == 0:
                ax.set_title(titles[j], fontsize=12)
                
            ax.set_xticks([])
            ax.set_yticks([])

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    process_images()
    create_classification_csv()
    # check_balance()
    # play_with_preprocessing(OUTPUT_FOLDER, num_images=6)



# data_augmentation = tf.keras.Sequential([
#   tf.keras.layers.RandomFlip("horizontal_and_vertical"),
#   tf.keras.layers.RandomRotation(0.2),
#   tf.keras.layers.RandomContrast(0.1), # Drones face varying light
# ])