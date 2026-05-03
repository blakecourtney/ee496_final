import tensorflow as tf
import pandas as pd
import os
from keras.models import Sequential
from tensorflow.keras import layers
import keras
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from sklearn.metrics import confusion_matrix, classification_report
# import tensorflow_model_optimization as tfmot

SARD_DIR = r'C:\Users\sonia\esp\EE496\drone\tinyML\search-and-rescue\train\images_compressed'
HERIDAL0_DIR = r'C:\Users\sonia\esp\EE496\drone\tinyML\heridal\extracted_images\images0'
HERIDAL1_DIR = r'C:\Users\sonia\esp\EE496\drone\tinyML\heridal\extracted_images\images1'
IMG_SIZE = (128, 128)
BATCH_SIZE = 32

# first combine and scramble the datasets 
df_sard = pd.read_csv(os.path.join(SARD_DIR, 'classification.csv'))
df_sard['full_path'] = df_sard['image_id'].apply(lambda x: os.path.join(SARD_DIR, x))

df_heridal0 = pd.read_csv(os.path.join(HERIDAL0_DIR, 'tiled_classification.csv'))
df_heridal0['full_path'] = df_heridal0['image_id'].apply(lambda x: os.path.join(HERIDAL0_DIR, x))
df_heridal1 = pd.read_csv(os.path.join(HERIDAL1_DIR, 'tiled_classification.csv'))
df_heridal1['full_path'] = df_heridal1['image_id'].apply(lambda x: os.path.join(HERIDAL1_DIR, x))

full_df = pd.concat([df_sard, df_heridal0, df_heridal1]).sample(frac=1).reset_index(drop=True)
full_df['label'] = full_df['label'].astype(str)

# create dataframe
datagen = tf.keras.preprocessing.image.ImageDataGenerator(
    rescale=1./255, 
    validation_split=0.2,
    rotation_range=15, 
    horizontal_flip=True
)

train_set = datagen.flow_from_dataframe(
    full_df, 
    x_col='full_path',
    y_col='label', 
    target_size=IMG_SIZE, 
    class_mode='binary', 
    subset='training',
    directory=None 
)

val_set = datagen.flow_from_dataframe(
    full_df, 
    x_col='full_path', 
    y_col='label', 
    target_size=IMG_SIZE, 
    class_mode='binary', 
    subset='validation',
    directory=None,
    shuffle='False',
)

# check balance 

label_counts = full_df['label'].value_counts()
print("--- Class Distribution ---")
print(label_counts)

total = len(full_df)
for label, count in label_counts.items():
    percentage = (count / total) * 100
    label_text = "Human" if float(label) == 1.0 else "Background"
    print(f"{label_text}: {count} images ({percentage:.2f}%)")

# Check training generator
from collections import Counter

train_labels = train_set.classes
counts = Counter(train_labels)
print("\n--- Training (Actual Images Found) ---")
print(f"Background (0): {counts[0]}")
print(f"Human (1): {counts[1]}")

val_labels = val_set.classes
counts = Counter(val_labels)
print("\n--- Validation (Actual Images Found) ---")
print(f"Background (0): {counts[0]}")
print(f"Human (1): {counts[1]}")


# visualize 
# images, labels = next(train_set)

# print(images)

# plt.figure(figsize=(12, 12))
# for i in range(9): # Show first 9 images
#     ax = plt.subplot(3, 3, i + 1)
#     plt.imshow(images[i])
    
#     title = "Human" if float(labels[i]) == 1.0 else "Background"
#     plt.title(title)
#     plt.axis("off")

# plt.tight_layout()
# plt.show()


# model definition
model = Sequential([
    layers.Input(shape=(128, 128, 3)),
    # Initial downsample
    layers.Conv2D(32, (3,3), strides=2, activation='relu', padding='same'),
    layers.BatchNormalization(),
    
    # Block 1: small features
    layers.SeparableConv2D(64, (3,3), activation='relu', padding='same'),
    layers.BatchNormalization(),
    layers.MaxPooling2D((2,2)),
    
    # Block 2: higher level patterns, larger window size
    layers.SeparableConv2D(128, (3,3), activation='relu', padding='same'),
    layers.BatchNormalization(),
    layers.GlobalAveragePooling2D(),
    
    layers.Dropout(0.4), # Increase dropout to force more robust learning
    layers.Dense(1, activation='sigmoid')
])

optimizer = keras.optimizers.Adam(learning_rate=0.001)
model.compile(loss='binary_crossentropy', optimizer=optimizer, metrics=['accuracy'])


# train model
# model.fit(train_set, validation_data=val_set, epochs=9, )
# model.save("drone_model_v2.h5")
# # model.load_weights(r'C:\Users\sonia\esp\EE496\drone\tinyML\drone_model_v2.h5')

# # check results with confusion matrix
# val_set.shuffle = False
# val_set.index_array = None 
# val_set.reset()

# preds = model.predict(val_set, steps=len(val_set), verbose=1)
# pred_labels = (preds > 0.5).astype(int).flatten()
# true_labels = val_set.classes

# print(f"Predictions: {len(pred_labels)} | True Labels: {len(true_labels)}")

# manual_acc = np.sum(pred_labels == true_labels) / len(true_labels)
# print(f"Manual Calculation Accuracy: {manual_acc:.4f}")
# cm = confusion_matrix(true_labels, pred_labels)

# plt.figure(figsize=(8, 6))
# sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
#             xticklabels=['Background', 'Human'], 
#             yticklabels=['Background', 'Human'])
# plt.ylabel('Actual')
# plt.xlabel('Predicted')
# plt.title('Drone Detection Confusion Matrix')
# plt.show()

# print(classification_report(true_labels, pred_labels, target_names=['Background', 'Human']))


# prune model
# never mind

# quantize model
model = tf.keras.models.load_model('drone_model_v2.h5')

# 2. Create a Representative Dataset Generator
# This pulls a small sample (e.g., 100 images) from your training set
def representative_data_gen():
    it = iter(train_set)
    
    for _ in range(100):
        try:
            # Get the next batch
            img, _ = next(it)
            # Yield the first image of the batch as a 4D tensor
            # (batch_size=1, height=96, width=96, channels=3)
            yield [img[0:1].astype(np.float32)]
        except StopIteration:
            # Restart if we run out of images before 100
            it = iter(train_set)
            img, _ = next(it)
            yield [img[0:1].astype(np.float32)]

# 3. Setup the TFLite Converter
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen

# 4. Force Full Integer Quantization
# This ensures NO floats are left in the model, making it 100% compatible with ESP-NN
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8  # Camera input will be INT8
converter.inference_output_type = tf.int8 # Model output will be INT8

# 5. Convert and Save
tflite_quant_model = converter.convert()

with open('drone_model_quant.tflite', 'wb') as f:
    f.write(tflite_quant_model)