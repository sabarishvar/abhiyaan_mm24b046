To evaluate the efficiency of different lane detection methods, we will compare machine learning approaches with classical techniques. The goal is to determine which method is more effective in terms of accuracy, robustness, and computational efficiency.

## 1. Machine Learning-Based Lane Detection Methods

ML-based approaches leverage deep learning models to detect lanes with high accuracy. Below are some commonly used models:

### A. LaneNet (Deep Learning-Based Lane Detection)
- **Architecture**: Uses an encoder-decoder network with a segmentation branch and an embedding branch.  
- **Advantages**:  
  - Learns lane features automatically from data.  
  - Works in conditions like shadows and other complex scenarios too
  - High accuracy compared to classical methods.  
- **Disadvantages**:  
  - Requires a large amount of labeled training data.(trained files which are complex to upload)  
  - Computationally expensive compared to traditional approaches.  

### B. SCNN (Spatial central neural network)
- **Architecture**: Uses convolutional layers with spatial propagation to improve lane detection accuracy.  
- **Advantages**:  
  - Captures lane structures better than traditional CNNs.  
  - Works well in occluded and challenging conditions.  
- **Disadvantages**:  
  - Requires powerful GPUs for real-time inference.  
  - Sensitive to dataset quality.  

### C. PolyLaneNet
- **Architecture**: Uses a polynomial regression model to predict lane structures.  
- **Advantages**:  
  - More efficient than pixel-wise segmentation models.  
  - Reduces computational cost.
  - It represents the lane in a curvy nature and not in straight lines.  
- **Disadvantages**:  
  - May struggle with complex road structures.  

## 2. Classical Lane Detection Techniques (Non-ML)

Classical methods use image processing techniques like denoising the image and edge detection and hough detection to detect lanes based on edge features and line detection algorithms.

### A. Canny Edge Detection + Hough Transform
- **Process**:
  - Apply **Gaussian blur** to remove noise.
  - Use **Canny Edge Detection** to detect lane edges.
  - Apply **Hough Transform** to detect lane lines.
- **Advantages**:
  - Computationally lightweight.
  - Works well on clear lane markings.
- **Disadvantages**:
  - Fails in low-light conditions, occlusions, or curved lanes.
  - Sensitive to noise and road artifacts.  

### B. Sobel + Region of Interest Masking
- **Process**:
  - Apply **Sobel operator** to extract edges.
  - Mask the region of interest to focus on lane areas.
  - Use contour detection or line fitting techniques.  
- **Advantages**:
  - Faster than ML-based methods.
  - Works in structured environments with well-defined lanes.  
- **Disadvantages**:
  - Sensitive to lighting variations.
  - Struggles in complex urban scenarios.  


## 3. Data-Backed Conclusion
- **ML models** (especially **LaneNet and SCNN**) outperform classical methods in terms of accuracy and robustness, particularly in challenging conditions (occlusions, shadows, night driving).  
- **Classical methods** (specifically ** Canny Edge & Hough Transform) are computationally cheaper and can run in real-time but struggle with complex lane scenarios.  
- **PolyLaneNet** provides a good trade-off between efficiency and accuracy.  
- **For real-time applications on embedded systems**, lightweight ML models or optimized classical methods are preferable.  

## Comparision of ML and Classical Techniques:
|Criteria|LaneNet|SCNN|PolyLaneNet|Canny+Hough(classical)|Sobel+ROI Masking(Classical)|
|--------|-------|----|-----------|----------------------|----------------------------|
|Accuracy|High|Very High|Moderate|Low|Moderate|
|Robustness|High|Very High|Moderate|Low|Moderate|
|Computational Cost|High|Very High|Moderate|Low|Moderate|
|Data Requirement|High|Very High|Moderate|Low|Low|
|Performance in Occulusions|High|High|Moderate|Poor|Poor|
|Real time processing|Possible with optimisation|Slow|Efficient|Very Fast|Fast|
|Adaptability|Learns from data|Learns from data|Learns from data|Limited|Limited|
